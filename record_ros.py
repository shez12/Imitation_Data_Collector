import rospy
from sensor_msgs.msg import Image
from control_msgs.msg import JointTrajectoryControllerState
from message_filters import ApproximateTimeSynchronizer, Subscriber
import signal
import sys
import time
import h5py
import cv2
import numpy as np
import os
from cv_bridge import CvBridge

class info_saver:
    def __init__(self, frequency, robot_name):
        try:
            rospy.init_node('sync_listener', anonymous=True)
            print("Node initialized")
        except:
            print("ROS node has been initialized")
        self.data = {
            'observations':{
                'images':{
                    'rgb_1': [], 'depth_1': [],
                    'rgb_2': [], 'depth_2': [],
                },
                'qpos':[],
                'qvel':[]  # Add this line
            },
            'action':[]
        }
        self.frequency = frequency
        self.data_collected = False
        self.last_message_time = rospy.Time.now()
        self.robot_name = robot_name
        self.bridge = CvBridge()


    def info_save(self, image_msg1, image_msg2, joint_state_msg, depth_msg1, depth_msg2):
        self.last_message_time = rospy.Time.now()
        print("Received synchronized messages")
        # Extract timestamp
        timestamp = joint_state_msg.header.stamp
        print(timestamp.to_sec())
        # Extract position and velocity

        self.data["observations"]["qpos"].append(joint_state_msg.actual.positions)
        self.data["observations"]["qvel"].append(joint_state_msg.actual.velocities)
        # Add data for camera1
        cv_image1 = self.imgmsg_to_cv2(image_msg1)  # Convert to OpenCV image
        cv_depth1 = self.imgmsg_to_cv2(depth_msg1)  # Convert to OpenCV image
        self.data["observations"]["images"]["rgb_1"].append(cv_image1)
        self.data["observations"]["images"]["depth_1"].append(cv_depth1)
        # Add data for camera2
        cv_image2 = self.imgmsg_to_cv2(image_msg2)  # Convert to OpenCV image
        cv_depth2 = self.imgmsg_to_cv2(depth_msg2)  # Convert to OpenCV image
        self.data["observations"]["images"]["rgb_2"].append(cv_image2)
        self.data["observations"]["images"]["depth_2"].append(cv_depth2)
        self.data_collected = True
        print(f"Data collected: {len(self.data['observations']['qpos'])} samples")
        rospy.sleep(1/self.frequency)
    

    def add_actions(self):
        self.data["action"] = [np.array(qpos, dtype=np.float32) for qpos in self.data["observations"]["qpos"][1:]]
        self.data["action"].append(np.array(self.data["observations"]["qpos"][-1], dtype=np.float32))


    def save_all_info(self, file_path, num):
        self.add_actions()
        os.makedirs(file_path, exist_ok=True)
        data_path = file_path + '/episode_' + str(num) + '.hdf5'
        with h5py.File(data_path, 'w') as f:
            for key, value in self.data.items():
                if key == 'observations':
                    obs_group = f.create_group('observations')
                    for obs_key, obs_value in value.items():
                        if obs_key == 'images':
                            img_group = obs_group.create_group('images')
                            for img_key, img_value in obs_value.items():
                                img_group.create_dataset(img_key, data=np.array(img_value))
                        else:
                            obs_group.create_dataset(obs_key, data=np.array(obs_value, dtype=np.float32))
                else:
                    f.create_dataset(key, data=np.array(value, dtype=np.float32))
        print(f"Data saved in hdf5: {data_path}")



    def imgmsg_to_cv2(self, img_msg):
        # Convert ROS Image message to OpenCV format
        output = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        if len(output.shape) == 2:  # If it's a depth image
            return output
        else:  # If it's an RGB image
            return cv2.cvtColor(output, cv2.COLOR_BGR2RGB)

    def listener(self):
        # camera1
        image_sub1 = Subscriber('/camera1/color/image_raw', Image)
        depth_sub1 = Subscriber('/camera1/depth/image_rect_raw', Image)
        # camera2
        image_sub2 = Subscriber('/camera2/color/image_raw', Image)
        depth_sub2 = Subscriber('/camera2/depth/image_rect_raw', Image)

        self.image_sub1 = image_sub1
        self.depth_sub1 = depth_sub1
        self.image_sub2 = image_sub2
        self.depth_sub2 = depth_sub2

        # Replace the topic existence checks with this function
        def topic_exists(topic_name):
            return any(topic_name in topic for topic, _ in rospy.get_published_topics())

        # Check if the topics exist
        robot_topic = '/' + self.robot_name + '/scaled_pos_joint_traj_controller/state'
        if not topic_exists(robot_topic):
            raise KeyError(f"{robot_topic} Topic does not exist")  
        else:
            print(f"{robot_topic} Topic exists")
        
        if not topic_exists('/camera1/color/image_raw'):
            raise KeyError("/camera1/...  Topic does not exist")
        else:
            print("/camera1 Topic exists")
        
        if not topic_exists('/camera2/color/image_raw'):
            raise KeyError("/camera2 Topic does not exist")
        else:
            print("/camera2 Topic exists")

        Subscriber_name ='/'+ self.robot_name + '/scaled_pos_joint_traj_controller/state'
        joint_state_sub = Subscriber(Subscriber_name, JointTrajectoryControllerState)
        self.joint_state_sub = joint_state_sub

        rospy.loginfo("Subscribers created")
        # 使用ApproximateTimeSynchronizer同步话题
        ats = ApproximateTimeSynchronizer([image_sub1, image_sub2, joint_state_sub, depth_sub1, depth_sub2], queue_size=10, slop=0.1)
        ats.registerCallback(self.info_save)
        
        print("Callback registered")
    
    def stop_subscriptions(self):
        # Unsubscribe from all topics
        self.image_sub1.unregister()
        self.depth_sub1.unregister()
        self.image_sub2.unregister()
        self.depth_sub2.unregister()
        self.joint_state_sub.unregister()
        print("Unsubscribed from all topics")
        


def signal_handler(sig, frame):
    print("Ctrl+C pressed. Saving data...")
    # Save the data
    time_str = time.strftime("%Y%m%d_%H%M%S")
    file_path = os.path.join('data', time_str)
    if info_saver.data_collected:
        info_saver.save_all_info(file_path,0)
        # info_saver.save_image_jpg(file_path)
        print("Data saved. Exiting...")
    else:
        print("No data collected. Exiting without saving.")
    sys.exit(0)




if __name__ == "__main__":
    info_saver = info_saver(10,"robot2")
    info_saver.listener()
    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()