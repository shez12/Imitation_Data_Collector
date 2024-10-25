import sys
import rospy
import time
from spatialmath import SE3, SO3
from sensor_msgs.msg import JointState
from omni_msgs.msg import OmniButtonEvent
from geometry_msgs.msg import PoseStamped
import numpy as np

sys.path.append("/home/hanglok/work/ur_slam")
from ik_step import init_robot
import ros_utils
import ros_utils.myIO
from utils import pose_util



class MyOmni():
    def __init__(self):
        self.gray_button = 0
        self.white_button = 0
        self.gray_button_flag = False 
        self.white_button_flag = False
        # Create subscribers for the button and joint states
        rospy.Subscriber('/phantom/phantom/button', OmniButtonEvent, self.button_callback)
        rospy.Subscriber('/phantom/end_effector_pose', PoseStamped, self.pose_callback)
        rospy.sleep(0.5)

    def button_callback(self, data):
        # Process the button event
        print(data)
        if data.grey_button == 1:
            self.gray_button_flag = not self.gray_button_flag
            print("gray button pressed")
        if data.white_button == 1:
            self.white_button_flag = not self.white_button_flag
            print("white button pressed")




    def pose_callback(self, data):
        pos = data.pose.position
        ori = data.pose.orientation
        self.pose = np.array([pos.x, pos.y, pos.z,  ori.z, ori.x, ori.y, ori.w,])

        



def get_quat_diff(pose_current, pose_past):
    '''
    move_robot from past pose to current pose
    '''
    R_curr = pose_util.quat_to_R(pose_current[3:7])
    R_past = pose_util.quat_to_R(pose_past[3:7])

    
    SO3_past = SO3(R_past)
    SO3_curr = SO3(R_curr)

    SO3_diff = SO3_curr * SO3_past.inv()
    
    return SO3_diff.R


def execute_command(pose_current, pose_past, t_move=0.005):
    '''
    move_robot
    '''
    ori_diff = get_quat_diff(pose_current, pose_past)
    pos_diff = pose_current[:3] - pose_past[:3]
    pos_diff[0], pos_diff[1] = -pos_diff[1], pos_diff[0]#axis direction fix

    tcp_move = [i * t_move for i in pos_diff]
    move_SE3 = np.eye(4)
    move_SE3[:3, :3] = ori_diff
    move_SE3[:3, 3] = tcp_move
    move_SE3 = SE3(move_SE3)
    # print("moving to \n",move_SE3)
    return move_SE3




if __name__ == "__main__":
    rospy.init_node('omni_controller', anonymous=True)
    robot = init_robot("robot1")
    omni = MyOmni()
    pose_past = None
    print("start.....................")
    while True:
        if omni.gray_button_flag:
            print("moving...")
            if pose_past is not None:
                pose_current = omni.pose
                action=execute_command(pose_current, pose_past)
                robot.step(action,wait=False)
                pose_past = pose_current
            else:
                pose_past = omni.pose

        else:
            pose_past = None
            move_SE3 = SE3(np.eye(4))
            robot.step(move_SE3, wait=False)
            
