import numpy as np
import collections
import matplotlib.pyplot as plt
import IPython
e = IPython.embed
import sys
import time
from spatialmath import SE3, SO3
import rospy
import threading
import dm_env
from robot_utils import Recorder, ImageRecorder
from controller import XboxController, control_robot

sys.path.append("/home/hanglok/work/ur_slam")
from utils.pose_util import *
import ros_utils.myGripper
from ik_step import init_robot
import ros_utils.myIO
sys.path.append("/home/hanglok/work/SBT/src/sbt_sensor/scripts")
import sbt_sensor_force  



class RealEnv:
    """
    Environment for real robot bi-manual manipulation
    Action space:      [arm (6),             # absolute joint position
                        gripper (1),    # normalized gripper position (0: close, 1: open)] (*2 if two arms)

    Observation space: {"qpos": Concat[ arm (6),          # absolute joint position
                                        gripper (1),  # normalized gripper position (0: close, 1: open)(*2 if two arms)
                        "qvel": Concat[ arm (6),         # absolute joint velocity (rad)
                                        gripper (1),  # normalized gripper velocity (pos: opening, neg: closing)(*2 if two arms)
                        "images": {"camera1": (480x640x3),        # h, w, c, dtype='uint8'
                                   "camera2": (480x640x3),         # h, w, c, dtype='uint8'
                                   ....}
    """

    def __init__(self, robot_names,camera_names = ['camera1', 'camera2'],ee_list = None):
        rospy.init_node('ik_step', anonymous=True)
        self.robot_names = robot_names
        self.robots = {}
        self.robot_infos = {}
        self.ee_list = ee_list
        for i, name in enumerate(robot_names):
            self.robots[name] = init_robot(name)
            self.robot_infos[name] = Recorder(name, init_node=False)
        self.image_recorder = ImageRecorder(init_node=False,camera_names = camera_names)

        # add controller
        self.controllers = {}
        for i in range(len(robot_names)):
            self.controllers[i] = XboxController(i)
        self.gripper = ros_utils.myGripper.MyGripper()
        self.IO = ros_utils.myIO.MyIO(fun=1,pin=16)
        self.sbt_force = sbt_sensor_force.sbt_sensor()
        time.sleep(2)

        # Wait for the first gripper state message
        # rospy.wait_for_message('/gripper/states', ros_utils.myGripper.GripperState, timeout=5)


    def get_qpos(self,robot_names,ee_list):
        '''
        args:
            robot_names: list of str
            ee_type: str, "gripper" or "vacuum"
        return:
            qpos: np.ndarray, shape (7,)
        '''
        qpos = []
        for i,robot_name in enumerate(robot_names):
            robot_qpos_raw = self.robot_infos[robot_name].qpos
            if "gripper" in ee_list[i]:
                gripper_action = self.gripper.current_position
                qpos.append(np.append(robot_qpos_raw,gripper_action))
            elif "vacuum" in ee_list[i] :
                IO_state = self.IO.state
                qpos.append(np.append(robot_qpos_raw,IO_state))
            elif "force" in ee_list[i]:
                qpos.append(np.append(robot_qpos_raw,self.sbt_force.force))
            else:
                raise ValueError(f"Invalid ee_type: {ee_list[i]}")
        return np.concatenate(qpos)

    def get_qvel(self,robot_names):
        qvel = []
        for robot_name in robot_names:
            robot_qvel_raw = self.robot_infos[robot_name].qvel
            qvel.append(np.append(robot_qvel_raw,0))
        return np.concatenate(qvel)

    def get_data(self,robot_names):
        data = []
        for robot_name in robot_names:
            data.append(self.robot_infos[robot_name].data)
        return data


    def get_images(self):
        return self.image_recorder.get_images()

    def get_observation(self):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos(self.robot_names,self.ee_list)
        obs['qvel'] = self.get_qvel(self.robot_names)
        obs['images'] = self.get_images()
        return obs

    def get_reward(self):
        return 0
    
    # def reset(self):
    #     home_pose = [0.04623123636528992, -0.01421597361326151, -0.09686455178674844, -0.6738959787541939, -0.22870789441255168, 0.6808505630333548, -0.1732034054788412]
    #     pose = pose_to_SE3(home_pose)
    #     self.robot.goto_pose(pose,wait=False)


    def start(self):
        print("start")
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation())
    
    
    def step(self, command=None, robot_names=None, ee_list=None):
        '''
        Designed to control multi robots depends on robot_name and ee_list
        args:
            command: if None, control robot by controller; if not None, control robot by command
            robot_names: list of str
            ee_list: list of str, ["gripper,vacuum", "vacuum","force"]
        '''
        if ee_list is None:
            raise ValueError("ee_list is None")

        if command is None:
            actions = [None] * len(robot_names)
            threads = []

            def control_robot_thread(i, robot_name):
                gripper =None
                IO_control = None
                force = None
                if "gripper" in ee_list[i] :
                    gripper = self.gripper
                    # actions[i] = control_robot(self.robots[robot_name], self.controllers[i], gripper=self.gripper)
                if "vacuum" in ee_list[i] :
                    IO_control = self.IO
                    # actions[i] = control_robot(self.robots[robot_name], self.controllers[i], IO_control=self.IO)
                if "force" in ee_list[i]  :
                    force =self.sbt_force.force
                # print("force is ",force)
                actions[i] = control_robot(self.robots[robot_name], self.controllers[i],gripper=gripper,IO_control=IO_control,force = force)

            for i, robot_name in enumerate(robot_names):
                thread = threading.Thread(target=control_robot_thread, args=(i, robot_name))
                thread.start()
                threads.append(thread)

            for thread in threads:
                thread.join()

            action = np.concatenate(actions)
        else:
            threads = []

            def move_robot_thread(i, robot_name):
                if "gripper" in ee_list[i]  :
                    self.robots[robot_name].move_joints(command[i*7:(i+1)*7-1])
                    self.gripper.set_gripper(command[i*7+6], force=2)
                if "vacuum" in ee_list[i]  :
                    self.robots[robot_name].move_joints(command[i*7:(i+1)*7-1])
                    self.IO.set_io_state(command[i*7+6])
                if "force" in ee_list[i]:
                    self.robots[robot_name].move_joints(command[i*7:(i+1)*7-1])

            for i, robot_name in enumerate(robot_names):
                thread = threading.Thread(target=move_robot_thread, args=(i, robot_name))
                thread.start()
                threads.append(thread)

            for thread in threads:
                thread.join()

            action = None
        
        # time.sleep(0.02)
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=self.get_reward(),
            discount=None,
            observation=self.get_observation()), action

# if __name__ == "__main__":
#     rospy.init_node("test")
#     env = RealEnv("robot1")
#     home_pose = [0.04623123636528992, -0.01421597361326151, -0.09686455178674844, -0.6738959787541939, -0.22870789441255168, 0.6808505630333548, -0.1732034054788412]
#     pose = pose_to_SE3(home_pose)
#     env.robot.goto_pose(pose,wait=False)