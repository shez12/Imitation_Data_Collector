import sys
import rospy
import pygame
import time
from spatialmath import SE3, SO3


sys.path.append("/home/hanglok/work/ur_slam")
import ros_utils.myGripper
from ik_step import init_robot
import ros_utils
import ros_utils.myIO




class XboxController:
    def __init__(self,num=0):
        # Initialize Pygame and the joystick subsystem
        pygame.init()
        pygame.joystick.init()
        
        # Check if a joystick/controller is connected
        self.joystick_count = pygame.joystick.get_count()
        
        if self.joystick_count == 0:
            print("No joystick/controller found.")
            raise KeyError("No joystick/controller found.")
        
        # Initialize the first connected joystick (assumed to be the Xbox controller)
        self.joystick = pygame.joystick.Joystick(num)
        self.joystick.init()

        print(f"Choose controller: {self.joystick.get_name()}")
    

    def capture_input(self):

        pygame.event.pump()  # Process event queue

        #record inputs
        self.inputs = {}

        # Capture button inputs
        self._capture_buttons()

        # Capture axis inputs (e.g., analog sticks)
        self._capture_axes()

        # Capture D-Pad (hat switch) inputs
        self._capture_hats()

        # Sleep to reduce spamming of output
        time.sleep(0.1)


    def _capture_buttons(self):
        # Loop through all buttons and check their states
        for button in range(self.joystick.get_numbuttons()):
            button_state = self.joystick.get_button(button)
            if button_state:  # Only print if the button is pressed
                # print(f"Button {button} pressed")
                name = "b_"+ str(button)
                self.inputs[name] = button_state

    def _capture_axes(self):
        # Loop through all axes (analog sticks) and capture their values
        for axis in range(self.joystick.get_numaxes()):
            axis_value = self.joystick.get_axis(axis)
            # Only print if the axis moves significantly (with dead zone filtering)
            if abs(axis_value) > 0.1:
                if not (axis_value == -1.00 and axis in [2,5]): 
                    # print(f"Axis {axis} value: {axis_value:.2f}")
                    name = "a_"+ str(axis)
                    self.inputs[name] = axis_value

    def _capture_hats(self):
        # Loop through all hats (D-Pad) and capture their values
        for hat in range(self.joystick.get_numhats()):
            hat_value = self.joystick.get_hat(hat)
            if hat_value != (0, 0):  # Only print if the D-Pad is pressed
                # print(f"Hat {hat} value: {hat_value}")
                name = "h_"+ str(hat)
                self.inputs[name] = hat_value


def input_translator(input_dict):
    # Translate the inputs into a more readable format

    
    tcp_x = 0
    tcp_y = 0
    tcp_z = 0
    rotate_clockwise = 1
    tcp_rx = 0
    tcp_ry = 0
    tcp_rz = 0
    io_state = -1
    gripper_value = -1

    if not input_dict:
        # print("No input provided.")
        tcp_move = [tcp_x, tcp_y, tcp_z]
        tcp_rotate = [tcp_rx, tcp_ry, tcp_rz]
        return tcp_move, tcp_rotate,io_state,gripper_value # Return default values

    for key, value in input_dict.items():
        if key[0] == "b":
            '''
            Button inputs
            b_0: A  ---> turn on motor
            b_1: B  ---> turn off motor
            b_2: X  ---> hold on rotate anticlockwise
            b_3: Y
            '''
            if key[0] == "b":
                if key[2] == "0":
                    # turn on motor
                    io_state = 1
                elif key[2] == "1":
                    # turn off motor
                    io_state = 0
                elif key[2] == "2":
                    rotate_clockwise = -1
                    pass
                elif key[2] == "3":

                    pass

            pass
        elif key[0] == "a":
            '''
            axis inputs
            a_0: Left stick X-axis  ---> tcp X-axis
            a_1: Left stick Y-axis  ---> tcp y-axis
            a_2: Left trigger
            a_3: Right stick X-axis 
            a_4: Right stick Y-axis ----> tcp z-axis
            a_5: Right trigger
            '''
            if key[0] == "a":
                if key[2] == "0":
                    tcp_y += value
                elif key[2] == "1":
                    tcp_x += value
                elif key[2] == "4":
                    tcp_z -= value
                elif key[2] == "5":
                    gripper_value = max(-1,value)
        elif key[0] == "h":
            '''
            hat inputs
            h_0:[0,1] D-Pad ---> rotate tcp along x-axis
            h_0:[-1,0] D-Pad ---> rotate tcp along y-axis
            h_2:[0,-1] D-Pad ---> rotate tcp along z-axis
            
            '''
            if value == (0,1):
                # rotate tcp along x-axis
                tcp_rx += rotate_clockwise
    
            elif value == (-1,0):
                # rotate tcp along y-axis
                tcp_ry += rotate_clockwise

            elif value == (0,-1):
                # rotate tcp along z-axis
                tcp_rz += rotate_clockwise


            pass  

    tcp_move = [tcp_x, tcp_y, tcp_z]
    tcp_rotate = [tcp_rx, tcp_ry, tcp_rz]
    return tcp_move, tcp_rotate, io_state, gripper_value
    

def execute_command(tcp_move, tcp_rotate, t_move=0.02, r_move=5):
    '''
    move_robot
    '''
    tcp_move = [i * t_move for i in tcp_move]
    tcp_rotate = [i * r_move for i in tcp_rotate]

    if abs(sum(tcp_move)) > 0:
        move_SE3 = SE3.Tx(tcp_move[0]) @ SE3.Ty(tcp_move[1]) @ SE3.Tz(tcp_move[2])
    elif abs(sum(tcp_rotate)) > 0:
        move_SE3 = SE3.Rx(tcp_rotate[0],unit='deg') @ SE3.Ry(tcp_rotate[1],unit='deg') @ SE3.Rz(tcp_rotate[2],unit='deg')
    else:
        move_SE3 = SE3.Tx(0)
    return move_SE3


def set_IO(state):
    # Initialize fun and pin here
    fun = 1
    pin = 16
    my_io = ros_utils.myIO.MyIO(fun, pin)
    my_io.set_io_state(state)


def control_robot(robot, controller,gripper=None,IO_control = None,force = None):
    '''
    robot: robot object
    controller: controller object
    gripper: control gripper class; if no need then None
    IO_control: control IO class; if no need then None
    '''
    
    if controller.joystick_count > 0:
        controller.capture_input()
        if controller.inputs != {}:
            print(controller.inputs)
        tcp_move, tcp_rotate, io_state, gripper_value = input_translator(controller.inputs)
        print(f"tcp move {tcp_move}, rotate {tcp_rotate}, {io_state}, {gripper_value}")
        if controller.inputs.get("b_7") == 1:
            print("Exiting...")
            exit()
        # if sum(tcp_move) + sum(tcp_rotate) !=0:
        if force is not None and abs(force) > 0.15:
            print("force is too big")
            # move upwards
            actions =robot.step(action=execute_command([0,0,0.5], [0,0,0]), wait=False,return_joints=True)
            #TODO impendence control, change orientiation
        else:
            actions =robot.step(action=execute_command(tcp_move, tcp_rotate), wait=False,return_joints=True)
        actions = list(actions)
        if IO_control is not None:
            if io_state != -1:
                set_IO(io_state)
            actions.append(io_state)

        if gripper is not None:
            value = 500*(1-gripper_value)
            if gripper_value >= -1:
                gripper.set_gripper(value, 20.0)
            actions.append(value)

        if gripper is None and IO_control is None:
            actions.append(0)
            return actions
        
                
        return actions
        

# def main():
#     # Initialize the ROS node
#     rospy.init_node('xbox_controller', anonymous=True)
#     robot = init_robot("robot1")
#     gripper = ros_utils.myGripper.MyGripper()
#     controller = XboxController()
#     if controller.joystick_count > 0:
#         while not rospy.is_shutdown():
#             controller.capture_input()
#             if controller.inputs != {}:
#                 print(controller.inputs)
#             tcp_move, tcp_rotate, io_state, gripper_value = input_translator(controller.inputs)
#             print(f"tcp move {tcp_move}, rotate {tcp_rotate}, {io_state}, {gripper_value}")
#             if controller.inputs.get("b_7") == 1:
#                 print("Exiting...")
#                 break
#             if io_state != -1:
#                 set_IO(io_state)
#             if sum(tcp_move) + sum(tcp_rotate) !=0:
#                 robot.step(action=execute_command(tcp_move, tcp_rotate), wait=False)
#             if gripper_value >= -1:
#                 gripper.set_gripper(500*(1-gripper_value), 20.0)       
#             # time.sleep(0.1)


if __name__ == "__main__":
    # main()
    controller1 = XboxController(0)
