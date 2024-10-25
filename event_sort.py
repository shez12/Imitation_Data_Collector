import tkinter as tk
from tkinter import ttk
import time
import threading
from controller import XboxController, input_translator,execute_command
import rospy
import sys

sys.path.append("/home/hanglok/work/ur_slam")
import ros_utils.myIO
from ik_step import init_robot
from record_ros import info_saver  

class GameUI:
    def __init__(self, master):
        self.master = master
        master.title("Game UI")
        master.geometry("800x600")

        # Configure row and column to expand
        master.grid_rowconfigure(0, weight=1)
        master.grid_columnconfigure(0, weight=1)

        self.frame = ttk.Frame(master)
        self.frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Configure frame's row and column to expand
        self.frame.grid_rowconfigure(0, weight=1)
        self.frame.grid_columnconfigure(0, weight=1)

        self.start_button = ttk.Button(self.frame, text="Start Game", command=self.start_game)
        self.start_button.grid(row=0, column=0)

        self.last_signal_time = 0
        self.is_monitoring = False
        self.controller = None
        self.robot = None

        self.action_num = 1
        self.info_saver = None
        self.is_recording = False

    def start_game(self):
        # Remove the start button
        self.start_button.grid_forget()

        # Create and display the new stage elements
        self.job_label = ttk.Label(self.frame, text="Enter job name:", font=("Arial", 14))
        self.job_label.grid(row=0, column=0, pady=(0, 10))

        self.job_entry = ttk.Entry(self.frame, width=30, font=("Arial", 12))
        self.job_entry.grid(row=1, column=0, pady=(0, 10))

        self.robot_label = ttk.Label(self.frame, text="Enter robot name:", font=("Arial", 14))
        self.robot_label.grid(row=2, column=0, pady=(10, 10))

        self.robot_entry = ttk.Entry(self.frame, width=30, font=("Arial", 12))
        self.robot_entry.insert(0, "robot2")  # Set default value
        self.robot_entry.grid(row=3, column=0, pady=(0, 20))

        self.eposide_label = ttk.Label(self.frame, text="Enter eposide index:", font=("Arial", 14))
        self.eposide_label.grid(row=4, column=0, pady=(10, 10))


        self.eposide_idx = ttk.Entry(self.frame, width=30, font=("Arial", 12))
        self.eposide_idx.insert(0, "0")  # Set default value
        self.eposide_idx.grid(row=5, column=0, pady=(0, 20))


        self.submit_button = ttk.Button(self.frame, text="Submit", command=self.submit_job_and_robot)
        self.submit_button.grid(row=6, column=0)

        # Center the new elements
        self.frame.grid_rowconfigure(0, weight=1)
        self.frame.grid_rowconfigure(6, weight=1)

    def submit_job_and_robot(self):
        self.job = self.job_entry.get()
        self.robot_name = self.robot_entry.get()
        
        print(f"Submitted job: {self.job}")
        print(f"Robot name: {self.robot_name}")
        self.start_stage3()

    def start_stage3(self):
        # Clear previous widgets
        for widget in self.frame.winfo_children():
            widget.grid_forget()

        self.signal_label = ttk.Label(self.frame, text="Controller Signal: None", font=("Arial", 14))
        self.signal_label.grid(row=0, column=0, pady=20)

        self.announcement_label = ttk.Label(self.frame, text="", font=("Arial", 12))
        self.announcement_label.grid(row=1, column=0, pady=20)

        # Add Stop button
        self.stop_button = ttk.Button(self.frame, text="Stop", command=self.stop_actions)
        self.stop_button.grid(row=2, column=0, pady=20)

        # Add save button
        self.save_button = ttk.Button(self.frame, text="Save", command=self.stop_recording)
        self.save_button.grid(row=3, column=0, pady=20)

        self.is_monitoring = True
        self.controller = XboxController()
        self.gripper = ros_utils.myGripper.MyGripper()
        self.robot = init_robot(self.robot_name)
        self.monitor_thread = threading.Thread(target=self.monitor_signals, daemon=True)
        self.monitor_thread.start()

    def monitor_signals(self):
        self.time_str = time.strftime("%Y%m%d_%H%M%S")
        while self.is_monitoring:
            self.controller.capture_input()
            if self.controller.inputs:
                self.last_signal_time = time.time()
                self.master.after(0, self.update_signal_label, str(self.controller.inputs))
                if not self.is_recording:
                    self.start_recording()
                    self.is_recording = True

            # else:
            #     current_time = time.time()
            #     if self.last_signal_time > 0 and current_time - self.last_signal_time > 2:#todo
            #         self.master.after(0, self.show_announcement)
            #         self.last_signal_time = 0
            #         if self.is_recording:
            #             self.stop_recording()

            tcp_move, tcp_rotate, io_state,gripper_value = input_translator(self.controller.inputs)
            if self.controller.inputs.get("b_7") == 1:
                print("Exiting...")
                break
            if io_state != -1:
                ros_utils.myIO.set_IO(io_state)
            if sum(tcp_move) + sum(tcp_rotate) != 0:
                self.robot.step(action=execute_command(tcp_move, tcp_rotate), wait=False)
            if gripper_value >= -1:
                self.gripper.set_gripper(500*(1-gripper_value), 20.0) 
            # time.sleep(0.1)

    def update_signal_label(self, signal_text):
        self.signal_label.config(text=f"Controller Signal: {signal_text}")
        self.announcement_label.config(text=f"Action {self.action_num}")

    def show_announcement(self):
        self.announcement_label.config(text="Movement break")
        self.signal_label.config(text="Controller Signal: None")

    def start_recording(self):
        print("Started recording")
        self.info_saver = info_saver(10,self.robot_name)  
        self.info_saver.listener()

    def stop_recording(self):
        self.show_announcement()
        self.info_saver.stop_subscriptions()
        self.is_recording = False
        file_path = f'data/{self.job}'
        self.info_saver.save_all_info(file_path,self.eposide_idx.get())
        print(f"Stopped recording and saved data")
        self.action_num += 1
        

    def stop_actions(self):
        self.is_monitoring = False
        if self.is_recording:
            self.stop_recording()
        if self.monitor_thread:
            self.monitor_thread.join()
        self.signal_label.config(text="Controller Signal: Stopped")
        self.announcement_label.config(text="All actions stopped")
        self.stop_button.config(state="disabled")
        
        # Transition to stage 4
        self.master.after(2000, self.start_stage4)

    def start_stage4(self):
        # Clear the frame
        for widget in self.frame.winfo_children():
            widget.destroy()

        # Set up stage 4 UI
        stage4_label = ttk.Label(self.frame, text="completed", font=("Arial", 16, "bold"))
        stage4_label.grid(row=0, column=0, pady=20)

        summary_label = ttk.Label(self.frame, text="all data saved.", font=("Arial", 12))
        summary_label.grid(row=1, column=0, pady=20)

        # Add any additional UI elements or functionality for stage 4

        close_button = ttk.Button(self.frame, text="Close", command=self.master.destroy)
        close_button.grid(row=2, column=0, pady=20)

def main():
    rospy.init_node('ik_step', anonymous=True)
    root = tk.Tk()
    game_ui = GameUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
