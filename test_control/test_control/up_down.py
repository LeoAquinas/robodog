import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import customtkinter as ctk
import math
import time
import threading
import numpy as np

# Define unique offsets for each leg
hip_offsets = [0.0, 0.0, 0.0, 0.0]  # Example offsets for each hip joint
uleg_offsets = [0.0, 0.0, 0.0, 0.0]  # Upper leg offsets
lleg_offsets = [0.0, 0.0, 0.0, 0.0]  # Lower leg offsets

class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'gazebo_joint_controller/commands', 10)

        self.joint_names = [
            'FR_hip_joint', 'FR_uleg_joint', 'FR_lleg_joint',
            'FL_hip_joint', 'FL_uleg_joint', 'FL_lleg_joint',
            'BR_hip_joint', 'BR_uleg_joint', 'BR_lleg_joint',
            'BL_hip_joint', 'BL_uleg_joint', 'BL_lleg_joint'
        ]
        
        # Initialize joint positions and gait parameters
        self.joint_positions = [0.0] * 12
        self.neutral_positions = [0.0] * 12
        self.step_phase = 0
        self.walking = False
        self.walk_thread = None
        self.current_stride = 0.15  # meters
        self.body_lean = math.radians(5)  # Initial forward lean
        self.period = 1.0  # Gait cycle period

        # Create GUI
        self.create_gui()

    # GUI TO CONTROL ROBOT
    def create_gui(self):
        self.root = ctk.CTk()
        self.root.title("Quadruped Control Panel")

        # Body Height Control (now in meters: 0.25 - 0.40 m)
        z_label = ctk.CTkLabel(self.root, text="Body Height (250-400 mm):")
        z_label.grid(row=0, column=0, padx=10, pady=(10,0), columnspan=2)
        
        self.z_slider = ctk.CTkSlider(self.root, from_=0.25, to=0.40, orientation='horizontal', command=self.update_z_display)
        self.z_slider.grid(row=1, column=0, padx=10, pady=(0,10), columnspan=2)
        self.z_slider.set(0.315)
        self.z_slider.bind("<Motion>", self.update_z)
        
        # Height value display (showing in mm)
        self.z_value_label = ctk.CTkLabel(self.root, text="355 mm")
        self.z_value_label.grid(row=1, column=2, padx=10)

        # Stride Length Control (now in meters: 0.05 - 0.30 m)
        stride_label = ctk.CTkLabel(self.root, text="Stride Length (50-300 mm):")
        stride_label.grid(row=2, column=0, padx=10, pady=(10,0), columnspan=2)
        
        self.stride_slider = ctk.CTkSlider(self.root, from_=0.05, to=0.30, orientation='horizontal', command=self.update_stride_display)
        self.stride_slider.grid(row=3, column=0, padx=10, pady=(0,10), columnspan=2)
        self.stride_slider.set(0.10)
        self.stride_slider.bind("<Motion>", self.update_stride)
        
        # Stride value display (showing in mm)
        self.stride_value_label = ctk.CTkLabel(self.root, text="150 mm")
        self.stride_value_label.grid(row=3, column=2, padx=10)

        # Control Buttons
        self.walk_button = ctk.CTkButton(self.root, text="Start Walking", command=self.toggle_walking)
        self.walk_button.grid(row=4, column=0, pady=10, padx=5)

        stop_button = ctk.CTkButton(self.root, text="Emergency Stop", command=self.emergency_stop)
        stop_button.grid(row=4, column=1, pady=10, padx=5)

        # Height-only update button
        height_button = ctk.CTkButton(self.root, text="Update Height", command=self.update_height)
        height_button.grid(row=5, column=0, columnspan=2, pady=10)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    # ------- EXTRA SECTION TO UPDATE SLIDER LABEL DISPLAYS ------- #
    def update_z_display(self, value):
        """Update height display in real-time (display in mm)"""
        self.z_value_label.configure(text=f"{int(float(value)*1000)} mm")
        self.update_z(value)

    def update_stride_display(self, value):
        """Update stride display in real-time (display in mm)"""
        self.stride_value_label.configure(text=f"{int(float(value)*1000)} mm")
        self.update_stride(value)

    # ------- SECTION TO UPDATE HEIGHT DISPLACEMENT ------- #
    def update_z(self, event):
        """Update height display and limits"""
        new_z = self.z_slider.get()  # Already in meters
        self.z_value_label.configure(text=f"{int(new_z*1000)} mm")
        
        # Update stride limits
        max_stride = self.calculate_max_stride(new_z)
        if self.current_stride > max_stride:
            self.current_stride = max_stride
            self.stride_slider.set(max_stride)

    def update_stride(self, event):
        """Update stride display"""
        new_stride = self.stride_slider.get()  # Already in meters
        self.stride_value_label.configure(text=f"{int(new_stride*1000)} mm")

    def update_height(self):
        """Stand at current height without walking"""
        z_val = self.z_slider.get()  # In meters
        self.joint_positions = self.neutral_positions.copy()
        
        # Calculate neutral position angles
        B, C, _ = self.calculate_ik(z_val, 0, 0)  # x=0 for neutral position
        
        # FR (front-right) and BL (back-left)
        self.joint_positions[1] = -B + 0.857 + uleg_offsets[0]
        self.joint_positions[2] = C - 1.569 + lleg_offsets[0]
        self.joint_positions[10] = -B + 0.857 + uleg_offsets[3]
        self.joint_positions[11] = C - 1.569 + lleg_offsets[3]
        
        # FL (front-left) and BR (back-right)
        self.joint_positions[4] = B - 0.857 + uleg_offsets[1]
        self.joint_positions[5] = -C + 1.569 + lleg_offsets[1]
        self.joint_positions[7] = B - 0.857 + uleg_offsets[2]
        self.joint_positions[8] = -C + 1.569 + lleg_offsets[2]

        self.get_logger().info(
            f"Height Update Positions:\n"
            f"FR: {self.joint_positions[0]:.2f}, {self.joint_positions[1]:.2f}, {self.joint_positions[2]:.2f}\n"
            f"FL: {self.joint_positions[3]:.2f}, {self.joint_positions[4]:.2f}, {self.joint_positions[5]:.2f}\n"
            f"BR: {self.joint_positions[6]:.2f}, {self.joint_positions[7]:.2f}, {self.joint_positions[8]:.2f}\n"
            f"BL: {self.joint_positions[9]:.2f}, {self.joint_positions[10]:.2f}, {self.joint_positions[11]:.2f}"
        )

        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher_.publish(msg)
        self.get_logger().info("Updated standing height")

    # ------- SECTION TO UPDATE MOVEMENT COMMAND ------- #
    def toggle_walking(self):
        if not self.walking:
            self.walking = True
            self.walk_button.configure(text="Stop Walking")
            self.walk_thread = threading.Thread(target=self.walk_loop)
            self.walk_thread.start()
        else:
            self.walking = False
            self.walk_button.configure(text="Start Walking")

    def walk_loop(self):
        while self.walking:
            start_time = time.time()
            cycle_duration = self.period
            z_val = self.z_slider.get()  # In meters
            
            # Calculate safe stride limits
            max_stride = self.calculate_max_stride(z_val)
            safe_stride = min(self.current_stride, max_stride)
            
            # Gait parameters
            R = safe_stride / 2
            duty_factor = 0.7  # Stance phase percentage
            
            elapsed = time.time() - start_time
            phase_progress = (elapsed % cycle_duration) / cycle_duration
            
            # Asymmetric gait pattern
            if phase_progress < duty_factor:
                # Stance phase - backward push
                x = R * (1 - 2 * phase_progress/duty_factor)
            else:
                # Swing phase - forward return
                x = R * (2 * (phase_progress - duty_factor)/(1 - duty_factor) - 1)
            
            self.update_step(z_val, R, x)
            time.sleep(0.8)

    # ------- SECTION TO SEND MOVEMENT COMMAND TO MOTOR ------- #
    def update_step(self, z_val, R, x):
        # Body lean compensation
        lean_comp = self.body_lean * 0.4 * R
        
        # Front legs calculation
        B_front, C_front, theta_front = self.calculate_ik(
            z_val + lean_comp, 
            x - lean_comp, 
            R
        )
        
        # Hind legs calculation
        B_hind, C_hind, theta_hind = self.calculate_ik(
            z_val - lean_comp,
            x + lean_comp,
            R
        )

        self.joint_positions = self.neutral_positions.copy()
        
        # if self.step_phase == 0:
        #     # FR (front-right) and BL (back-left)
        #     self.joint_positions[1] = -B_front + 0.857 - theta_front + uleg_offsets[0]
        #     self.joint_positions[2] = C_front - 1.569 - theta_front + lleg_offsets[0]  # Changed sign
        #     self.joint_positions[10] = -B_hind + 0.857 + theta_hind + uleg_offsets[3]
        #     self.joint_positions[11] = -C_hind - 1.569 - theta_hind + lleg_offsets[3]  # Changed sign
        # else:
        #     # FL (front-left) and BR (back-right)
        #     self.joint_positions[4] = B_front - 0.857 + theta_front + uleg_offsets[1]
        #     self.joint_positions[5] = -C_front + 1.569 - theta_front + lleg_offsets[1]  # Changed sign
        #     self.joint_positions[7] =-B_hind - 0.857 - theta_hind + uleg_offsets[2]
        #     self.joint_positions[8] = -C_hind + 1.569 + theta_hind + lleg_offsets[2]  # Changed sign
        if self.step_phase == 0:
            # FR (front-right) and BL (back-left) in stance
            self.joint_positions[1] = -(-B_front + 1.169 + theta_front + uleg_offsets[0])  # FR upper
            self.joint_positions[2] = -(C_front - 0.857 + theta_front + lleg_offsets[0])   # FR lower
            self.joint_positions[10] = -(-B_hind + 1.169 + theta_hind + uleg_offsets[3])    # BL upper
            self.joint_positions[11] = -(C_hind - 0.857 + theta_hind + lleg_offsets[3])     # BL lower
        else:
            # FL (front-left) and BR (back-right) in stance
            self.joint_positions[4] = B_front - 0.857 + theta_front + uleg_offsets[1]   # FL upper
            self.joint_positions[5] = -C_front + 1.569 + theta_front + lleg_offsets[1]   # FL lower
            self.joint_positions[7] = B_hind - 0.857 + theta_hind + uleg_offsets[2]      # BR upper
            self.joint_positions[8] = -C_hind + 1.569 + theta_hind + lleg_offsets[2]     # BR lower


        # # Apply body lean to all hips
        # hip_lean = self.body_lean * (1 + 0.2 * math.sin(time.time() * 2))
        # for i in [0, 3, 6, 9]:
        #     self.joint_positions[i] = hip_lean

        self.get_logger().info(
            f"Joint Positions:\n"
            f"FR: {self.joint_positions[0]:.2f}, {self.joint_positions[1]:.2f}, {self.joint_positions[2]:.2f}\n"
            f"FL: {self.joint_positions[3]:.2f}, {self.joint_positions[4]:.2f}, {self.joint_positions[5]:.2f}\n"
            f"BR: {self.joint_positions[6]:.2f}, {self.joint_positions[7]:.2f}, {self.joint_positions[8]:.2f}\n"
            f"BL: {self.joint_positions[9]:.2f}, {self.joint_positions[10]:.2f}, {self.joint_positions[11]:.2f}"
        )

        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher_.publish(msg)
        self.step_phase = 1 - self.step_phase

    # ------- SECTION TO CALCULATE IK ------- #
    def calculate_ik(self, c_old, x, R):
        # Elliptical trajectory with safety margins
        z = math.sqrt(R**2 - (0.85*x)**2) * 0.95  # Flattened ellipse with margin
        theta = math.atan(x/c_old)
        c = max((c_old - z)/math.cos(theta), 0.15)  # Minimum leg extension

        # Link lengths (meters)
        a = 0.200
        b = 0.250

        cos_B = ((a**2 + c**2 - b**2) / (2*a*c))
        cos_C = ((a**2 + b**2 - c**2) / (2*a*b))
        cos_B = np.clip(cos_B, -1, 1)
        cos_C = np.clip(cos_C, -1, 1)

        return math.acos(cos_B), math.acos(cos_C), theta

    def calculate_max_stride(self, z_height):
        """Calculate maximum safe stride based on current height (meters)"""
        a = 0.200
        b = 0.2306
        max_reach = math.sqrt((a + b)**2 - z_height**2)
        return max_reach * 0.8  # 80% of theoretical max

    def update_stride(self, event):
        """Handle stride changes with smooth transition"""
        target_stride = self.stride_slider.get()  # In meters
        z_val = self.z_slider.get()  # In meters
        max_stride = self.calculate_max_stride(z_val)
        
        if target_stride > max_stride:
            self.stride_slider.set(max_stride)
            target_stride = max_stride
        
        # Smooth interpolation
        self.current_stride += (target_stride - self.current_stride) * 0.1
        
        # Adjust gait period based on stride
        self.period = 0.8 + (self.current_stride * 1.5)

    def emergency_stop(self):
        self.walking = False
        self.joint_positions = self.neutral_positions.copy()
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher_.publish(msg)

    def on_closing(self):
        self.walking = False
        if self.walk_thread is not None:
            self.walk_thread.join()
        self.root.quit()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    joint_controller_node = JointControllerNode()
    rclpy.spin(joint_controller_node)

if __name__ == '__main__':
    main()
