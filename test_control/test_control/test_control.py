import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import customtkinter as ctk

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


        self.joint_positions = [0.0] * len(self.joint_names)

        # Start the GUI
        self.create_gui()

    def create_gui(self):
        self.root = ctk.CTk()
        self.root.title("Gazebo Joint Controller")

        # Create input fields for each joint
        self.entries = {}
        for i, joint in enumerate(self.joint_names):
            label = ctk.CTkLabel(self.root, text=joint)
            label.grid(row=i, column=0, padx=10, pady=5)

            entry = ctk.CTkEntry(self.root)
            entry.grid(row=i, column=1, padx=10, pady=5)
            entry.insert(0, str(self.joint_positions[i]))
            self.entries[joint] = entry

        # Create a button to publish the joint positions
        publish_button = ctk.CTkButton(self.root, text="Publish", command=self.publish_joint_positions)
        publish_button.grid(row=len(self.joint_names), columnspan=2, pady=10)

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def publish_joint_positions(self):
        # Update joint positions from the GUI entries
        for i, joint in enumerate(self.joint_names):
            try:
                self.joint_positions[i] = float(self.entries[joint].get())
            except ValueError:
                self.get_logger().warn(f"Invalid input for {joint}, using previous value.")

        # Create a message and publish it
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

    def on_closing(self):
        self.root.quit()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    joint_controller_node = JointControllerNode()
    rclpy.spin(joint_controller_node)

if __name__ == '__main__':
    main()