import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Char


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        # ROS2 publisher to send commands to VESC Diff Drive Node
        self.cmd_vel_pub = self.create_publisher(Char, 'cmd_vel', 10)

        # ROS2 subscriber for joystick inputs
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Initialize joystick axes and buttons mapping
        self.linear_axis = 1  # Left stick vertical for forward/backward
        self.angular_axis = 0  # Left stick horizontal for turning
        self.stop_button = 0  # Button 0 to stop

        # Speed scaling factors
        self.linear_scale = 1.0
        self.angular_scale = 1.0

        self.get_logger().info("Joystick Teleop Node Initialized!")

    def joy_callback(self, joy_msg):
        # Access joystick axes
        linear_value = joy_msg.axes[self.linear_axis] * self.linear_scale
        angular_value = joy_msg.axes[self.angular_axis] * self.angular_scale

        # Print axes and button states for debugging
        print(f"Joystick Axes: {joy_msg.axes}")
        print(f"Joystick Buttons: {joy_msg.buttons}")

        # Map joystick input to a keypress
        key = self.map_joystick_to_key(linear_value, angular_value, joy_msg.buttons)

        # Publish key command as Char
        if key:
            msg = Char()
            msg.data = ord(key)  # Convert character to ASCII
            self.cmd_vel_pub.publish(msg)
            print(f"Publishing Key: {key}")  # Print the published key

    def map_joystick_to_key(self, linear, angular, buttons):
        """
        Maps joystick input to a keypress command.
        """
        if buttons[self.stop_button]:  # Stop button pressed
            print("Stop Button Pressed")
            return 'q'  # Stop key

        if linear > 0.5:
            print("Moving Forward")
            return 'e'  # Move forward
        elif linear < -0.5:
            print("Moving Backward")
            return 'c'  # Move backward
        elif angular < 0.5:
            print("Turning Right")
            return 'd'  # Turn right
        elif angular > -0.5:
            print("Turning Left")
            return 'a'  # Turn left

        return None  # No significant input


def main(args=None):
    rclpy.init(args=args)
    joystick_teleop = JoystickTeleop()

    try:
        rclpy.spin(joystick_teleop)
    except KeyboardInterrupt:
        joystick_teleop.get_logger().info("Shutting down Joystick Teleop Node...")
    finally:
        joystick_teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
