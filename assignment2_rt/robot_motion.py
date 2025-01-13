import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
import time

LINEAR_THR = 5.0
ANGULAR_THR = 5.0

class Robot(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        # Publish the new velocity on the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_robot_position = self.create_publisher(String, 'robot_position', 10)
        self.sub = self.subscriptions(Bool, 'control_robot', self.control_callback, 10)
        self.velocity = Twist()
        self.stop = False

    def move_robot(self, x, y, z):
        self.velocity.linear.x = x
        self.velocity.linear.y = y
        self.velocity.angular.z = z
        self.publisher_.publish(self.velocity)
        
        x_feet = x * 3.28
        y_feet = y * 3.28
        result = f'{x_feet} - {y_feet}'
        msg = String()
        msg.data = result
        self.publisher_robot_position.publish(msg)
        
        self.get_logger().info(f'Moving robot: {self.velocity.linear.x} forward and {self.velocity.angular.z} angular')
        
    def control_callback(self, msg):
        self.stop = msg.data
        if msg.data:
            self.get_logger().info('Robot stopped')
        else:
            self.get_logger().info('Robot restart')
        

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()

    try:
        while rclpy.ok():
            if robot.stop:
                continue
            # Ask the user to insert the values for linear velocity and angular
            try:
                x = float(input("Enter the linear velocity x: "))
                x = min(x, LINEAR_THR)
                x = max(x, -LINEAR_THR)
                #y = float(input("Enter the linear velocity y: ")) # Not supported here
                z = float(input("Enter angular velocity z: "))
                z = min(z, ANGULAR_THR)
                z = max(z, -ANGULAR_THR)
            except ValueError:
                print("Invalid input. Please enter numeric values.")
                continue 

            robot.move_robot(x, 0.0, z)
            # Waits 3 seconds before stopping the robot and asks the user for the new velocity again
            time.sleep(3)
            # Stop the robot by publishing a zero velocity
            robot.move_robot(0.0, 0.0, 0.0)
    except KeyboardInterrupt:
        # Handle the interrupt (Ctrl+C)
        print("Program interrupted")
    except Exception as e:
        # Handle other exception
        print(f"An error occurred: {e}")
    finally:
        robot.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()