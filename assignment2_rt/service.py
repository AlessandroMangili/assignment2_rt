import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool

class StopRestartService(Node):
    def __init__(self):
        super().__init__('control_robot_service')
        self.publisher_ = self.create_publisher(Bool, 'control_robot', 10)  # publish True if the user want to stop the robot, otherwise false when he want to restart it
        self.publisher_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.srv = self.create_service(SetBool, 'stop_restart_robot', self.set_velocity_callback)
        self.get_logger().info('Robot control service is ready')

    def set_velocity_callback(self, request, response):
        stop = request.data   # If the request.data is true, then stop the robot, otherwise restart it
        if stop:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_cmd.publish(twist)
        
        msg = Bool()
        msg.data = request.data
        self.publisher_.publish(msg)
        

        response.success = True
        if stop:
            response.message = "Robot stopped"
        else:
            response.message = "Robot restarted"
        return response

def main(args=None):
    rclpy.init(args=args)
    control_robot = StopRestartService()
    
    try:
        rclpy.spin(control_robot)
    except KeyboardInterrupt:
        # Handle the interrupt (Ctrl+C)
        print("Program interrupted")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        control_robot.destroy_node()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
