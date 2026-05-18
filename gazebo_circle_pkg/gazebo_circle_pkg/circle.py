import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped

class GazeboCircle(Node):
    def __init__(self):
        super().__init__('circle')
        qos_profile = QoSProfile(depth=10)
        self.gazebo_circle = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile)
        self.timer = self.create_timer(1, self.turtlebot3_msg)

    def turtlebot3_msg(self):
        msg = TwistStamped()           
        
        msg.twist.linear.x = 2.1
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 1.0

        self.gazebo_circle.publish(msg)
        self.get_logger().info(
            'Published: linear(x={:.2f}, y={:.2f}, z={:.2f}), angular(x={:.2f}, y={:.2f}, z={:.2f})'.format(
                msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z
            )
        )

def main(args=None):
    rclpy.init(args=args)
    node = GazeboCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()