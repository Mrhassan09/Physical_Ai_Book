import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    """
    A simple ROS 2 publisher node that publishes a "Hello World" message
    every second to the 'chatter' topic.
    """
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Publisher node started. Publishing to "chatter" topic...')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    try:
        simple_publisher = SimplePublisher()
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        if 'simple_publisher' in locals() and rclpy.ok():
            simple_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
