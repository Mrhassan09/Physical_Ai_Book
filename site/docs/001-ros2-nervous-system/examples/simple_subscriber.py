import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    """
    A simple ROS 2 subscriber node that listens to the 'chatter' topic
    and logs the messages it receives.
    """
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscriber node started. Listening to "chatter" topic...')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    try:
        simple_subscriber = SimpleSubscriber()
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        if 'simple_subscriber' in locals() and rclpy.ok():
            simple_subscriber.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
