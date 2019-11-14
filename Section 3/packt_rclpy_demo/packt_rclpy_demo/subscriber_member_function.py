import rclpy

from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
  
  def __init__(self):
    super().__init__('listener')
    self.subscription = self.create_subscription(
      String,
      'chatter',
      self.listener_callback
    )
    self.subscription
  
  def listener_callback(self, msg):
    self.get_logger().info("I heard: {0}".format(msg.data))

def main(args=None):
  rclpy.init(args=args)

  try:
    listener = Listener()
    rclpy.spin(listener)
  finally:
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main':
  main()