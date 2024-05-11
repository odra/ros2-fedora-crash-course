import rclpy
from rclpy.node import Node as BaseNode
from example_interfaces.msg import String

from sample_pkg import node


class SimpleSubNode(BaseNode):
    def __init__(self):
        super(SimpleSubNode, self).__init__('simple_sub')
 
        self.sub = self.create_subscription(String, 'simple_pub', self.handler, 10)
        # self.create_timer(0.5, self.publish)
        self.get_logger().info('Simple subscriber initialized')

    def handler(self, msg):
        self.get_logger().info(f'Received simple_pub data: {msg.data}')


def main(args=None):
    node.run(SimpleSubNode, args)


if __name__ == '__main__':
    main()
