import rclpy
from rclpy.node import Node as BaseNode
from example_interfaces.msg import String

from sample_pkg import node


class SimplePubNode(BaseNode):
    def __init__(self):
        super(SimplePubNode, self).__init__('simple_pub')
        
        # params
        self.declare_parameter('nickname','unamed')

        self.pub = self.create_publisher(String, 'simple_pub', 10)
        self.create_timer(0.5, self.publish)
        self.get_logger().info('Simple publisher initialized')

    def publish(self):
        nickname = self.get_parameter('nickname').value
        msg = String()
        msg.data = f'hello from simple_pub ({nickname})'

        self.pub.publish(msg)


def main(args=None):
    node.run(SimplePubNode, args)

if __name__ == '__main__':
    main()
