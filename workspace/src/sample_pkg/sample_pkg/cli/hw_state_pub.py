import random

import rclpy
from rclpy.node import Node as BaseNode

from sample_pkg import node
from sample_interfaces.msg import HWState


class HWStatePubNode(BaseNode):
    def __init__(self):
        super().__init__('hw_state_pub')
 
        self.pub = self.create_publisher(HWState, 'hw_state_pub', 10)
        self.create_timer(0.5, self.publish)
        self.get_logger().info('HWState publisher initialized')

    def publish(self):
        msg = HWState()
        msg.temperature = random.randrange(0, 40)
        msg.ready = False if msg.temperature < 10 else True
        msg.comment = f'{msg.temperature}C, Readiness: {msg.ready}'

        self.pub.publish(msg)


def main(args=None):
    node.run(HWStatePubNode, args)

if __name__ == '__main__':
    main()
