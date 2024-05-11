import rclpy
from rclpy.node import Node as BaseNode

from sample_pkg import node


class SimpleNode(BaseNode):
    def __init__(self):
        self._name = 'simple_node'
        self._counter = 0
        super(SimpleNode, self).__init__(self._name)
        self.get_logger().info('Simple node initialized')
        self.create_timer(0.5, self.callback)

    def callback(self):
        self._counter += 1
        self.get_logger().info(f'[{self._name}] Counter: {self._counter}')


def main(args=None):
    node.run(SimpleNode, args)


if __name__ == '__main__':
    main()
