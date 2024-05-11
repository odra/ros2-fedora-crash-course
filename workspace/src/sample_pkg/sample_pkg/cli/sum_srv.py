import rclpy
from rclpy.node import Node as BaseNode
from example_interfaces.srv import AddTwoInts

from sample_pkg import node


class SumNode(BaseNode):
    def __init__(self):
        self._name = 'sum_node'
        super(SumNode, self).__init__(self._name)
        self.get_logger().info('Sum node initialized')
        self._s = self.create_service(AddTwoInts, 'sum_two_ints', self.callback)

    def callback(self, req, res):
        self.get_logger().info(f'Sum: {req.a}, {req.b}')
        res.sum = req.a + req.b 

        return res


def main(args=None):
    node.run(SumNode, args)


if __name__ == '__main__':
    main()
