from functools import partial

import rclpy
from rclpy.node import Node as BaseNode
from example_interfaces.srv import AddTwoInts


class SumClient(BaseNode):
    def __init__(self):
        self._name = 'sum_two_ints_cli'
        self._timeout = 5.0
        super().__init__(self._name)

        for (a, b) in [(3, 5), (3, 4)]:
            self.call(a, b)

    def call(self, a, b):
        client = self.create_client(AddTwoInts, 'sum_two_ints')
    
        while not client.wait_for_service(self._timeout):
            self.get_logger().warn('Waiting for server sum_two_ints')

        req = AddTwoInts.Request()
        req.a = a
        req.b = b

        future = client.call_async(req)
        future.add_done_callback(partial(self.callback_done, a=a, b=b))
        self.get_logger().info('Waiting for response')
 
    def callback_done(self, fn, a, b):
        try:
            res = fn.result()
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        self.get_logger().info(f'Sum: {res.sum}')


def main(args=None):
    rclpy.init(args=args)

    node = SumClient()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
