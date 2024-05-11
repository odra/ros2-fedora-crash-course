import rclpy


def run(node, args=None, **kwargs):
    # start node
    rclpy.init(args=args)

    if kwargs.get('spin', True):
        rclpy.spin(node())

    rclpy.shutdown()
