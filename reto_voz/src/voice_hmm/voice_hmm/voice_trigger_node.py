import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class VoiceTriggerNode(Node):
    def __init__(self):
        super().__init__("voice_trigger_node")

        self.publisher = self.create_publisher(
            Bool,
            "/voice/listen_flag",
            10,
        )

        self.get_logger().info("Voice trigger ready. Press ENTER to request one recording.")

        self.input_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True,
        )
        self.input_thread.start()

    def keyboard_loop(self):
        while rclpy.ok():
            try:
                input()
            except EOFError:
                return

            msg = Bool()
            msg.data = True
            self.publisher.publish(msg)
            self.get_logger().info("Published /voice/listen_flag = true")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceTriggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
