import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotMovePublisher(Node):

    def __init__(self):
        super().__init__('robot_move_publisher')

        self.publisher_ = self.create_publisher(
            String,
            '/robot_move',
            10
        )

        self.timer = self.create_timer(
            2.0,
            self.publish_move
        )

        self.moves = ["e7e5", "g8f6", "b8c6"]
        self.index = 0

        self.get_logger().info("🤖 Robot move publisher iniciado")

    def publish_move(self):
        msg = String()
        msg.data = self.moves[self.index]
        self.publisher_.publish(msg)

        self.get_logger().info(f"📤 Publicado robot_move: {msg.data}")

        self.index = (self.index + 1) % len(self.moves)


def main():
    rclpy.init()
    node = RobotMovePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
