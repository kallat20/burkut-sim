import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from burkut_msgs.msg import Obstacle
from burkut_msgs.msg import ObstacleArray

#direklerin gazebodaki koordinatları
POLE = [
    (-50.0, -15.0, "pole_11"),
    (-50.0, -18.0, "pole_12"),
    (  0.0, -18.0, "pole_21"),
    (  0.0, -21.0, "pole_22"),
    ( 50.0, -15.0, "pole_31"),
    ( 50.0, -18.0, "pole_32"),
]

class PoleTestPublisher(Node):
    def __init__(self):
        super().__init__('pole_test_publisher')
        self.publisher_ = self.create_publisher(ObstacleArray, 'pole_test', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) #1hz ilk aşama için yeterli

    def timer_callback(self):
        msg = ObstacleArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for (x, y, label) in POLE:
            obs = Obstacle()
            obs.x = x
            obs.y = y
            obs.z = 0.0
            obs.radius = 0.1
            obs.height = 3.0
            obs.confidence = 1.0
            obs.type = 'pole'
            msg.obstacles.append(obs)  # type: ignore

        self.publisher_.publish(msg)
        self.get_logger().info(f'{len(msg.obstacles)} engeller yayınlandı')

def main(args=None):
    rclpy.init(args=args)
    node = PoleTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Engeller yayınlandı')
    finally:
        node.destroy_node()
        rclpy.shutdown()
