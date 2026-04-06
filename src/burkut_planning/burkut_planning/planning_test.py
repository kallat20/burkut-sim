from burkut_msgs.msg._waypoint import Waypoint
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from burkut_msgs.msg import ObstacleArray
from burkut_msgs.msg import WaypointArray

class PlanningTest(Node):
    def __init__(self):
        super().__init__('planning_test')
        self.subscriber_ = self.create_subscription(
            ObstacleArray, 
            'pole_test', 
            self.listener_callback, #type: ignore
            10
            )
        self.publisher_ = self.create_publisher(WaypointArray, 'waypoints', 10)
        timer = 1.0
        self.timer = self.create_timer(timer, self.waypoint_callback)
        self.points = []


    def listener_callback(self, msg: ObstacleArray):
        self.get_logger().info(f'Engeller alındı: {len(msg.obstacles)}')

        points = []
        used =set()
        for i, p1 in enumerate(msg.obstacles):
            for j, p2 in enumerate(msg.obstacles):
                if i >=j or j in used:
                    continue

                if abs(p1.x - p2.x) < 1.0 and abs(p1.y - p2.y) < 6.0:
                    if p1.radius == p2.radius:
                        center_x = (p1.x + p2.x) / 2
                        center_y = (p1.y + p2.y) / 2
                        fark = 0.0
                    else:
                        fark = p1.radius - p2.radius
                        center_x = ((p1.x + p2.x) / 2) + fark
                        center_y = ((p1.y + p2.y) / 2) + fark
                    avg_height = (p1.height + p2.height) / 2
                    confidence = (p1.confidence + p2.confidence) / 2
                    type = 'pole'
                    points.append((center_x, center_y, avg_height, confidence, type,fark))
                    used.update([i, j])

        self.points = sorted(points, key=lambda x: x[0])

    def waypoint_callback(self):
        i = 0
        msg = WaypointArray()

        for (x, y, avg_height, confidence, type, fark) in self.points:
            wp = Waypoint()
            wp.x = x
            wp.y = y
            if avg_height / 2 < 2.0:    #ileride sıkıntı çıkarabilir zemin 0 alma engel yüksekliği doğruluğuna bak
                wp.z = 2.0
            else:
                wp.z = avg_height
            wp.yaw = 0.0                                    
            wp.acceptance_radius = 2.0
            wp.fark = fark #type:ignore

            msg.waypoints.append(wp)        #type:ignore
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PlanningTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('waypoint yayinlandi')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    


        



        


