import rclpy
from rclpy.node import Node
from burkut_msgs.msg import WaypointArray
from .px4_interface import PX4Interface
from .states.takeoff import Takeoff
from .states.navigate import Navigate
from .states.land import Land
from enum import Enum, auto

class State(Enum):
    IDLE = auto()
    TAKEOFF = auto()
    NAVIGATE = auto()
    LAND = auto()
    Done = auto()

class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')

        self.px4 = PX4Interface(self)

        # State

        self.takeoff = Takeoff(self.px4, target_altitude=20.0)
        self.navigate = Navigate(self.px4)
        self.land = Land(self.px4)

        self.state = State.IDLE
        self.waypoints = []
        
        self.create_subscription(
            WaypointArray,
            'waypoints',
            self.wp_cb,
            10
        )

        self.create_timer(0.1, self._mission)

    def wp_cb(self, msg: WaypointArray):
        self.waypoints = msg.waypoints
        if self.state == State.IDLE and self.waypoints:
            self.get_logger().info('Waypoints alındı KALKIŞ')
            self.state = State.TAKEOFF

    def _mission(self):
        
        
        if self.state == State.IDLE:
            pass                                             #idle durumunda hiçbir şey yapma waypoint bekle

        elif self.state == State.TAKEOFF:
            if self.takeoff.update():
                self.get_logger().info('Takeoff tamamlandı')
                self.navigate.set_waypoints(self.waypoints)
                self.state = State.NAVIGATE
        
        elif self.state == State.NAVIGATE:
            if self.navigate.update():
                self.get_logger().info('Navigasyon tamamlandı')
                self.state = State.LAND

        elif self.state == State.LAND:
            if self.land.update():
                self.get_logger().info('Landing tamamlandı')
                self.state = State.Done
        

def main(args=None):
  rclpy.init(args=args)
  node = FlightController()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()
