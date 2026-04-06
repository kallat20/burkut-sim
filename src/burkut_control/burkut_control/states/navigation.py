from burkut_control.interface import PX4Interface
import math
class Navigate:
    def __init__(self, px4: PX4Interface):
        self.px4 = px4
        self.waypoints = []
        self.current_index = 0

    def update(self) -> bool:
        """navigation durumunu günceller"""
        if not self.waypoints:
            return True

        self.px4.publish_offboard_heartbeat()

        wp = self.waypoints[self.current_index]
        self.px4.publish_setpoint(wp.x, wp.y, -wp.z, wp.yaw)

        #acceptance radius kontrolü

        pos = self.px4.local_pos
        if pos:
            mesafe = math.sqrt(
                (pos.x - wp.x)**2 +
                (pos.y -wp.y)**2 +
                (pos.z - wp.z)**2
            )
            if mesafe < wp.acceptance_radius:
                self.current_index += 1
                if self.current_index < len(self.waypoints):
                    return True

        return False