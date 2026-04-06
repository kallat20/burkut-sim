from burkut_control.interface import PX4Interface
class Takeoff:
    def __init__(self, px4:PX4Interface, target_altitude: float):
        self.px4 = px4
        self.target_altitude = target_altitude
        self.counter = 0
    
    def update(self) -> bool:
        """takeoff durumunu günceller"""
        self.px4.publish_offboard_heartbeat()

        if self.counter < 10:
            self.px4.publish_setpoint(0.0, 0.0, -self.target_altitude)
            self.counter += 1
            return False
        
        if self.counter == 10:
            self.px4.arm()
            self.counter += 1
            
        self.px4.publish_setpoint(0.0, 0.0, -self.target_altitude)

        pos = self.px4.local_pos

        if pos and abs(pos.z - self.target_altitude) < 0.4:
            return True
        return False