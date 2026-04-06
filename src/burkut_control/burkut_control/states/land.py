from burkut_control.interface import PX4Interface
from px4_msgs.msg import VehicleCommand
class Land:
    def __init__(self, px4: PX4Interface):
        self.px4 = px4
        self.land_commanded = False

    def update(self) -> bool:
        """land durumunu günceller"""
        self.px4.publish_offboard_heartbeat()

        if not self.land_commanded:
            self.px4.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_LAND_START)
            self.land_commanded = True
        
        status = self.px4.vehicle_status
        if status and status.arming_state == 1:
            return True
        return False
        