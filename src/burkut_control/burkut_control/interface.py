from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)

class PX4Interface:
    def __init__(self, node):
        self.node = node
        
        # Publisher'lar
        self.offboard_mode_pub = node.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = node.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_cmd_pub = node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscriber'lar
        self.local_pos = None
        self.vehicle_status = None
        node.create_subscription(VehicleLocalPosition,
            '/fmu/out/vehicle_local_position', self._pos_cb, 10)
        node.create_subscription(VehicleStatus,
            '/fmu/out/vehicle_status', self._status_cb, 10)

    def _pos_cb(self, msg): self.local_pos = msg
    def _status_cb(self, msg): self.vehicle_status = msg

    def publish_offboard_heartbeat(self, position=True):
        # PX4 offboard modda kalmak için her 100ms'de yayınlanmalı
        msg = OffboardControlMode()
        msg.position = position
        msg.timestamp = self._timestamp()
        self.offboard_mode_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw=float('nan')):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self._timestamp()
        self.setpoint_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp()
        self.vehicle_cmd_pub.publish(msg)

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard_mode(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def _timestamp(self):
        return self.node.get_clock().now().nanoseconds // 1000

        