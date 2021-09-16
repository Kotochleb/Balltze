import rclpy
from rclpy.node import Node

from dynamixel_ax_12a import DynamixelPort, Motor

from balltze_msgs import MotorStatus, MotorStatusArr, Target, TargetArr, TargetStatus, TargetStatusArr


class BalltzeMotorsNode(Node):

    def __init__(self):
        super().__init__('balltze_motors')

        self.declare_parameter('serial_port_0', '/dev/ttyTHS1')
        self.declare_parameter('serial_port_1', '/dev/ttyTHS2')
        self.declare_parameter('max_torque', 1.5)

        self._serial_port_0 = self.get_parameter('serial_port_0').get_parameter_value().string_value
        self._serial_port_1 = self.get_parameter('serial_port_1').get_parameter_value().string_value
        self._max_torqueport_handler = self.get_parameter('max_torque').get_parameter_value().float_value

        self._motors_command = None
        self._motors_state = []*12

        port_handler = DynamixelPort(port_id=self._serial_port, pin=18)
        motors = [Motor(port_handler, 0x01), Motor(port_handler, 0x02), Motor(port_handler, 0x03),
                  Motor(port_handler, 0x04), Motor(port_handler, 0x05), Motor(port_handler, 0x06),
                  Motor(port_handler, 0xFE)]


    def loop(self):
        if self._motors_command:
            for i, motor_command in enum(self._motors_command):
                self._motors[i].position_speed_torque(
                    motor_command.position,
                    motor_command.velocity,
                    motor_command.torque
                )
                self._motors_state[i] = self._motors[i].current_position_speed_torque
            self._motors[-1].action()


    def _motor_callback(self, msg):
        self._motors_command = msg.motors


def main(args=None):
    rclpy.init(args=args)
    balltze_motors_node = BalltzeMotorsNode()
    rclpy.spin(balltze_motors_node)


if __name__ == '__main__':
    main()