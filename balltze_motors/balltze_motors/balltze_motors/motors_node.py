# import rclpy
# from rclpy.node import Node

from dynamixel_ax_12a.motor import Motor
from dynamixel_ax_12a.protocol import DynamixelPort

# from balltze_msgs import MotorStatus, MotorStatusArr, Target, TargetArr, TargetStatus, TargetStatusArr


# class BalltzeMotorsNode(Node):

#     def __init__(self):
#         super().__init__('balltze_motors')

#         self.declare_parameter('serial_port', '/dev/ttyAMA0')
#         self.declare_parameter('max_torque', 1.5)

#         self._serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
#         self._max_torqueport_handler = self.get_parameter('max_torque').get_parameter_value().float_value

#         self._motors_state = None

#         port_handler = DynamixelPort(port_id=self._serial_port, pin=18)
#         motor = Motor(port_handler, 1)
#         # motor.position_speed_torque(1, 2, 3)
#         motor.factory_reset()

#     def loop(self):
#         if self._motors_state:
#             for i, motor in enum(self._motors_state):
#                 self._motors[i].position_speed_torque(
#                     motor.position,
#                     motor.velocity,
#                     motor.torque
#                 )
#             self._motors_state = None
        
#         states = MotorStateArr()
#         motor_state = MotorState()
#         for motor in self._motors:
#             motor_state_arr = motor.
#             motor_state = 
#             states.append()

#     def _motor_callback(self, msg):
#         self._motors_state = msg.motors


# def main(args=None):
#     rclpy.init(args=args)
#     balltze_motors_node = BalltzeMotorsNode()
#     rclpy.spin(balltze_motors_node)


if __name__ == '__main__':
    # port_handler = DynamixelPort(port_id=self._serial_port, pin=18)
    port_handler = DynamixelPort()
    motor = Motor(port_handler, 1)
    # motor.position_speed_torque(1, 2, 3)
    motor.factory_reset()
    # main()