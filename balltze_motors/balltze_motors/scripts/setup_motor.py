#!/usr/bin/python3

import time
import argparse

from dynamixel_ax_12a import DynamixelPort, Motor

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument('-id', dest='motor_id', type=int, help="Motor ID to be set", required=True, default=0x01)
parser.add_argument('-p', dest='pin', type=int, help="Direction pin", required=False, default=18)
parser.add_argument('-s', '--serial', dest='serial', type=str, help="Serial port string", required=False, default='/dev/ttyAMA0')
parser.add_argument('-b', '--baudrate', dest='motor_baud', type=int,
                    help="Bus baudrate, baudrates: 1 - 1M, 3 - 500 000, 4 - 400 000, 7 - 250 000, 9 - 200 000, 16 - 115200, 34 - 57600, 103 - 19200, 207 - 9600",
                    required=False, default=1)
parser.add_argument('--connect-baudrate', dest='bus_baud', type=int,
                    help="Bus baudrate used to first connec to motor, baudrates: 1 - 1M, 3 - 500 000, 4 - 400 000, 7 - 250 000, 9 - 200 000, 16 - 115200, 34 - 57600, 103 - 19200, 207 - 9600",
                    required=False, default=1)

parser.add_argument('--return_delay_time',   dest='return_delay_time',   type=int, help="Return time of message", required=False, default=250)
parser.add_argument('--temperature_limit',   dest='temperature_limit',   type=int, help="Max safe temperature of motor", required=False, default=70)
parser.add_argument('--min_voltage_limit',   dest='min_voltage_limit',   type=int, help="Minimal save voltage", required=False, default=60)
parser.add_argument('--max_voltage_limit',   dest='max_voltage_limit',   type=int, help="Maximal safe voltage", required=False, default=140)
parser.add_argument('--status_return_level', dest='status_return_level', type=int, help="Status warning level. Reffer to manual", required=False, default=2)
parser.add_argument('--alarm_led',           dest='alarm_led',           type=int, help="Status warning level displayed at LED. Reffer to manual", required=False, default=36)

parser.add_argument('--cw_angle_limit',      dest='cw_angle_limit',      type=int, help="Clockwise Angle Limit", required=False, default=0)
parser.add_argument('--ccw_angle_limit',     dest='ccw_angle_limit',     type=int, help="Counter-Clockwise Angle Limit", required=False, default=1023)
parser.add_argument('--max_torque',          dest='max_torque',          type=int, help="Maximum Torque", required=False, default=1023)

args = parser.parse_args()

bauds = {
    1 : 1000000,
    3 : 500000,
    4 : 400000,
    7 : 250000,
    9 : 200000,
    16 : 115200,
    34 : 57600,
    103 : 19200,
    207 : 9600
}

# Reset motor to factory settings
port_handler = DynamixelPort(port_id=args.serial, pin=args.pin, baudrate=bauds[args.bus_baud])
broadcast = Motor(port_handler, 0xFE)
broadcast.factory_reset()
time.sleep(3)

# Set parameters
del port_handler
port_handler = DynamixelPort(port_id=args.serial, pin=args.pin, baudrate=bauds[1])
motor = Motor(port_handler, 0x01)
motor.eeprom.id.value = [args.motor_id]
motor.eeprom.baudrate.value = [args.motor_baud]
motor.eeprom.return_delay_time.value = [args.return_delay_time]
motor.eeprom.temperature_limit.value = [args.temperature_limit]
motor.eeprom.min_voltage_limit.value = [args.min_voltage_limit]
motor.eeprom.max_voltage_limit.value = [args.max_voltage_limit]
motor.eeprom.status_return_level.value = [args.status_return_level]
motor.eeprom.alarm_led.value = [args.alarm_led]

motor.eeprom.cw_angle_limit.value = motor._split_bytes(args.cw_angle_limit)
motor.eeprom.ccw_angle_limit.value = motor._split_bytes(args.ccw_angle_limit)
motor.eeprom.max_torque.value = motor._split_bytes(args.max_torque)

motor.action()

time.sleep(1)

# Validate parameters
del port_handler
port_handler = DynamixelPort(port_id=args.serial, pin=args.pin, baudrate=bauds[args.motor_baud])
motor = Motor(port_handler, args.motor_id)
print(f'''
\t baudrate:            {motor.eeprom.baudrate.value} : {[args.motor_baud]}
\t return_delay_time:   {motor.eeprom.return_delay_time.value} : {[args.return_delay_time]}
\t temperature_limit:   {motor.eeprom.temperature_limit.value} : {[args.temperature_limit]}
\t min_voltage_limit:   {motor.eeprom.min_voltage_limit.value} : {[args.min_voltage_limit]}
\t max_voltage_limit:   {motor.eeprom.max_voltage_limit.value} : {[args.max_voltage_limit]}
\t status_return_level: {motor.eeprom.status_return_level.value} : {[args.status_return_level]}
\t alarm_led:           {motor.eeprom.alarm_led.value} : {[args.alarm_led]}
\t cw_angle_limit:      {motor.eeprom.cw_angle_limit.value} : {motor._split_bytes(args.cw_angle_limit)} 
\t ccw_angle_limit:     {motor.eeprom.ccw_angle_limit.value} : {motor._split_bytes(args.ccw_angle_limit)} 
\t max_torque:          {motor.eeprom.max_torque.value} : {motor._split_bytes(args.max_torque)} 
''')