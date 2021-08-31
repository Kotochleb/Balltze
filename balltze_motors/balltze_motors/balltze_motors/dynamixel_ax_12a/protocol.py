import time
from serial import Serial
import RPi.GPIO as GPIO


from .motor import Motor as Motor

class DynamixelPort:
    PROTOCOL_SLEEP = 0.0001
    DIRECTION_SWITCH_TIME = 0.0001
    DIRECTION_RX = GPIO.LOW
    DIRECTION_TX = GPIO.HIGH

    def __init__(self, port_id='/dev/ttyAMA0', pin=18):
        self._port_id = port_id
        self._data_pin = pin
        self._port = Serial(self._port_id, baudrate=1000000, timeout=0.001)
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._data_pin, GPIO.OUT)

    def data_direction(self, dir):
        GPIO.output(self._data_pin, dir)
        time.sleep(DynamixelPort.DIRECTION_SWITCH_TIME)

    def write(self, packet_id, length, instruction, params):
        self.data_direction(DynamixelPort.DIRECTION_TX)
        time.sleep(DynamixelPort.PROTOCOL_SLEEP)
        payload = [0xFF]
        payload.append(0xFF)
        payload.append(packet_id)
        payload.append(length)
        payload.append(instruction)
        payload.extend(params)
        payload.append(self.checksum(payload[2:]))
        print(payload)
        self._port.write(payload)


    def write_cell(self, packet_id, length, instruction, cell, params):
        payload = [cell]
        payload.extend(params)
        self.write(packet_id, length, instruction, payload)


    def read(self, packet_id):
        self.data_direction(DynamixelPort.DIRECTION_RX)
        time.sleep(DynamixelPort.PROTOCOL_SLEEP)
        response = self._port.read(5)
        print(response)
        if response[0] != 0xFF or response[1] != 0xFF:
            raise ValueError
        if response[2] != packet_id:
            raise ValueError
        
        resp_len = response[3]
        payload = self._port.read(resp_len)
        error = payload[0]
        if error:
            raise ValueError
            # TODO implement error handling

        checksum = (self.checksum(response + payload[:-1]))
        if payload[-1] != checksum:
            raise ValueError()

        if resp_len > 0:
            return payload[1:-1]
        return []


    def flush(self):
        self._port.flush()

    def checksum(self, payload):
        return (~(sum(payload))) & 0xFF