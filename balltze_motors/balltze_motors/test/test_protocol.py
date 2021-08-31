#!/usr/bin/env python

import os
import sys
import pytest

sys.path.append(os.path.abspath(__file__ + '/../../'))

from balltze_motors.dynamixel_ax_12a.protocol import DynamixelPort


@pytest.mark.linter
@pytest.mark.checksum
def test_checksum():
    protocol = DynamixelPort(None, None)
    assert protocol.checksum([0x01, 0x02, 0x01]) == 0xFB
    assert protocol.checksum([0x01, 0x02, 0x00]) == 0xFC
    assert protocol.checksum([0x01, 0x02, 0x00]) == 0xFC
    assert protocol.checksum([0xFE, 0x02, 0x05]) == 0xFA
    assert protocol.checksum([0x01, 0x03, 0x00, 0x20]) ==  0xDB
    assert protocol.checksum([0x01, 0x04, 0x02, 0x2B, 0x01]) ==  0xCC
    assert protocol.checksum([0xFE, 0x04, 0x03, 0x03, 0x01]) == 0xF6
    assert protocol.checksum([0x01, 0x05, 0x04, 0x1E, 0xF4, 0x01]) == 0xE2