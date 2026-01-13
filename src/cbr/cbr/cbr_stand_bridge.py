import serial
import threading
import time
import sys
import re
from serial.tools import list_ports
from rclpy.node import Node

BAUD_RATE = 115200

class CBRStandBridge(Node):
    def __init__(self):
        super().__init__('cbr_stand_bridge')

        self.declare_parameter('buadrate', BAUD_RATE)

        self._load_params()

    def _load_params(self):
        """Load parameters from the parameter server."""
        self.can_bitrate = self.get_parameter('buadrate').value