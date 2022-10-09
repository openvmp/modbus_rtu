import sys

sys.path.append("../serial/test/lib")

from serial_test import SerialTesterNode

import rclpy
from rclpy.node import Node as rclpyNode
from serial.srv import InjectOutput
from modbus.srv import HoldingRegisterRead
import std_msgs.msg


class ModbusRtuTesterNode(SerialTesterNode):
    test_context = {}

    def __init__(self, name="modbus_rtu_tester_node"):
        super().__init__(name)

    def subscribe_modbus_rtu(self, id):
        self.create_subscription(
            std_msgs.msg.UInt32,
            "/modbus/bus" + str(id) + "/rtu/crc_check_failed",
            lambda msg: print("RTU: CRC check failed!"),
            10,
        )
        self.create_subscription(
            std_msgs.msg.UInt32,
            "/modbus/bus" + str(id) + "/rtu/unwanted_input",
            lambda msg: print("RTU: Unwanted input received!"),
            10,
        )
        self.create_subscription(
            std_msgs.msg.UInt32,
            "/modbus/bus" + str(id) + "/rtu/partial_input",
            lambda msg: print("RTU: Partial input received!"),
            10,
        )

    def modbus_holding_register_read(self, bus, leaf_id, addr, count, timeout=10.0):
        client = self.create_client(
            HoldingRegisterRead, "/modbus/bus" + str(bus) + "/holding_register_read"
        )
        ready = client.wait_for_service(timeout_sec=timeout)
        if not ready:
            raise RuntimeError("Wait for service timed out")

        request = HoldingRegisterRead.Request()
        request.leaf_id = leaf_id
        request.addr = addr
        request.count = count

        future = client.call_async(request)
        return future
