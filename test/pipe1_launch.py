import os
import struct
import tempfile
from time import sleep

import rclpy
from rclpy.node import Node as rclpyNode
from serial.srv import InjectOutput
from modbus.srv import HoldingRegisterRead
import std_msgs.msg


import pytest
import unittest
from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    ExecuteProcess,
)
from launch_testing.actions import ReadyToTest
import launch_testing.markers
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable


TTY1 = "/tmp/ttyS21"
TTY2 = "/tmp/ttyS22"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():

    socat = ExecuteProcess(
        name="socat",
        cmd=[
            [
                FindExecutable(name="socat"),
                " -s",
                " PTY,rawer,link=",
                TTY1,
                " PTY,rawer,link=",
                TTY2,
            ]
        ],
        shell=True,
    )
    node1 = Node(
        name="modbus_rtu_com1",
        package="modbus_rtu",
        executable="modbus_rtu_standalone",
        # arguments=["--ros-args", "--log-level", "debug"],
        parameters=[
            {
                "modbus_prefix": "/modbus/bus1",
                "serial_prefix": "/serial/com1",
                "serial_dev_name": TTY1,
                # "serial_skip_init": True,
                "serial_baud_rate": 115200,
                "serial_data": 8,
                "serial_parity": False,
                "serial_stop": 1,
                "serial_flow_control": True,
            }
        ],
        output="screen",
    )
    node2 = Node(
        name="serial_com2",
        package="serial",
        executable="serial_standalone",
        # arguments=["--ros-args", "--log-level", "debug"],
        parameters=[
            {
                "serial_prefix": "/serial/com2",
                "serial_dev_name": TTY2,
                # "serial_skip_init": True,
                "serial_baud_rate": 115200,
                "serial_data": 8,
                "serial_parity": False,
                "serial_stop": 1,
                "serial_flow_control": True,
            }
        ],
        output="screen",
    )

    return (
        LaunchDescription(
            [
                socat,
                RegisterEventHandler(
                    event_handler=OnProcessStart(
                        target_action=socat,
                        on_start=[
                            node1,
                            node2,
                        ],
                    )
                ),
                ReadyToTest(),
            ]
        ),
        {"socat": socat, "serial_com1": node1, "serial_com2": node2},
    )


class TestHoldingRegisterRead1(unittest.TestCase):
    def test_basic(self, proc_output):
        rclpy.init()
        try:
            sleep(3)
            node = MakeTesterNode("test_node")
            node.subscribe(1)
            node.subscribe(2)
            node.subscribe(1, "output")
            node.subscribe(2, "output")
            node.subscribe_modbus_rtu(1)

            response = node.modbus_holding_register_read(1, 1, 1, 1)
            assert response, "Could not inject!"
            sleep(3)

            assert response.len == 2, "value mismatch"
            assert response.values[0] == 16705, "value mismatch"
        finally:
            rclpy.shutdown()
            _ignore = 1


class MakeTesterNode(rclpyNode):
    test_context = {}

    def __init__(self, name="tester_node"):
        super().__init__(name)

    def _save(self, id, direction, msg):
        print("Data received on com" + str(id) + " on " + direction)
        if not id in self.test_context:
            self.test_context[id] = {}
        if not direction in self.test_context[id]:
            self.test_context[id][direction] = ""
        self.test_context[id][direction] += msg.data

    def subscribe(
        self,
        id=1,
        direction="input",
    ):
        this_node = self
        subscription = self.create_subscription(
            std_msgs.msg.String,
            "/serial/com" + str(id) + "/inspect/" + direction,
            lambda msg: this_node._save(id, direction, msg),
            10,
        )
        return subscription

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

    def inject(self, id=1, direction="output", text="test", timeout=10.0):
        client = self.create_client(
            InjectOutput, "/serial/com" + str(id) + "/inject/" + str(direction)
        )
        ready = client.wait_for_service(timeout_sec=timeout)
        if not ready:
            raise RuntimeError("Wait for service timed out")

        request = InjectOutput.Request()
        request.data = str(text, "latin_1", "ignore")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        assert future.done(), "Client request timed out"

        _response = future.result()

        return True

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
        sleep(1)
        self.inject(2, "output", bytes.fromhex("01 03 02 41 41 24 48"))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)

        assert future.done(), "Client request timed out"

        response = future.result()
        print(response)

        return response
