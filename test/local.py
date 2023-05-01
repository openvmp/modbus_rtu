import sys

sys.path.append("test/lib")

from modbus_rtu_test import ModbusRtuTesterNode

from time import sleep
import rclpy

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
        package="remote_modbus_rtu",
        executable="remote_modbus_rtu_standalone",
        # arguments=["--ros-args", "--log-level", "debug"],
        parameters=[
            {
                "serial_is_remote": False,
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
        package="remote_serial",
        executable="remote_serial_standalone",
        # arguments=["--ros-args", "--log-level", "debug"],
        parameters=[
            {
                "serial_is_remote": False,
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
        {"socat": socat, "modbus_rtu_com1": node1, "serial_com2": node2},
    )


class TestHoldingRegisterRead1(unittest.TestCase):
    def test_basic(self, proc_output):
        rclpy.init()
        try:
            sleep(3)
            node = ModbusRtuTesterNode()
            node.subscribe(1)
            node.subscribe(2)
            node.subscribe(1, "output")
            node.subscribe(2, "output")
            node.subscribe_modbus_rtu(1)

            future = node.modbus_holding_register_read(1, 1, 1, 1)
            sleep(2)
            node.inject(2, "output", bytes.fromhex("01 03 02 41 41 48 24"))
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)

            assert future.done(), "Client request timed out"

            response = future.result()
            print(response)
            assert response, "Could not inject!"
            sleep(3)

            assert response.exception_code == 0, "value mismatch"
            assert response.len == 2, "value mismatch"
            assert response.values[0] == 16705, "value mismatch"
        finally:
            rclpy.shutdown()
            _ignore = 1
