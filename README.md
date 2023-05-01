# ROS2 Modbus RTU driver

[![License](./apache20.svg)](./LICENSE.txt)

This package is an ultimate C++ implementation of Modbus RTU for ROS2.

It can be used either as a library or a standalone process. In both cases it
provides messaging topics for introspection and debugging. It performs wisely
in case of serial line saturation in any of the I/O directions.

### Modbus RTU via ROS2 interface

```
$ ros2 run remote_modbus_rtu remote_modbus_rtu_standalone \
  --ros-args \
  --remap modbus_rtu:__node:=modbus_rtu_example_bus \
  -p modbus_is_remote:=false \
  -p modbus_prefix:=/modbus/example_bus \
  -p serial_is_remote:=false \
  -p serial_prefix:=/serial/com1 \
  -p serial_dev_name:=/dev/ttyS0 \
  -p serial_baud_rate:=115200 \
  -p serial_data:=8 \
  -p serial_parity:=false \
  -p serial_stop:=1 \
  -p serial_flow_control:=true
...

$ ros2 run <<<your package>>> <<<your executable>>> \
   --ros-args \
   -p modbus_prefix:=/modbus/example_bus \
   ...
```

This way the devices get exposed to all other ROS2 nodes (including remote ones
and troubleshooting/debugging tools).

```mermaid
flowchart TB
    cli["<p style='text-align:left;'># Modbus debugging\n$ ros2 topic echo /modbus/example_bus/id01</p>"] -. "DDS" .-> topic_modbus
    app["Any application"] -- "DDS\n(with context switch)" --> topic_modbus
    cli_serial["<p style='text-align:left;'># Serial debugging\n$ ros2 topic echo /serial</p>"] -. "DDS" ....-> topic_serial[/ROS2 interfaces:\n/serial/com1/.../]
    subgraph modbus_exe["Process: remote_modbus_rtu_standalone"]
      subgraph modbus["Library: remote_modbus"]
        topic_modbus[/ROS2 interfaces:\n/modbus/example_bus/id01/.../] --> driver_modbus["Modbus implementation"]
      end
      subgraph serial["Library: remote_serial"]
        topic_serial --> driver["Serial port driver"]
      end
      driver_modbus --> rtu["Modbus RTU implementation"]
      rtu -- "DDS\n(potentially without\ncontext switch)" --> topic_serial
      rtu -- "or native API calls\n(no context switch)" --> driver
      driver --> file{{"Character device"}}
    end
```

### Modbus RTU via native API interface

```
$ ros2 run <<<your package>>> <<<your executable>>> \
  --ros-args \
  --remap modbus_rtu:__node:=modbus_rtu_example_bus \
  -p modbus_is_remote:=false \
  -p modbus_prefix:=/modbus/example_bus \
  -p serial_is_remote:=false \
  -p serial_prefix:=/serial/com1 \
  -p serial_dev_name:=/dev/ttyS0 \
  -p serial_baud_rate:=115200 \
  -p serial_data:=8 \
  -p serial_parity:=false \
  -p serial_stop:=1 \
  -p serial_flow_control:=true
...
```
This way the devices can be accessed directly (without DDS calls via networking
stack) for best performance. If the performance benefits are not critical, the
same ROS2 messaging interface can also be used even if the library is linked
into the same process (for simple packaging or any other reason).

At the same time, the same bus will be expose to other ROS2 nodes
via DDS. However the performance benefit will only be enjoyed by the process
that is linked with this library directly.

```mermaid
flowchart TB
    cli_serial["# Serial debugging\n$ ros2 topic echo /serial"] -. "DDS" ....-> topic_serial[/ROS2 interfaces:\n/serial/.../]
    subgraph app["Your application"]
      code["Your code"] -- "Native API calls\n(no context switch)" --> driver_modbus
      code["Your code"] -- "or DDS\n(potentially without\ncontext switch)" --> topic_modbus
      subgraph modbus_exe["Library: remote_modbus_rtu"]
        subgraph modbus["Library: remote_modbus"]
          topic_modbus[/ROS2 interfaces:\n/modbus/example_bus/id01/] --> driver_modbus["Modbus implementation"]
        end
        subgraph serial["Library: remote_serial"]
          topic_serial --> driver["Serial port driver"]
        end
        driver_modbus --> rtu["Modbus RTU implementation"]
        rtu -- "Native API calls" --> driver
        driver --> file{{"Character device"}}
      end
    end
    cli["# Modbus debugging\n$ ros2 topic echo /modbus/example_bus/id01"] -. "DDS" ..-> topic_modbus
```

### Usage

#### Configurable

The following code uses either local native API or remote ROS2 interface calls depending on the parameter "modbus_is_remote":

```c++
#include "remote_modbus_rtu/factory.hpp"

...
  auto modbus_rtu_impl = remote_modbus_rtu::Factory::New(this);
  modbus_rtu_impl->holding_register_read(...);
...
```

#### Deterministic

The following code initializes Modbus RTU locally and uses native API calls:

```c++
#include "remote_modbus_rtu/implementation.hpp"

...
  auto modbus_rtu_impl = std::make_shared<remote_modbus_rtu::Implementation>(this);
  modbus_rtu_impl->holding_register_read(...);
...
```
