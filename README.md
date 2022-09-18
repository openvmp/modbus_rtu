# ROS2 Modbus RTU

## Introduction

This package is an ultimate C++ implementation of Modbus RTU for ROS2. It is
intended to implement in

It can be used either as a library or a standalone process. In both cases it
provides messaging topics for introspection and debugging. It performs wisely
in case of serial line saturation in any of the I/O directions.

It is a part of [the OpenVMP project](https://github.com/openvmp/openvmp)
But it is made to be universal and usable anywhere.


## Overview

There are two ways how to use ROS2 Modbus RTU in your projects.

### Modbus RTU via ROS2 interface

This way the devices get exposed to all other ROS2 nodes (including remote ones
and troubleshooting/debugging tools).

```mermaid
flowchart TB
    cli["# Modbus debugging\n$ ros2 topic echo /modbus/example_bus/01"] -. "DDS" .-> topic_modbus
    app["Any application"] -- "DDS\n(with context switch)" --> topic_modbus
    cli_serial["# Serial debugging\n$ ros2 topic echo /serial"] -. "DDS" ....-> topic_serial[/ROS2 interfaces:\n/serial/.../]
    subgraph modbus_exe["Process: modbus_rtu_standalone"]
      subgraph modbus["Library: modbus"]
        topic_modbus[/ROS2 interfaces:\n/modbus/example_bus/01/] --> driver_modbus["Modbus implementation"]
      end
      subgraph serial["Library: serial"]
        topic_serial --> driver["Serial port driver"]
      end
      driver_modbus --> rtu["Modbus RTU implementation"]
      rtu -- "DDS\n(potentially without\ncontext switch)" --> topic_serial
      rtu -- "or native API calls" --> driver
      driver --> file{{"Character device"}}
    end
```

### Modbus RTU via native API interface

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
      code["Your code"] -- "Native API calls" --> driver_modbus
      code["Your code"] -- "or DDS\n(potentially without\ncontext switch)" --> topic_modbus
      subgraph modbus_exe["Library: modbus_rtu_standalone"]
        subgraph modbus["Library: modbus"]
          topic_modbus[/ROS2 interfaces:\n/modbus/example_bus/01/] --> driver_modbus["Modbus implementation"]
        end
        subgraph serial["Library: serial"]
          topic_serial --> driver["Serial port driver"]
        end
        driver_modbus --> rtu["Modbus RTU implementation"]
        rtu -- "Native API calls" --> driver
        driver --> file{{"Character device"}}
      end
    end
    cli["# Modbus debugging\n$ ros2 topic echo /modbus/example_bus/01"] -. "DDS" ..-> topic_modbus
```
