/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_MODBUS_RTU_NODE_H
#define OPENVMP_MODBUS_RTU_NODE_H

#include <memory>
#include <string>

#include "modbus_rtu/interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace modbus_rtu {

class Node : public rclcpp::Node {
 public:
  Node();

 private:
  std::shared_ptr<ModbusRtuInterface> intf_;
};

}  // namespace modbus_rtu

#endif  // OPENVMP_MODBUS_RTU_NODE_H
