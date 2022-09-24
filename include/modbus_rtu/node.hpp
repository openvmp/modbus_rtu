/*
 * Copyright 2022 OpenVMP Authors
 *
 * Licensed under HIPPOCRATIC LICENSE Version 3.0.
 * Generated using
 * https://firstdonoharm.dev/version/3/0/bds-bod-cl-eco-ffd-media-mil-soc-sup-sv.md
 * See https://github.com/openvmp/openvmp/blob/main/docs/License.md for more
 * details.
 *
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
