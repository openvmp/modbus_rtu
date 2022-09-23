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

#include "modbus_rtu/node.hpp"

namespace modbus_rtu {

Node::Node(std::shared_ptr<serial::InterfaceNative> prov)
    : rclcpp::Node::Node("modbus_rtu"), prov_{prov} {
  intf_ =
      std::shared_ptr<ModbusRtuInterface>(new ModbusRtuInterface(this, prov));
}

}  // namespace modbus_rtu
