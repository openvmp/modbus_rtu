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

#ifndef OPENVMP_MODBUS_RTU_INTERFACE_H
#define OPENVMP_MODBUS_RTU_INTERFACE_H

#include <future>
#include <memory>
#include <string>

#include "modbus/interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/interface_native.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace modbus_rtu {

class Node;

class ModbusRtuInterface : public modbus::ModbusInterface {
 public:
  ModbusRtuInterface(Node *node, std::shared_ptr<serial::InterfaceNative> prov);
  virtual ~ModbusRtuInterface() {}

 protected:
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr rtu_crc_check_failed_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr rtu_unwanted_input_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr rtu_partial_input_;

  rclcpp::FutureReturnCode holding_register_read_handler_real_(
      const std::shared_ptr<modbus::srv::HoldingRegisterRead::Request> request,
      std::shared_ptr<modbus::srv::HoldingRegisterRead::Response> response)
      override;
  rclcpp::FutureReturnCode holding_register_write_handler_real_(
      const std::shared_ptr<modbus::srv::HoldingRegisterWrite::Request> request,
      std::shared_ptr<modbus::srv::HoldingRegisterWrite::Response> response)
      override;
  rclcpp::FutureReturnCode holding_register_write_multiple_handler_real_(
      const std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Request>
          request,
      std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Response>
          response) override;

 private:
  std::shared_ptr<serial::InterfaceNative> prov_;

  class Promise {
   public:
    Promise(uint8_t leaf_id, uint8_t fc) : leaf_id{leaf_id}, fc{fc} {}
    uint8_t leaf_id;
    uint8_t fc;
    std::promise<std::string> promise;
  };
  std::vector<Promise> input_promises_;
  std::string input_queue_;
  std::mutex input_promises_mutex_;

  static void input_cb_(const std::string &msg, void *user_data);
  void input_cb_real_(const std::string &msg);

  // published values
  uint32_t rtu_crc_check_failed__value_;
  uint32_t rtu_unwanted_input__value_;
  uint32_t rtu_partial_input__value_;
};

}  // namespace modbus_rtu

#endif  // OPENVMP_MODBUS_RTU_INTERFACE_H
