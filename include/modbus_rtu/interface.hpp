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
#include "serial/interface.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace modbus_rtu {

class ModbusRtuInterface : public modbus::ModbusInterface {
 public:
  ModbusRtuInterface(rclcpp::Node *node);
  virtual ~ModbusRtuInterface() {}

 protected:
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr rtu_crc_check_failed_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr rtu_unwanted_input_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr rtu_partial_input_;

  virtual rclcpp::FutureReturnCode holding_register_read_handler_real_(
      const std::shared_ptr<modbus::srv::HoldingRegisterRead::Request> request,
      std::shared_ptr<modbus::srv::HoldingRegisterRead::Response> response)
      override;
  virtual rclcpp::FutureReturnCode holding_register_write_handler_real_(
      const std::shared_ptr<modbus::srv::HoldingRegisterWrite::Request> request,
      std::shared_ptr<modbus::srv::HoldingRegisterWrite::Response> response)
      override;
  virtual rclcpp::FutureReturnCode
  holding_register_write_multiple_handler_real_(
      const std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Request>
          request,
      std::shared_ptr<modbus::srv::HoldingRegisterWriteMultiple::Response>
          response) override;
  virtual rclcpp::FutureReturnCode get_com_event_log_handler_real_(
      const std::shared_ptr<modbus::srv::GetComEventLog::Request> request,
      std::shared_ptr<modbus::srv::GetComEventLog::Response> response) override;
  virtual rclcpp::FutureReturnCode read_device_id_handler_real_(
      const std::shared_ptr<modbus::srv::ReadDeviceId::Request> request,
      std::shared_ptr<modbus::srv::ReadDeviceId::Response> response) override;

 private:
  serial::Interface prov_;

  class Promise {
   public:
    Promise(uint8_t leaf_id, uint8_t fc) : leaf_id{leaf_id}, fc{fc} {}
    uint8_t leaf_id;
    uint8_t fc;
    std::promise<std::string> promise;
  };
  // input_promises_ contain everyone waiting for their lef_id to repspond
  std::vector<Promise> input_promises_;
  // input_queue_ accumulates data from previous read()s until the entire frame
  // arrives
  std::string input_queue_;
  std::mutex input_promises_mutex_;

  // input_cb is a static method to receive callbacks from the serial module.
  static void input_cb_(const std::string &msg, void *user_data);
  // input_cb_real_ is the real handler of the callbacks from the serial module.
  void input_cb_real_(const std::string &msg);
  // send_request_ send a request and waits for a response
  std::string send_request_(uint8_t leaf_id, const std::string &output);
  std::string modbus_rtu_frame_(uint8_t *data, size_t size);

  // published values
  uint32_t rtu_crc_check_failed__value_;
  uint32_t rtu_unwanted_input__value_;
  uint32_t rtu_partial_input__value_;
};

}  // namespace modbus_rtu

#endif  // OPENVMP_MODBUS_RTU_INTERFACE_H