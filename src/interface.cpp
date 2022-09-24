/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "modbus_rtu/interface.hpp"

#include <chrono>
#include <cstdlib>

#include "modbus/protocol.hpp"
#include "modbus_rtu/node.hpp"
#include "serial/utils.hpp"

using namespace std::chrono_literals;

namespace modbus_rtu {

static const std::map<uint8_t, const size_t> fc_to_len_static = {
    {MODBUS_FC_READ_HOLDING_REGISTERS, 2 + 1 + 2},
    {MODBUS_FC_PRESET_SINGLE_REGISTER, 2 + 4 + 2},
    {MODBUS_FC_PRESET_MULTIPLE_REGISTERS, 2 + 4 + 2},
    {0x80, 2 + 1 + 2},
};
static const std::map<uint8_t, const size_t>
    fc_to_len_dynamic_short_multiplier_pos = {
        {MODBUS_FC_READ_HOLDING_REGISTERS, 2},
        {MODBUS_FC_PRESET_SINGLE_REGISTER, 0},
        {MODBUS_FC_PRESET_MULTIPLE_REGISTERS, 0},
        {0x80, 0},
};

// Compute the MODBUS RTU CRC
#if 0
static uint16_t modbus_rtu_crc(uint8_t buf[], int len) {
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];  // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {  // Loop over each bit
      if ((crc & 0x0001) != 0) {    // If the LSB is set
        crc >>= 1;                  // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else        // Else LSB is not set
        crc >>= 1;  // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or
  // swap bytes)
  return crc;
}
#else
static uint16_t modbus_rtu_crc(const uint8_t *nData, uint16_t wLength) {
  static const uint16_t wCRCTable[] = {
      0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601,
      0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0,
      0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81,
      0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801, 0X18C0, 0X1980, 0XD941,
      0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01,
      0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0,
      0X1680, 0XD641, 0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081,
      0X1040, 0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
      0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441, 0X3C00,
      0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41, 0XFA01, 0X3AC0,
      0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981,
      0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41,
      0X2D00, 0XEDC1, 0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541, 0X2700,
      0XE7C1, 0XE681, 0X2640, 0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0,
      0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281,
      0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
      0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01,
      0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1,
      0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80,
      0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40, 0XB401, 0X74C0, 0X7580, 0XB541,
      0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101,
      0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0,
      0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481,
      0X5440, 0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
      0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841, 0X8801,
      0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40, 0X4E00, 0X8EC1,
      0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1, 0X8581,
      0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341,
      0X4100, 0X81C1, 0X8081, 0X4040};

  uint8_t nTemp;
  uint16_t wCRCWord = 0xFFFF;

  while (wLength--) {
    nTemp = *nData++ ^ wCRCWord;
    wCRCWord >>= 8;
    wCRCWord ^= wCRCTable[nTemp];
  }
  return wCRCWord;
}
#endif

std::string ModbusRtuInterface::modbus_rtu_frame_(uint8_t *data, size_t size) {
  uint16_t crc = modbus_rtu_crc((uint8_t *)&data[0], size - 2);
  data[sizeof(data) - 2] = (crc & 0xFF00) >> 8;  // high
  data[sizeof(data) - 1] = crc & 0xFF;           // low

  return std::string((char *)&data[0], sizeof(data));
}

ModbusRtuInterface::ModbusRtuInterface(rclcpp::Node *node)
    : ModbusInterface(node), prov_(node) {
  rtu_crc_check_failed_ = node->create_publisher<std_msgs::msg::UInt32>(
      interface_prefix_.as_string() + "/rtu/crc_check_failed", 10);
  rtu_unwanted_input_ = node->create_publisher<std_msgs::msg::UInt32>(
      interface_prefix_.as_string() + "/rtu/unwanted_input", 10);
  rtu_partial_input_ = node->create_publisher<std_msgs::msg::UInt32>(
      interface_prefix_.as_string() + "/rtu/partial_input", 10);

  prov_.register_input_cb(&ModbusRtuInterface::input_cb_, this);
}

/* static */ void ModbusRtuInterface::input_cb_(const std::string &msg,
                                                void *user_data) {
  (void)msg;
  (void)user_data;

  ModbusRtuInterface *that = (ModbusRtuInterface *)user_data;
  that->input_cb_real_(msg);
}

void ModbusRtuInterface::input_cb_real_(const std::string &msg) {
  input_promises_mutex_.lock();

  // TODO(clairbee): optimize it to avoid excessive copying
  input_queue_ += msg;
  RCLCPP_DEBUG(node_->get_logger(), "Received data: %s",
               (serial::utils::bin2hex(msg)).c_str());

  // check if there is anything to do
  while (input_queue_.length() > 0 && input_promises_.size() != 0) {
    if (input_queue_.length() == 1) {
      // there is not enough data yet
      input_promises_mutex_.unlock();
      return;
    }

    uint8_t received_leaf_id = (uint8_t)input_queue_[0];
    uint8_t received_fc = (uint8_t)input_queue_[1];

    auto p = input_promises_.begin();
    for (; p != input_promises_.end(); p++) {
      uint8_t expected_leaf_id = p->leaf_id;
      uint8_t expected_fc = p->fc;

      RCLCPP_DEBUG(node_->get_logger(),
                   "Expected: %02x %02x, Received: %02x %02x", expected_leaf_id,
                   expected_fc, received_leaf_id, received_fc);
      if (received_leaf_id == expected_leaf_id &&
          (received_fc == expected_fc || received_fc == (expected_fc | 0x80))) {
        break;
      }
    }

    if (p == input_promises_.end()) {
      // Not found anyone waiting on these 2 bytes, dropping 1 of them in the
      // attempt to recover
      MODBUS_PUBLISH_INC(UInt32, rtu_unwanted_input_, 1);
      input_queue_.erase(0, 1);
      continue;
    }
    // We've found the promise waiting on this leaf_id and fc

    // check if this is an exception
    if (received_fc & 0x80) {
      received_fc = 0x80;  // use this constant value while calculating length
    }

    // Input data validation
    if (fc_to_len_static.find(received_fc) == fc_to_len_static.end()) {
      // Unsupported fc, drop 1 byte in an attempt to recover
      input_queue_.erase(0, 1);
      continue;
    }

    // See if we have all the data delivered
    size_t expected_static_len = fc_to_len_static.at(received_fc);
    if (input_queue_.length() < expected_static_len) {
      // not enough data to parse it yet
      MODBUS_PUBLISH_INC(UInt32, rtu_partial_input_, 1);
      input_promises_mutex_.unlock();
      return;
    }
    size_t expected_dynamic_len = 0;
    size_t expected_dynamic_len_pos =
        fc_to_len_dynamic_short_multiplier_pos.at(received_fc);
    if (expected_dynamic_len_pos != 0) {
      expected_dynamic_len = (uint8_t)input_queue_[expected_dynamic_len_pos];
    }
    size_t expected_len = expected_static_len + expected_dynamic_len;
    if (input_queue_.length() < expected_len) {
      // not enough data to parse it yet
      MODBUS_PUBLISH_INC(UInt32, rtu_partial_input_, 1);
      input_promises_mutex_.unlock();
      return;
    }

    // Verify CRC
    uint16_t crc = ntohs(*(uint16_t *)(&input_queue_[expected_len - 2]));
    uint16_t expected_crc =
        modbus_rtu_crc((uint8_t *)&input_queue_[0], expected_len - 2);
    if (crc != expected_crc) {
      // CRC check failed, dropping the first byte in an attempt to find the
      // next message somewhere in this body
      RCLCPP_DEBUG(
          node_->get_logger(), "CRC failed, expected: %s",
          (serial::utils::bin2hex(std::string((char *)&expected_crc, 2)))
              .c_str());
      MODBUS_PUBLISH_INC(UInt32, rtu_crc_check_failed_, 1);
      input_queue_.erase(0, 1);
      continue;
    }

    // CRC is OK
    std::string response(&input_queue_[1], expected_len - 3);  // 3=leaf_id+crc
    p->promise.set_value(response);
    input_queue_.erase(0, expected_len);
    input_promises_.erase(p);
  }

  // input_queue_ is either already empty, or it's unwanted garbage (nobody is
  // waiting for input)
  input_queue_ = "";

  input_promises_mutex_.unlock();
}

std::string ModbusRtuInterface::send_request_(uint8_t leaf_id,
                                              const std::string &output) {
  input_promises_mutex_.lock();
  input_promises_.emplace_back(leaf_id, MODBUS_FC_READ_HOLDING_REGISTERS);
  std::future<std::string> f = input_promises_.back().promise.get_future();
  input_promises_mutex_.unlock();

  // Make sure to write after the promise is already queued
  prov_.output(output);

  // TODO(clairbee): implement a timeout here
  f.wait();
  RCLCPP_DEBUG(node_->get_logger(), "Future arrived");

  std::string result = f.get();
  RCLCPP_DEBUG(node_->get_logger(), "Received RTU response: %s",
               (serial::utils::bin2hex(result)).c_str());

  return result;
}

}  // namespace modbus_rtu