#include <arpa/inet.h>

#include <chrono>
#include <cstdlib>

#include "remote_modbus/protocol.hpp"
#include "remote_modbus_rtu/implementation.hpp"
#include "remote_modbus_rtu/node.hpp"
#include "remote_serial/utils.hpp"



using namespace std::chrono_literals;

namespace remote_modbus_rtu {


rclcpp::FutureReturnCode Implementation::coil_read_handler_real_(
    const std::shared_ptr<remote_modbus::srv::CoilRead::Request>
        request,
    std::shared_ptr<remote_modbus::srv::CoilRead::Response>
        response) {

  static const uint8_t fc = MODBUS_FC_READ_COIL;
  uint8_t data[] = {
      request->leaf_id,
      fc,
      (uint8_t)((request->addr & 0xFF00) >> 8),   // high
      (uint8_t)(request->addr & 0xFF),            // low
      (uint8_t)((request->count & 0xFF00) >> 8),  // high
      (uint8_t)(request->count & 0xFF),           // low
      0,                                          // crc high
      0                                           // crclow
  };
  std::string output = modbus_rtu_frame_(data, sizeof(data));

  auto result = send_request_(request->leaf_id, fc, output);
  if (result.length() < 2) {  // 2 = fc + exception_code (or len)
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }
  int bit_counter = 0;
  uint8_t fc_received = (uint8_t)result[0];
  switch (fc_received) {
    case fc:
      // See if we have amount of data that is consistent with length
      response->len = result[1];    //result[1] is the response length in bytes
      // Read the dynamic part in
      for (int i = 0; i < (response->len); i++) {    
        for(int bit_ = 0; bit_ < 8; bit_++){
          auto _result_ = (result[2 +  i] >> bit_)&1;
          response->values.push_back(*(bool *)&_result_);
          bit_counter++;
          if(bit_counter == request->count)
            {break;}
        }
      }


      return rclcpp::FutureReturnCode::SUCCESS;

    case 0x80 | fc:
      // this is an error report
      response->exception_code = (uint8_t)result[1];
      /* fall through */

    default:
      return rclcpp::FutureReturnCode::INTERRUPTED;
  }
}



rclcpp::FutureReturnCode Implementation::coil_write_handler_real_(
    const std::shared_ptr<remote_modbus::srv::CoilWrite::Request>
        request,
    std::shared_ptr<remote_modbus::srv::CoilWrite::Response>
        response) {
  static const uint8_t fc = MODBUS_FC_WRITE_COIL;
  if (request->value == 1){
    request->value = 0xFF00;}
  uint8_t data[] = {
      request->leaf_id,
      fc,
      (uint8_t)((request->addr & 0xFF00) >> 8),   // high
      (uint8_t)(request->addr & 0xFF),            // low
      (uint8_t)((request->value & 0xFF00) >> 8),  // high
      (uint8_t)(request->value & 0xFF),           // low
      0,                                          // crc high
      0                                           // crclow
  };
  std::string output = modbus_rtu_frame_(data, sizeof(data));

  auto result = send_request_(request->leaf_id, fc, output);
  if (result.length() < 2) {  // 2 = fc + exception_code (or len)
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }
  uint8_t fc_received = (uint8_t)result[0];
  switch (fc_received) {
    case fc:
      // See if we have amount of data that is consistent with length
      response->addr = ntohs(*(uint16_t *)&result[1]);
      response->value = ntohs(*(uint16_t *)&result[3]);

      return rclcpp::FutureReturnCode::SUCCESS;

    case 0x80 | fc:
      // this is an error report
      response->exception_code = (uint8_t)result[1];
      /* fall through */

    default:
      return rclcpp::FutureReturnCode::INTERRUPTED;
  }
}

rclcpp::FutureReturnCode Implementation::coil_continuous_write_handler_real_(
    const std::shared_ptr<remote_modbus::srv::CoilContinuousWrite::Request>
        request,
    std::shared_ptr<remote_modbus::srv::CoilContinuousWrite::Response>
        response) {
  static const uint8_t fc = MODBUS_FC_WRITE_COIL_CONTINUOUS;
  //if (request->value == 1){
  //  request->value = 0xFF00;}

  uint8_t request_size =  (request->count + 7)/8; // count is number of coils ,data size in bytes
  int loop_counter = 0;
  std::vector<uint8_t> values_ ;
  for (int i = 0; i < request_size; i++ )
  {
      for (int j = 0; j<8  ;j++){
        values_[i] = (values_[i] << 1) | request->values[j];
        if(loop_counter ==request->count )
          {break;}
        loop_counter++;
      }
  }

  size_t  data_size = (size_t)(request_size + 7 + 2); // number of index in data vector
  
  std::vector<uint8_t> data = {
      request->leaf_id,
      fc,
      (uint8_t)((request->addr & 0xFF00) >> 8),   // high
      (uint8_t)(request->addr & 0xFF),            // low
      (uint8_t)((request->count & 0xFF00) >> 8),  // high
      (uint8_t)(request->count & 0xFF),           // low
      request_size,
  };

    // start filling out
    for (size_t i = 0; i < request_size; ++i) {
      data[7 + i ] = values_[i]; // high byte
      }
  data[data_size - 1 ] = 0;   // CRC
  data[data_size - 2 ] = 0;   // CRC


  std::string output = modbus_rtu_frame_(&data[0], data.size());

  auto result = send_request_(request->leaf_id, fc, output);
  if (result.length() < 2) {  // 2 = fc + exception_code (or len)
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }
  uint8_t fc_received = (uint8_t)result[0];
  switch (fc_received) {
    case fc:
      // See if we have amount of data that is consistent with length
      response->addr = ntohs(*(uint16_t *)&result[1]);
      response->count = ntohs(*(uint16_t *)&result[2]);

      return rclcpp::FutureReturnCode::SUCCESS;

    case 0x80 | fc:
      // this is an error report
      response->exception_code = (uint8_t)result[1];
      /* fall through */

    default:
      return rclcpp::FutureReturnCode::INTERRUPTED;
  }
}
}