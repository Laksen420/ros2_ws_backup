// rfid_node.cpp
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Include the CAEN RFID Light library (and related host functions)
// Wrap them in extern "C" to disable C++ name mangling.
extern "C" {
#include "CAENRFIDLib_Light.h"
#include "host.h"
#include "IO_Light.h"
#include "Protocol_Light.h"
}

// Use a shorthand for the std::chrono literals
using namespace std::chrono_literals;

// A simple ROS2 node that publishes detected RFID tag IDs
class RFIDNode : public rclcpp::Node
{
public:
  RFIDNode()
  : Node("rfid_node")
  {
    // Create a publisher (e.g. publishing a string on topic "rfid_tag")
    publisher_ = this->create_publisher<std_msgs::msg::String>("rfid_tag", 10);

    // Create a timer callback to run inventory every second.
    timer_ = this->create_wall_timer(1s, std::bind(&RFIDNode::timer_callback, this));

    // Initialize the RFID reader structure with host functions.
    // These functions come from host.c and are declared in host.h.
    reader_.connect       = _connect;
    reader_.disconnect    = _disconnect;
    reader_.tx            = _tx;
    reader_.rx            = _rx;
    reader_.clear_rx_data = _clear_rx_data;
    reader_.enable_irqs   = _enable_irqs;
    reader_.disable_irqs  = _disable_irqs;

    // Set up RS232 connection parameters. (Adjust the COM port if needed.)
    port_params_.com         = (char*)"/dev/ttyACM0";
    port_params_.baudrate    = 921600;
    port_params_.dataBits    = 8;
    port_params_.stopBits    = 1;
    port_params_.parity      = 0;
    port_params_.flowControl = 0;

    // Connect the RFID reader using the CAEN RFID API.
    CAENRFIDErrorCodes ec = CAENRFID_Connect(&reader_, CAENRFID_RS232, &port_params_);
    if (ec != CAENRFID_StatusOK) {
      RCLCPP_ERROR(this->get_logger(), "RFID connect failed (error %d)", ec);
    } else {
      RCLCPP_INFO(this->get_logger(), "RFID reader connected");
    }
  }

  ~RFIDNode()
  {
    // Always disconnect when shutting down.
    CAENRFID_Disconnect(&reader_);
  }

private:
  void timer_callback()
  {
    // This example performs a simple inventory round.
    // The API (see example.c) returns a linked list of tags.
    CAENRFIDTagList* tag_list = nullptr;
    uint16_t numTags = 0;
    // We pass a flag of 0 for a standard (non-continuous) inventory.
    CAENRFIDErrorCodes ec = CAENRFID_InventoryTag(
      &reader_,         // reader pointer
      (char*)"Source_0",// source name (adjust if needed)
      0, 0, 0,          // no mask
      NULL, 0, 0,       // no mask length and flag = 0
      &tag_list,        // pointer to tag list pointer
      &numTags);        // number of tags

    if (ec != CAENRFID_StatusOK) {
      RCLCPP_WARN(this->get_logger(), "Inventory error: %d", ec);
      return;
    }

    // If at least one tag was detected, publish its ID (we use the first tag)
    if (numTags > 0 && tag_list != nullptr) {
      // Convert the tag's binary ID to a hex string.
      char tag_hex[2 * MAX_ID_LENGTH + 1] = {0};
      for (int i = 0; i < tag_list->Tag.Length; i++) {
        sprintf(&tag_hex[i * 2], "%02X", tag_list->Tag.ID[i]);
      }

      auto message = std_msgs::msg::String();
      message.data = std::string(tag_hex);
      publisher_->publish(message);

      RCLCPP_INFO(this->get_logger(), "Published tag: %s", tag_hex);
    } else {
      RCLCPP_INFO(this->get_logger(), "No tag detected");
    }

    // Free the tag list if it was allocated.
    while (tag_list != nullptr) {
      CAENRFIDTagList* next = tag_list->Next;
      free(tag_list);
      tag_list = next;
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // The RFID reader object (from CAENRFIDTypes_Light.h)
  CAENRFIDReader reader_;
  // RS232 connection parameters (from host.h)
  RS232_params port_params_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RFIDNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
