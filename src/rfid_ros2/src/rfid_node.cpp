#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"

// Include the CAEN RFID SDK headers with C linkage.
extern "C" {
  #include "CAENRFIDLib_Light.h"
  #include "host.h"
  #include "IO_Light.h"
  #include "Protocol_Light.h"
}

using namespace std::chrono_literals;

#define CAPTURE_WINDOW_MS 1000  // Capture window duration in milliseconds

// Structure to store a captured tag reading.
struct TagReading {
  std::string epc;
  int16_t rssi;
};

class RFIDNode : public rclcpp::Node
{
public:
  RFIDNode() : Node("rfid_node"), capturing_(false)
  {
    // Publisher for the best tag result.
    best_tag_pub_ = this->create_publisher<std_msgs::msg::String>("rfid_best_tag", 10);

    // Subscribe to the trigger topic.
    trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "trigger",
      10,
      std::bind(&RFIDNode::triggerCallback, this, std::placeholders::_1)
    );

    // Set up the RFID reader structure with host function pointers.
    reader_.connect       = _connect;
    reader_.disconnect    = _disconnect;
    reader_.tx            = _tx;
    reader_.rx            = _rx;
    reader_.clear_rx_data = _clear_rx_data;
    reader_.enable_irqs   = _enable_irqs;
    reader_.disable_irqs  = _disable_irqs;

    // RS232 parameters (adjust port if needed).
    port_params_.com         = (char*)"/dev/ttyACM0";  // adjust if needed
    port_params_.baudrate    = 921600;
    port_params_.dataBits    = 8;
    port_params_.stopBits    = 1;
    port_params_.parity      = 0;
    port_params_.flowControl = 0;

    // Connect to the RFID reader.
    CAENRFIDErrorCodes ec = CAENRFID_Connect(&reader_, CAENRFID_USB, &port_params_);
    if (ec != CAENRFID_StatusOK) {
      RCLCPP_ERROR(this->get_logger(), "Error connecting RFID reader: %d", ec);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "RFID reader connected");

    // NOTE: We do not call CAENRFID_SetSourceConfiguration, CAENRFID_AddReadPoint,
    // or CAENRFID_SetProtocol. The Light C SDK sample code in the full SDK typically
    // gets the logical source and then uses its default configuration.
    //
    // This avoids error code -7 and keeps the reader in a state where it can be reconnected.

    // Start a timer to poll for tag data (this callback is active only during a capture window).
    timer_ = this->create_wall_timer(10ms, std::bind(&RFIDNode::timerCallback, this));
  }

  ~RFIDNode() override
  {
    // If a capture is still in progress, try to abort it.
    if (capturing_) {
      CAENRFIDErrorCodes ec = CAENRFID_InventoryAbort(&reader_);
      if (ec != CAENRFID_StatusOK) {
        RCLCPP_WARN(this->get_logger(), "InventoryAbort returned error code: %d", ec);
      } else {
        RCLCPP_INFO(this->get_logger(), "Continuous inventory aborted successfully.");
      }
    }
    // Disconnect the reader.
    CAENRFIDErrorCodes ec = CAENRFID_Disconnect(&reader_);
    if (ec != CAENRFID_StatusOK) {
      RCLCPP_ERROR(this->get_logger(), "Error disconnecting RFID reader: %d", ec);
    } else {
      RCLCPP_INFO(this->get_logger(), "RFID reader disconnected successfully.");
    }
  }

private:
  // Trigger callback: start a capture window and begin inventory.
  void triggerCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
  {
    // If a capture is already in progress, ignore the new trigger.
    if (capturing_) {
      RCLCPP_WARN(this->get_logger(), "Capture already in progress, ignoring trigger.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Trigger received: starting capture window.");
    capturing_ = true;
    capture_start_ = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(snap_mutex_);
      snap_data_.clear();
    }
    // Start continuous inventory only when triggered.
    // Use flags for RSSI, continuous, and framed inventory.
    uint16_t flags = RSSI | CONTINUOS | FRAMED;
    CAENRFIDErrorCodes ec = CAENRFID_InventoryTag(&reader_, (char*)"Source_0", 0, 0, 0, NULL, 0, flags, NULL, NULL);
    if (ec != CAENRFID_StatusOK) {
      RCLCPP_ERROR(this->get_logger(), "Error starting continuous inventory: %d", ec);
    } else {
      RCLCPP_INFO(this->get_logger(), "Continuous inventory started after trigger.");
    }
  }

  // Timer callback: poll for framed tags while capture is active.
  void timerCallback()
  {
    if (capturing_) {
      CAENRFIDTag tag;
      memset(&tag, 0, sizeof(tag));
      bool has_tag = false;
      bool has_result_code = false;
      CAENRFIDErrorCodes ec = CAENRFID_GetFramedTag(&reader_, &has_tag, &tag, &has_result_code);
      if (has_tag && ec == CAENRFID_StatusOK) {
        TagReading reading;
        reading.rssi = tag.RSSI;
        reading.epc = convertTagIDtoHex(tag);
        {
          std::lock_guard<std::mutex> lock(snap_mutex_);
          snap_data_.push_back(reading);
        }
      }
      // Check if the capture window has elapsed.
      auto now = std::chrono::steady_clock::now();
      auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - capture_start_).count();
      if (elapsed_ms >= CAPTURE_WINDOW_MS) {
        // Stop continuous inventory.
        CAENRFIDErrorCodes ec = CAENRFID_InventoryAbort(&reader_);
        if (ec != CAENRFID_StatusOK) {
          RCLCPP_WARN(this->get_logger(), "InventoryAbort returned error code: %d", ec);
        } else {
          RCLCPP_INFO(this->get_logger(), "Continuous inventory aborted after capture window.");
        }
        processCaptureWindow();
        capturing_ = false;
      }
    }
  }

  // Convert tag ID to a hex string.
  std::string convertTagIDtoHex(const CAENRFIDTag &tag)
  {
    char buf[2 * MAX_ID_LENGTH + 1] = {0};
    for (int i = 0; i < tag.Length; i++) {
      sprintf(&buf[i * 2], "%02X", tag.ID[i]);
    }
    return std::string(buf);
  }

  // Process the data captured during the window.
  void processCaptureWindow()
  {
    std::lock_guard<std::mutex> lock(snap_mutex_);
    if (snap_data_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No tags captured in the window.");
      return;
    }
    int best_index = 0;
    int16_t best_rssi = snap_data_[0].rssi;
    for (size_t i = 1; i < snap_data_.size(); ++i) {
      if (snap_data_[i].rssi > best_rssi) {
        best_rssi = snap_data_[i].rssi;
        best_index = i;
      }
    }
    auto message = std_msgs::msg::String();
    message.data = snap_data_[best_index].epc;
    best_tag_pub_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Best tag: EPC=%s, RSSI=%d",
                message.data.c_str(), best_rssi);
  }

  // ROS publishers, subscribers, and timer.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr best_tag_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // CAEN RFID reader and RS232 parameters.
  CAENRFIDReader reader_;
  RS232_params port_params_;

  // Capture window variables.
  std::atomic<bool> capturing_;
  std::chrono::steady_clock::time_point capture_start_;
  std::vector<TagReading> snap_data_;
  std::mutex snap_mutex_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RFIDNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
