#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>
#include <thread>
#include <atomic>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Include the CAEN RFID SDK headers in an extern "C" block:
extern "C" {
  #include "CAENRFIDLib_Light.h"
  #include "host.h"
  #include "IO_Light.h"
  #include "Protocol_Light.h"
}

using namespace std::chrono_literals;

// Capture window in milliseconds
#define CAPTURE_WINDOW_MS 1000

// Maximum number of readings to store during the capture window
#define MAX_SNAP_TAGS 200

// Structure to store a tag reading (EPC and its RSSI)
struct TagReading {
  std::string epc;
  int16_t rssi;
};

class RFIDNode : public rclcpp::Node
{
public:
  RFIDNode()
  : Node("rfid_node"), capturing_(false)
  {
    // Create a publisher for the "best tag" message.
    publisher_ = this->create_publisher<std_msgs::msg::String>("rfid_best_tag", 10);

    // Set up the CAEN RFID reader structure with host functions.
    reader_.connect       = _connect;
    reader_.disconnect    = _disconnect;
    reader_.tx            = _tx;
    reader_.rx            = _rx;
    reader_.clear_rx_data = _clear_rx_data;
    reader_.enable_irqs   = _enable_irqs;
    reader_.disable_irqs  = _disable_irqs;

    // RS232 parameters (adjust port if needed).
    port_params_.com = (char*)"/dev/ttyACM0";
    port_params_.baudrate = 921600;
    port_params_.dataBits = 8;
    port_params_.stopBits = 1;
    port_params_.parity = 0;
    port_params_.flowControl = 0;

    // Connect the reader.
    CAENRFIDErrorCodes ec = CAENRFID_Connect(&reader_, CAENRFID_USB, &port_params_);
    if (ec != CAENRFID_StatusOK) {
      RCLCPP_ERROR(this->get_logger(), "Error connecting RFID reader: %d", ec);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "RFID reader connected");

    // Configure continuous inventory:
    ec = CAENRFID_SetSourceConfiguration(&reader_, "Source_0", CONFIG_READCYCLE, 0);
    if (ec != CAENRFID_StatusOK) {
      RCLCPP_WARN(this->get_logger(), "Warning: cannot set infinite cycles: %d", ec);
    }
    ec = CAENRFID_AddReadPoint(&reader_, "Source_0", "Ant0");
    if (ec != CAENRFID_StatusOK) {
      RCLCPP_WARN(this->get_logger(), "AddReadPoint returned %d (may be okay)", ec);
    }
    ec = CAENRFID_SetProtocol(&reader_, CAENRFID_EPC_C1G2);
    if (ec != CAENRFID_StatusOK) {
      RCLCPP_ERROR(this->get_logger(), "Error setting protocol: %d", ec);
      CAENRFID_Disconnect(&reader_);
      return;
    }
    uint16_t flags = RSSI | CONTINUOS | FRAMED;
    ec = CAENRFID_InventoryTag(&reader_, "Source_0", 0,0,0, NULL,0, flags, NULL, NULL);
    if (ec != CAENRFID_StatusOK) {
      RCLCPP_ERROR(this->get_logger(), "Error starting continuous inventory: %d", ec);
      CAENRFID_Disconnect(&reader_);
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Continuous inventory started (RSSI enabled)");

    // Set stdin to nonblocking mode so we can check for ENTER without blocking.
    int fd_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, fd_flags | O_NONBLOCK);

    // Start a thread to monitor user input and do capture windows.
    capture_thread_ = std::thread(&RFIDNode::captureLoop, this);
  }

  ~RFIDNode() override
  {
    stop_capture_ = true;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    CAENRFID_Disconnect(&reader_);
  }

private:
  // Convert a tag's binary ID into a hex string.
  void convertTagIDtoHex(const CAENRFIDTag* tag, std::string &outHex)
  {
    char buffer[3 * MAX_ID_LENGTH] = {0};
    for (int i = 0; i < tag->Length; i++) {
      sprintf(&buffer[i*2], "%02X", tag->ID[i]);
    }
    outHex = std::string(buffer);
  }

  // Check (nonblocking) whether ENTER has been pressed on stdin.
  bool checkIfEnterPressed()
  {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    int ret = select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
    if (ret > 0) {
      char c;
      read(STDIN_FILENO, &c, 1);
      return (c == '\n');
    }
    return false;
  }

  // This thread continuously monitors for ENTER to trigger a capture window.
  void captureLoop()
  {
    while (!stop_capture_) {
      if (!capturing_ && checkIfEnterPressed()) {
        RCLCPP_INFO(this->get_logger(), "Starting 1s capture window...");
        capturing_ = true;
        captureStart_ = std::chrono::steady_clock::now();
        snapData_.clear();
      }

      if (capturing_) {
        // Capture tags during the window.
        CAENRFIDTag tag;
        memset(&tag, 0, sizeof(tag));
        bool has_tag = false;
        bool has_result_code = false;
        CAENRFIDErrorCodes ec = CAENRFID_GetFramedTag(&reader_, &has_tag, &tag, &has_result_code);
        if (has_tag && ec == CAENRFID_StatusOK) {
          TagReading reading;
          convertTagIDtoHex(&tag, reading.epc);
          reading.rssi = tag.RSSI;
          snapData_.push_back(reading);
        }
        // Check if the capture window has elapsed.
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - captureStart_).count();
        if (elapsed_ms >= CAPTURE_WINDOW_MS) {
          if (snapData_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No tags captured in the window.");
          } else {
            // Select the tag with the highest RSSI.
            int bestIndex = 0;
            int16_t bestRSSI = snapData_[0].rssi;
            for (size_t i = 1; i < snapData_.size(); i++) {
              if (snapData_[i].rssi > bestRSSI) {
                bestRSSI = snapData_[i].rssi;
                bestIndex = i;
              }
            }
            // Publish the best tag.
            auto message = std_msgs::msg::String();
            message.data = snapData_[bestIndex].epc;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Best tag: EPC=%s, RSSI=%d", 
                        snapData_[bestIndex].epc.c_str(), bestRSSI);
          }
          capturing_ = false;
        }
      }
      std::this_thread::sleep_for(10ms);
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::thread capture_thread_;
  std::atomic<bool> stop_capture_{false};

  // The CAEN RFID reader and its RS232 parameters.
  CAENRFIDReader reader_;
  RS232_params port_params_;

  // Variables for the capture window.
  std::atomic<bool> capturing_;
  std::chrono::steady_clock::time_point captureStart_;
  std::vector<TagReading> snapData_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RFIDNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
