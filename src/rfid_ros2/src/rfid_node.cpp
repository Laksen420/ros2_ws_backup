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
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

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

    // Connect to the reader, trying both possible ports
    connectReader();

    // Set up non-blocking stdin for Enter key detection
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag &= ~ICANON; // Turn off canonical mode
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);

    RCLCPP_INFO(this->get_logger(), "RFID reader ready. Press ENTER to trigger a tag scan.");

    // Start a timer to poll for tag data and check for Enter key
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
    if (reader_connected_) {
      CAENRFIDErrorCodes ec = CAENRFID_Disconnect(&reader_);
      if (ec != CAENRFID_StatusOK) {
        RCLCPP_ERROR(this->get_logger(), "Error disconnecting RFID reader: %d", ec);
      } else {
        RCLCPP_INFO(this->get_logger(), "RFID reader disconnected successfully.");
      }
    }
  }

private:
  // Try to connect to the reader on multiple possible ports
  bool connectReader() {
    // RS232 parameters with correct port
    port_params_.baudrate    = 921600;
    port_params_.dataBits    = 8;
    port_params_.stopBits    = 1;
    port_params_.parity      = 0;
    port_params_.flowControl = 0;

    // Try the symlink first, then fall back to ACM ports
    const char* ports[] = {"/dev/ttyRFID", "/dev/ttyACM0", "/dev/ttyACM1"};
    bool connected = false;
    
    for (const char* port : ports) {
      // Skip ports that don't exist
      if (access(port, F_OK) != 0) {
        RCLCPP_DEBUG(this->get_logger(), "Port %s doesn't exist, skipping", port);
        continue;
      }
      
      port_params_.com = (char*)port;
      RCLCPP_INFO(this->get_logger(), "Trying to connect to RFID reader at %s...", port);
      
      CAENRFIDErrorCodes ec = CAENRFID_Connect(&reader_, CAENRFID_USB, &port_params_);
      if (ec == CAENRFID_StatusOK) {
        RCLCPP_INFO(this->get_logger(), "RFID reader connected at %s", port);
        connected = true;
        reader_connected_ = true;
        
        // Save which port worked
        active_port_ = port;
        
        // Configure reader
        ec = CAENRFID_SetSourceConfiguration(&reader_, (char*)"Source_0", CONFIG_READCYCLE, 0);
        if (ec != CAENRFID_StatusOK) {
          RCLCPP_WARN(this->get_logger(), "Warning: can't set infinite cycles: %d", ec);
        }
        
        ec = CAENRFID_AddReadPoint(&reader_, (char*)"Source_0", (char*)"Ant0");
        if (ec != CAENRFID_StatusOK) {
          RCLCPP_WARN(this->get_logger(), "AddReadPoint returned %d (may be okay)", ec);
        }
        
        ec = CAENRFID_SetProtocol(&reader_, CAENRFID_EPC_C1G2);
        if (ec != CAENRFID_StatusOK) {
          RCLCPP_ERROR(this->get_logger(), "Error setting protocol: %d", ec);
          return false;
        }
        
        break;
      }
    }
    
    return connected;
  }

  bool checkIfEnterPressed() {
    fd_set rfds;
    struct timeval tv = {0, 0}; // Non-blocking check
    
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    
    int ret = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
    if (ret > 0) {
      char c = getchar(); // Read one char
      return (c == '\n'); // Return true if Enter was pressed
    }
    return false; 
  }

  // Try to reconnect if connection is lost
  bool tryReconnect() {
    RCLCPP_INFO(this->get_logger(), "Trying to reconnect...");
    
    // If already connected, disconnect first
    if (reader_connected_) {
      CAENRFID_Disconnect(&reader_);
      reader_connected_ = false;
      std::this_thread::sleep_for(100ms);
    }
    
    return connectReader();
  }

  // Trigger callback: start a capture window and begin inventory.
  void triggerCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
  {
    // If a capture is already in progress, ignore the new trigger.
    if (capturing_) {
      RCLCPP_WARN(this->get_logger(), "Capture already in progress, ignoring trigger.");
      return;
    }
    
    // If reader is not connected or the current port is gone, try to reconnect
    if (!reader_connected_ || (access(active_port_.c_str(), F_OK) != 0)) {
      if (!tryReconnect()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reconnect, cannot trigger.");
        return;
      }
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
      capturing_ = false;
      
      // If connection error, try to reconnect
      if (ec == -2 || ec == -7) {
        RCLCPP_INFO(this->get_logger(), "Trying to reconnect due to error %d.", ec);
        if (tryReconnect()) {
          RCLCPP_INFO(this->get_logger(), "Reconnected. Try triggering again.");
        }
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Continuous inventory started after trigger.");
    }
  }

  // Timer callback: poll for framed tags while capture is active.
  void timerCallback()
  {
    // First check if Enter was pressed when not already capturing
    if (!capturing_ && checkIfEnterPressed()) {
      RCLCPP_INFO(this->get_logger(), "ENTER pressed: triggering tag scan.");
      // Create an empty message to trigger the scan
      auto empty_msg = std::make_shared<std_msgs::msg::Empty>();
      // Call the trigger callback directly
      triggerCallback(empty_msg);
    }
    
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
          if (ec == -7) {
            RCLCPP_INFO(this->get_logger(), "Ignoring error -7 at end of capture window, likely a reset condition.");
          }
        } else {
          RCLCPP_INFO(this->get_logger(), "Continuous inventory aborted successfully.");
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
  std::atomic<bool> capturing_{false};
  std::atomic<bool> reader_connected_{false};
  std::string active_port_;
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
