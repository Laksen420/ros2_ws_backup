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
#include <map>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"

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
    
    // Publisher for the number of pallets detected
    pallet_count_pub_ = this->create_publisher<std_msgs::msg::Int32>("rfid_pallet_count", 10);

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
  // K-means clustering to separate readings into antenna groups
  std::vector<std::vector<int16_t>> clusterRssiValues(const std::vector<int16_t>& rssi_values) {
    if (rssi_values.size() < 2) {
        return {rssi_values};
    }
    
    // Initialize with min and max as starting centroids
    int16_t min_rssi = *std::min_element(rssi_values.begin(), rssi_values.end());
    int16_t max_rssi = *std::max_element(rssi_values.begin(), rssi_values.end());
    
    // If the range is small, just return a single cluster
    if (max_rssi - min_rssi < 100) {
        return {rssi_values};
    }
    
    // Simple k-means with k=2 for antenna separation
    std::vector<int16_t> centroids = {min_rssi, max_rssi};
    std::vector<std::vector<int16_t>> clusters(2);
    
    // Single iteration is usually enough for this binary case
    for (int16_t rssi : rssi_values) {
        int cluster_idx = (std::abs(rssi - centroids[0]) < std::abs(rssi - centroids[1])) ? 0 : 1;
        clusters[cluster_idx].push_back(rssi);
    }
    
    // Remove empty clusters
    clusters.erase(std::remove_if(clusters.begin(), clusters.end(),
                                  [](const std::vector<int16_t>& c) { return c.empty(); }),
                  clusters.end());
    
    return clusters;
  }

  // Remove outliers using IQR method
  std::vector<int16_t> removeOutliers(const std::vector<int16_t>& values) {
    if (values.size() < 4) {
        return values; // Too few values to determine outliers
    }
    
    // Create a sorted copy
    std::vector<int16_t> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    
    // Calculate Q1 and Q3
    size_t q1_idx = sorted.size() / 4;
    size_t q3_idx = sorted.size() * 3 / 4;
    int16_t q1 = sorted[q1_idx];
    int16_t q3 = sorted[q3_idx];
    
    // Calculate IQR and bounds
    int16_t iqr = q3 - q1;
    int16_t lower_bound = q1 - 1.5 * iqr;
    int16_t upper_bound = q3 + 1.5 * iqr;
    
    // Filter outliers
    std::vector<int16_t> filtered;
    for (int16_t val : values) {
        if (val >= lower_bound && val <= upper_bound) {
            filtered.push_back(val);
        }
    }
    
    return filtered;
  }

  // Try to connect to the reader on multiple possible ports
  bool connectReader() {
    RCLCPP_INFO(this->get_logger(), "Starting RFID reader connection...");
    
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
        RCLCPP_INFO(this->get_logger(), "Port %s doesn't exist, skipping", port);
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

        // Add both antennas as read points to Source_0
        ec = CAENRFID_AddReadPoint(&reader_, (char*)"Source_0", (char*)"Ant0");
        if (ec != CAENRFID_StatusOK) {
          RCLCPP_WARN(this->get_logger(), "AddReadPoint for Ant0 returned %d (may be okay)", ec);
        }

        ec = CAENRFID_AddReadPoint(&reader_, (char*)"Source_0", (char*)"Ant1");
        if (ec != CAENRFID_StatusOK) {
          RCLCPP_WARN(this->get_logger(), "AddReadPoint for Ant1 returned %d (may be okay)", ec);
        }

        // Check status of both antennas
        CAENRFIDReadPointStatus status;
        ec = CAENRFID_GetReadPointStatus(&reader_, (char*)"Ant0", &status);
        if (ec == CAENRFID_StatusOK) {
          RCLCPP_INFO(this->get_logger(), "Ant0 status: %d", status);
        } else {
          RCLCPP_WARN(this->get_logger(), "Could not get Ant0 status: %d", ec);
        }

        ec = CAENRFID_GetReadPointStatus(&reader_, (char*)"Ant1", &status);
        if (ec == CAENRFID_StatusOK) {
          RCLCPP_INFO(this->get_logger(), "Ant1 status: %d", status);
        } else {
          RCLCPP_WARN(this->get_logger(), "Could not get Ant1 status: %d", ec);
        }

        // Set Q value to 2 since we'll only ever detect 0-2 tags
        ec = CAENRFID_SetSourceConfiguration(&reader_, (char*)"Source_0", CONFIG_G2_Q_VALUE, 2);
        if (ec != CAENRFID_StatusOK) {
          RCLCPP_WARN(this->get_logger(), "Warning: can't set Q value: %d", ec);
        } else {
          RCLCPP_INFO(this->get_logger(), "Q value set to 2");
        }


        // Set protocol
        ec = CAENRFID_SetProtocol(&reader_, CAENRFID_EPC_C1G2);
        if (ec != CAENRFID_StatusOK) {
          RCLCPP_ERROR(this->get_logger(), "Error setting protocol: %d", ec);
          return false;
        }

        break;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to RFID reader at %s, error: %d", port, ec);
      }
    }

    if (!connected) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to RFID reader on any port");
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
        
        // Print every tag read in real-time
        RCLCPP_INFO(this->get_logger(), "Tag detected: EPC=%s, RSSI=%d", 
                  reading.epc.c_str(), reading.rssi);
        
        {
          std::lock_guard<std::mutex> lock(snap_mutex_);
          snap_data_.push_back(reading);
        }
      } else if (ec != CAENRFID_StatusOK && ec != -1) {
        // Error -1 is just no data available, so don't log that
        RCLCPP_WARN(this->get_logger(), "Error getting framed tag: %d", ec);
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
    RCLCPP_INFO(this->get_logger(), "------ Processing Capture Window ------");
    RCLCPP_INFO(this->get_logger(), "Total readings captured: %ld", snap_data_.size());
    
    if (snap_data_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No tags captured in the window.");
      
      // Publish zero pallets detected
      auto count_msg = std_msgs::msg::Int32();
      count_msg.data = 0;
      pallet_count_pub_->publish(count_msg);
      return;
    }
    
    // Group readings by EPC
    std::map<std::string, std::vector<int16_t>> tag_rssi_groups;
    for (const auto& reading : snap_data_) {
      tag_rssi_groups[reading.epc].push_back(reading.rssi);
    }
    
    // Print summary of each unique tag
    RCLCPP_INFO(this->get_logger(), "Unique tags detected: %ld", tag_rssi_groups.size());
    
    // Score and rank tags to find pallets
    std::vector<std::pair<std::string, float>> tag_scores;
    
    // Use a more appropriate threshold based on observed values
    const int MIN_RSSI_THRESHOLD = -700; // Conservative threshold for detecting close tags
    const int MIN_READINGS = 3; // Minimum number of readings to consider stable
    const int MAX_RSSI_RANGE = 70; // Maximum allowed range between min and max RSSI
    
    for (const auto& [epc, rssi_values] : tag_rssi_groups) {
      // Skip if too few readings
      if (rssi_values.size() < MIN_READINGS) {
        RCLCPP_INFO(this->get_logger(), "Tag %s has too few readings (%ld), skipping", 
                   epc.c_str(), rssi_values.size());
        continue;
      }
      
      // Step 1: Remove outliers using IQR method
      std::vector<int16_t> filtered_values = removeOutliers(rssi_values);
      RCLCPP_INFO(this->get_logger(), "Tag %s: Original readings=%ld, After outlier removal=%ld", 
                 epc.c_str(), rssi_values.size(), filtered_values.size());
      
      // Step 2: Cluster the filtered values to separate antenna readings
      std::vector<std::vector<int16_t>> antenna_clusters = clusterRssiValues(filtered_values);
      
      // Log the clusters
      bool valid_detection = false;
      for (size_t i = 0; i < antenna_clusters.size(); i++) {
        // Calculate average RSSI
        float cluster_avg = 0;
        for (int16_t val : antenna_clusters[i]) {
          cluster_avg += val;
        }
        cluster_avg /= antenna_clusters[i].size();
        
        // Calculate min-max range instead of variance
        int16_t min_rssi = *std::min_element(antenna_clusters[i].begin(), antenna_clusters[i].end());
        int16_t max_rssi = *std::max_element(antenna_clusters[i].begin(), antenna_clusters[i].end());
        int16_t rssi_range = max_rssi - min_rssi;
        
        RCLCPP_INFO(this->get_logger(), "Tag %s - Antenna cluster %ld: Count=%ld, Avg RSSI=%.1f, Range=%d (Min=%d, Max=%d)",
                   epc.c_str(), i, antenna_clusters[i].size(), cluster_avg, rssi_range, min_rssi, max_rssi);
        
        // Check if this cluster meets our criteria for a valid pallet detection
        if (antenna_clusters[i].size() >= MIN_READINGS && 
            cluster_avg > MIN_RSSI_THRESHOLD && 
            rssi_range <= MAX_RSSI_RANGE) {
          
          // Calculate a score for this tag based on signal strength and stability
          float signal_strength_score = (cluster_avg + 650.0) / 350.0; // Normalize to 0-1 range
          float stability_score = 1.0 - (static_cast<float>(rssi_range) / MAX_RSSI_RANGE);
          float size_score = std::min(1.0f, antenna_clusters[i].size() / 10.0f); // Bonus for more readings
          
          // Combined score gives preference to strong, stable signals
          float combined_score = signal_strength_score * 0.6 + stability_score * 0.3 + size_score * 0.1;
          
          tag_scores.push_back({epc, combined_score});
          RCLCPP_INFO(this->get_logger(), "Scored tag: EPC=%s, Score=%.2f, Antenna=%ld, Avg RSSI=%.1f",
                     epc.c_str(), combined_score, i, cluster_avg);
          
          valid_detection = true;
          break; // One valid cluster is enough to count this tag
        }
      }
      
      if (!valid_detection) {
        RCLCPP_INFO(this->get_logger(), "Tag %s doesn't meet criteria in any antenna cluster, skipping", 
                   epc.c_str());
      }
    }
    
    // Sort tags by score (highest first)
    std::sort(tag_scores.begin(), tag_scores.end(), 
              [](const auto& a, const auto& b) { return a.second > b.second; });
    
    // Take the top 2 tags at most (since we can have at most 2 pallets)
    std::vector<std::string> detected_pallets;
    for (size_t i = 0; i < std::min(size_t(2), tag_scores.size()); i++) {
      detected_pallets.push_back(tag_scores[i].first);
      RCLCPP_INFO(this->get_logger(), "Selected pallet %ld: %s with score %.2f", 
                 i+1, tag_scores[i].first.c_str(), tag_scores[i].second);
    }
    
    int pallet_count = detected_pallets.size();
    RCLCPP_INFO(this->get_logger(), "Total pallets detected: %d", pallet_count);
    
    // Publish pallet count
    auto count_msg = std_msgs::msg::Int32();
    count_msg.data = pallet_count;
    pallet_count_pub_->publish(count_msg);
    
    // Publish tag IDs for each detected pallet
    if (!detected_pallets.empty()) {
      // Publish the highest scoring tag
      auto message = std_msgs::msg::String();
      message.data = detected_pallets[0]; // Top scoring tag
      best_tag_pub_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Published best tag: %s", detected_pallets[0].c_str());
    }
  }

  // ROS publishers, subscribers, and timer.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr best_tag_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pallet_count_pub_;
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
