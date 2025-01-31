#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "CAENRFIDLib_Light.h"
#include "host.h"

#include <chrono>
#include <string>
#include <cstdio>
#include <cstring>

using namespace std::chrono_literals;

class RFIDNode : public rclcpp::Node
{
public:
    RFIDNode() : Node("rfid_node")
    {
        RCLCPP_INFO(get_logger(), "RFIDNode starting...");

        // Publisher for the EPC
        rfid_pub_ = create_publisher<std_msgs::msg::String>("rfid_data", 10);

        if (!init_reader()) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize CAENRFID reader");
            rclcpp::shutdown();
            return;
        }

        // Timer to poll for new tags every 200ms
        timer_ = create_wall_timer(200ms, std::bind(&RFIDNode::poll_tags, this));
    }

    ~RFIDNode()
    {
        RCLCPP_INFO(get_logger(), "Shutting down RFIDNode...");
        // Abort inventory
        CAENRFIDErrorCodes ec = CAENRFID_InventoryAbort(&reader_);
        if (ec == CAENRFID_StatusOK) {
            // read leftover frames
            while (true) {
                bool has_tag = false, has_rc = false;
                CAENRFIDTag tag;
                memset(&tag, 0, sizeof(tag));
                ec = CAENRFID_GetFramedTag(&reader_, &has_tag, &tag, &has_rc);
                if (has_rc) {
                    RCLCPP_INFO(get_logger(), "Final code after abort: %d", ec);
                    break;
                }
                if (ec != CAENRFID_StatusOK && !has_tag) {
                    break;
                }
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Error aborting inventory: %d", ec);
        }

        // Disconnect
        ec = CAENRFID_Disconnect(&reader_);
        if (ec == CAENRFID_StatusOK) {
            RCLCPP_INFO(get_logger(), "Disconnected from RFID reader.");
        } else {
            RCLCPP_ERROR(get_logger(), "Error disconnecting: %d", ec);
        }
    }

private:
    bool init_reader()
    {
        // Prepare the CAENRFIDReader struct
        reader_.connect = _connect;
        reader_.disconnect = _disconnect;
        reader_.tx = _tx;
        reader_.rx = _rx;
        reader_.clear_rx_data = _clear_rx_data;
        reader_.enable_irqs = _enable_irqs;
        reader_.disable_irqs = _disable_irqs;

        RS232_params port_params = {
            .com = "/dev/ttyACM1", // or whatever device for your RFID
            .baudrate = 921600,
            .dataBits = 8,
            .stopBits = 1,
            .parity = 0,
            .flowControl = 0
        };

        CAENRFIDErrorCodes ec = CAENRFID_Connect(&reader_, CAENRFID_USB, &port_params);
        if (ec != CAENRFID_StatusOK) {
            RCLCPP_ERROR(get_logger(), "Error connecting: %d", ec);
            return false;
        }
        RCLCPP_INFO(get_logger(), "Connected to CAEN RFID reader.");

        // unlimited read cycles => 0
        ec = CAENRFID_SetSourceConfiguration(&reader_, "Source_0", CONFIG_READCYCLE, 0);
        if (ec != CAENRFID_StatusOK) {
            RCLCPP_WARN(get_logger(), "Warning: can't set infinite cycles: %d", ec);
        }

        // optional: add readpoint
        ec = CAENRFID_AddReadPoint(&reader_, "Source_0", "Ant0");
        if (ec != CAENRFID_StatusOK) {
            RCLCPP_WARN(get_logger(), "AddReadPoint returned %d (maybe ok)", ec);
        }

        // set protocol
        ec = CAENRFID_SetProtocol(&reader_, CAENRFID_EPC_C1G2);
        if (ec != CAENRFID_StatusOK) {
            RCLCPP_ERROR(get_logger(), "Error setting protocol: %d", ec);
            CAENRFID_Disconnect(&reader_);
            return false;
        }

        // Start continuous + framed inventory with RSSI
        uint16_t flags = RSSI | CONTINUOS | FRAMED; // 0x07
        ec = CAENRFID_InventoryTag(&reader_,
                                  "Source_0",
                                  0, 0, 0, NULL, 0,
                                  flags,
                                  NULL,
                                  NULL);
        if (ec != CAENRFID_StatusOK) {
            RCLCPP_ERROR(get_logger(), "Error starting continuous inventory: %d", ec);
            CAENRFID_Disconnect(&reader_);
            return false;
        }
        RCLCPP_INFO(get_logger(), "Continuous inventory started (RSSI enabled).");
        return true;
    }

    void poll_tags()
    {
        bool has_tag = false;
        bool has_rc = false;
        CAENRFIDTag tag;
        memset(&tag, 0, sizeof(tag));

        CAENRFIDErrorCodes ec = CAENRFID_GetFramedTag(&reader_, &has_tag, &tag, &has_rc);
        if (has_rc) {
            // inventory ended
            RCLCPP_WARN(get_logger(), "Inventory ended, code: %d", ec);
            return;
        }
        if (ec != CAENRFID_StatusOK && !has_tag) {
            // no new tag, minor read error, or no data
            return;
        }

        if (has_tag) {
            // convert ID to hex
            std::string epc_hex;
            char buffer[3];
            for (int i = 0; i < tag.Length; i++) {
                sprintf(buffer, "%02X", tag.ID[i]);
                epc_hex += buffer;
            }
            // Publish on /rfid_data
            auto msg = std_msgs::msg::String();
            msg.data = epc_hex;
            rfid_pub_->publish(msg);

            RCLCPP_INFO(get_logger(), "Published RFID EPC=%s, RSSI=%d", epc_hex.c_str(), tag.RSSI);
        }
    }

    CAENRFIDReader reader_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rfid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RFIDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
