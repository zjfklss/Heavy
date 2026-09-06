#include "serial_test/serial_test.hpp"
#include "serial_test/crc.hpp"
#include "serial_test/packet.hpp"

#include <functional>
#include <chrono>
#include <utility>

namespace serial_test
{

SerialTest::SerialTest(const rclcpp::NodeOptions& options)
    : Node("serial_test", options)
    , owned_ctx_(std::make_unique<drivers::common::IoContext>(2))
    , serial_driver_(std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_))
{
    RCLCPP_INFO(get_logger(), "Starting SerialTest...");
    
    debug_ = this->declare_parameter("debug", false);
    getParams();
    
    from_client_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/from_custom_client", 10);
    
    to_client_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/to_custom_client", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            send0311Packet(reinterpret_cast<const uint8_t*>(msg->data.c_str()), msg->data.size());
        });
    
    stats_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { printFrequency(); });
    
    try
    {
        serial_driver_->init_port(device_name_, *device_config_);
        if (!serial_driver_->port()->is_open())
        {
            serial_driver_->port()->open();
            receive_thread_ = std::thread(&SerialTest::receiveData, this);
            RCLCPP_INFO(get_logger(), "Serial port opened: %s", device_name_.c_str());
        }
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR(get_logger(), "Error: %s - %s", device_name_.c_str(), ex.what());
        throw ex;
    }
}

SerialTest::~SerialTest()
{
    if (receive_thread_.joinable())
        receive_thread_.join();
    if (serial_driver_->port()->is_open())
        serial_driver_->port()->close();
}

void SerialTest::getParams()
{
    using FlowControl = drivers::serial_driver::FlowControl;
    using Parity = drivers::serial_driver::Parity;
    using StopBits = drivers::serial_driver::StopBits;
    
    try
    {
        device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
        int baud_rate = declare_parameter<int>("baud_rate", 921600);
        
        device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
            baud_rate, FlowControl::NONE, Parity::NONE, StopBits::ONE);
        
        RCLCPP_INFO(get_logger(), "Serial: %s, baud=%d", device_name_.c_str(), baud_rate);
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR(get_logger(), "Parameter error: %s", ex.what());
        throw;
    }
}

void SerialTest::receiveData()
{
    std::vector<uint8_t> sof(1);
    std::vector<uint8_t> header_tail(4);
    
    while (rclcpp::ok())
    {
        try
        {
            serial_driver_->port()->receive(sof);
            
            if (sof[0] == 0xA5)
            {
                serial_driver_->port()->receive(header_tail);

                FrameHeader frame_header;
                frame_header.SOF = sof[0];
                frame_header.data_length = static_cast<uint16_t>(header_tail[0]) |
                    (static_cast<uint16_t>(header_tail[1]) << 8);
                frame_header.seq = header_tail[2];
                frame_header.crc8 = header_tail[3];

                uint8_t header_data[4] = {
                    frame_header.SOF,
                    static_cast<uint8_t>(frame_header.data_length & 0xFF),
                    static_cast<uint8_t>(frame_header.data_length >> 8),
                    frame_header.seq,
                };

                if (crc8::Get_CRC8_Check_Sum(header_data, 4, crc8::CRC8_INIT) !=
                    frame_header.crc8)
                {
                    continue;
                }

                if (frame_header.data_length > 300)
                {
                    RCLCPP_WARN(get_logger(), "Drop frame with invalid data_length=%u",
                                frame_header.data_length);
                    continue;
                }

                std::vector<uint8_t> payload(2 + frame_header.data_length + 2);
                serial_driver_->port()->receive(payload);

                std::vector<uint8_t> frame;
                frame.reserve(5 + payload.size());
                frame.push_back(frame_header.SOF);
                frame.insert(frame.end(), header_tail.begin(), header_tail.end());
                frame.insert(frame.end(), payload.begin(), payload.end());

                if (!crc16::Verify_CRC16_Check_Sum(frame.data(), frame.size()))
                {
                    continue;
                }

                uint16_t cmd_id = static_cast<uint16_t>(payload[0]) |
                    (static_cast<uint16_t>(payload[1]) << 8);

                if (cmd_id == 0x0311)
                {
                    std_msgs::msg::String msg;
                    msg.data = std::string(
                        reinterpret_cast<char*>(payload.data() + 2),
                        frame_header.data_length);
                    from_client_pub_->publish(msg);
                    if (from_client_callback_)
                    {
                        from_client_callback_(msg.data);
                    }
                    
                    if (debug_)
                    {
                        RCLCPP_INFO(get_logger(), "Received from client (0x0311), seq=%d, size=%u", 
                                    frame_header.seq, frame_header.data_length);
                    }
                }
            }
        }
        catch (const std::exception& ex)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20, "Receive error: %s", ex.what());
            reopenPort();
        }
    }
}

void SerialTest::send0310Packet(const uint8_t* data, size_t len)
{
    if (len > 300)
    {
        RCLCPP_ERROR(get_logger(), "Data too long for 0x0310! Max 300, got %zu", len);
        return;
    }
    
    std::lock_guard<std::mutex> lock(send_mutex_);
    
    try
    {
        SendPacket0310 packet;
        packet.header.SOF = 0xA5;
        packet.header.data_length = sizeof(RobotCustomData);
        packet.header.seq = packet_seq_++;
        
        uint8_t header_data[4] = {packet.header.SOF, 
                                  (uint8_t)(packet.header.data_length & 0xFF),
                                  (uint8_t)(packet.header.data_length >> 8),
                                  packet.header.seq};
        packet.header.crc8 = crc8::Get_CRC8_Check_Sum(header_data, 4, crc8::CRC8_INIT);
        packet.cmd_id = 0x0310;
        memset(packet.data.data, 0, 300);
        memcpy(packet.data.data, data, len);
        crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
        
        serial_driver_->port()->send(toVector(packet));
        
        send_count_++;
        
        if (debug_)
        {
            RCLCPP_INFO(get_logger(), "Sent (0x0310), seq=%d, size=%zu", 
                        packet.header.seq, len);
        }
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR(get_logger(), "Send 0x0310 error: %s", ex.what());
        reopenPort();
    }
}

void SerialTest::send0311Packet(const uint8_t* data, size_t len)
{
    if (len > 30)
    {
        RCLCPP_ERROR(get_logger(), "Data too long for 0x0311! Max 30, got %zu", len);
        return;
    }
    
    std::lock_guard<std::mutex> lock(send_mutex_);
    
    try
    {
        SendPacket0311 packet;
        packet.header.SOF = 0xA5;
        packet.header.data_length = sizeof(CustomClientCommand);
        packet.header.seq = packet_seq_++;
        
        uint8_t header_data[4] = {packet.header.SOF, 
                                  (uint8_t)(packet.header.data_length & 0xFF),
                                  (uint8_t)(packet.header.data_length >> 8),
                                  packet.header.seq};
        packet.header.crc8 = crc8::Get_CRC8_Check_Sum(header_data, 4, crc8::CRC8_INIT);
        packet.cmd_id = 0x0311;
        memset(packet.data.data, 0, 30);
        memcpy(packet.data.data, data, len);
        crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
        
        serial_driver_->port()->send(toVector(packet));
        
        if (debug_)
        {
            RCLCPP_INFO(get_logger(), "Sent (0x0311), seq=%d, size=%zu", packet.header.seq, len);
        }
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR(get_logger(), "Send 0x0311 error: %s", ex.what());
        reopenPort();
    }
}

void SerialTest::sendToCustomClient(const uint8_t* data, size_t len)
{
    send0310Packet(data, len);
}

void SerialTest::sendToCustomClient(const std::string& data)
{
    send0310Packet(reinterpret_cast<const uint8_t*>(data.c_str()), data.size());
}

void SerialTest::setFromClientCallback(std::function<void(const std::string&)> callback)
{
    from_client_callback_ = std::move(callback);
}

void SerialTest::printFrequency()
{
    int current_count = send_count_;
    int hz = current_count - last_count_;
    RCLCPP_INFO(get_logger(), "0x0310 Send Frequency: %d Hz", hz);
    last_count_ = current_count;
}

void SerialTest::reopenPort()
{
    RCLCPP_WARN(get_logger(), "Reopening port...");
    try
    {
        if (serial_driver_->port()->is_open())
            serial_driver_->port()->close();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        serial_driver_->port()->open();
        RCLCPP_INFO(get_logger(), "Port reopened");
    }
    catch (const std::exception& ex)
    {
        RCLCPP_ERROR(get_logger(), "Reopen error: %s", ex.what());
        if (rclcpp::ok())
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            reopenPort();
        }
    }
}

} // namespace serial_test

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serial_test::SerialTest)
