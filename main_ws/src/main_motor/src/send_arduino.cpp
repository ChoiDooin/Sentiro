#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <serial/serial.h>

class MotorCommandNode : public rclcpp::Node
{
public:
    MotorCommandNode()
        : Node("motor_command_node")
    {
        // 시리얼 포트 설정
        try
        {
            serial_port_.setPort("/dev/ttyACM0"); // 아두이노 포트에 맞게 수정
            serial_port_.setBaudrate(9600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(to);
            serial_port_.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
        }

        if (serial_port_.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial port initialized");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
        }

        // 토픽 구독 설정
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "motor_command", 10,
            std::bind(&MotorCommandNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (serial_port_.isOpen())
        {
            char command = (msg->data == 1) ? '1' : '0';
            serial_port_.write(reinterpret_cast<const uint8_t *>(&command), 1);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    serial::Serial serial_port_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
