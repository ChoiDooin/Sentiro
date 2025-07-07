#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

class TestNode : public rclcpp::Node
{
public:
    TestNode() : Node("test_node"), start_time(this->now()), now_time(this->now()), elapsed_time(0, 0)
    {
        // 시리얼 포트 설정
        try
        {
            serial_port_1.setPort("/dev/ttyACM0"); // 뢰전 CD, 리니어 연결 아두이노
            serial_port_1.setBaudrate(9600);

            serial_port_2.setPort("/dev/ttyACM1"); // 플랩 DC 연결 아두이노
            serial_port_2.setBaudrate(9600);

            serial::Timeout to = serial::Timeout::simpleTimeout(1000);

            serial_port_1.setTimeout(to);
            serial_port_1.open();

            serial_port_2.setTimeout(to);
            serial_port_2.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open port: %s", e.what());
        }

        if (serial_port_1.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial port 1 initialized");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port 1");
        }

        if (serial_port_2.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial port 2 initialized");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port 2");
        }
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TestNode::timerCallback, this));
    }

private:
    // uint8_t serial_step = '0';
    char serial_step1 = '0';
    char serial_step2 = '0';
    bool sent_flag = false;

    std::string received1;
    std::string received2;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial serial_port_1; // 뢰전 DC, 리니어 연결 아두이노
    serial::Serial serial_port_2; // 플랩 DC 연결 아두이노

    rclcpp::Time start_time = this->now();
    rclcpp::Time now_time;
    rclcpp::Duration elapsed_time;

    void timerCallback()
    {
        now_time = this->now();
        elapsed_time = now_time - start_time;

        if (elapsed_time.seconds() > 10.0)
        {
            serial_step1 = '1';
            serial_step2 = '9';
            serial_port_1.write(reinterpret_cast<const uint8_t *>(&serial_step1), 1);
            serial_port_2.write(reinterpret_cast<const uint8_t *>(&serial_step2), 1);
        }
        // RCLCPP_INFO(this->get_logger(), "Sent '%c' to port 1", serial_step1);
        received1 = serial_port_1.readline(100, "\n");
        RCLCPP_INFO(this->get_logger(), "Received '%s' to port 1", received1.c_str());

        // RCLCPP_INFO(this->get_logger(), "Sent '%c' to port 2", serial_step2);
        received2 = serial_port_2.readline(100, "\n");
        RCLCPP_INFO(this->get_logger(), "Received '%s' to port 2", received2.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
};
