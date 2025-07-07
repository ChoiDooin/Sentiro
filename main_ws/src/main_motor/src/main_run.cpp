#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <serial/serial.h>

class MainRunNode : public rclcpp::Node
{
public:
    MainRunNode() : Node("main_run_node"), start_time(this->now()), now_time(this->now()), elapsed_time(0, 0)
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

        omo_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/omo_cmd_vel", 10);

        // goal_point_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/goal_point", 10);

        gui_odom_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/gui_odom", 10);
        // Create a subscriber for the odometry data
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MainRunNode::odomCallback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MainRunNode::cmdVelCallback, this, std::placeholders::_1));

        cam_data_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/user_position", 10, std::bind(&MainRunNode::camDataCallback, this, std::placeholders::_1));

        web_data_sub_ = this->create_subscription<std_msgs::msg::Int64>(
            "/web_data", 10, std::bind(&MainRunNode::webDataCallback, this, std::placeholders::_1));
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MainRunNode::timerCallback, this));
    }

private:
    // --- 제어 상수 ---
    static constexpr double DESIRED_DISTANCE = 0.7; // 목표 거리 [m]
    static constexpr double K_LINEAR = 0.2;         // 선속도 이득
    static constexpr double K_ANGULAR = 0.002;      // 각속도 이득
    static constexpr double MAX_LINEAR = 0.3;       // 선속도 제한 [m/s]
    static constexpr double MAX_ANGULAR = 0.3;      // 각속도 제한 [rad/s]

    // --- 변수 ---
    char serial_step = '0';

    double pub_linear_x = 0;
    double pub_angular_z = 0;

    double nav_linear_x = 0;
    double nav_angular_z = 0;
    double start_yaw = 0;

    double goal_x = 0;
    double goal_y = 0;
    double goal_yaw = 0;

    double cam_x = 0;
    double cam_y = 0;
    double cam_distance = 0;

    double x = 0;
    double y = 0;
    double quaternion_z = 0;
    double quaternion_w = 0;
    double yaw = 0;

    int step_ = 0;

    int person_recog = 0; // 사람 인식 경험 여부

    int web_go_info = 0;

    int run_cnt = 0;

    char prev_serial_step = '0'; // 이전 단계 저장
    // 구독 및 발행
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr omo_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr goal_point_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gui_odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cam_data_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr web_data_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial serial_port_1; // 뢰전 CD, 리니어 연결 아두이노
    serial::Serial serial_port_2; // 플랩 DC 연결 아두이노

    rclcpp::Time start_time;       // 시작 시간
    rclcpp::Time now_time;         // 현재 시간
    rclcpp::Duration elapsed_time; // 경과 시간
    bool rotate = false;           // 회전판 상태
    int flap = 0;                  // 플랩 상태
    int linear = 0;                // 리니어 모터 상태

    bool send_flag = true; // 회전판 회전 명령 전송 여부

    bool linear_up_time = false; // 리니어 모터 상승 대기 시간 여부
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // // Check if the message has the expected number of elements
        // if (msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Received zero velocity command");
        //     return;
        // }

        nav_linear_x = msg->linear.x;
        nav_angular_z = msg->angular.z;
    }

    void webDataCallback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        // Check if the message has the expected number of elements
        // if (msg->data.size() < 2)
        // {
        //     RCLCPP_WARN(this->get_logger(), "Received web data message with insufficient data");
        //     return;
        // }
        // web_go_info = 1;

        step_ = 4;
        start_time = this->now(); // 현재 시간을 시작 시간으로 저장
    }

    void camDataCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    { // Extract the camera data
        cam_x = msg->x;
        cam_y = msg->y;
        cam_distance = msg->z * 0.001; // mm to m

        if (cam_distance > 0.0)
        {
            if (cam_distance < DESIRED_DISTANCE)
            {
                step_ = 2; // 일정 거리 이하 접근
            }
            else
            {
                step_ = 1; // 사람 정면 인식
            }
        }
        else // cam_distance가 0 이하인 경우 = 사람 인식 X
        {
            if (person_recog == 1)
            {
                step_ = 3;       // 터치 들어오지 않음
                start_yaw = yaw; // 현재 yaw를 시작 yaw로 저장
                RCLCPP_INFO(this->get_logger(), "Person recognition lost, starting rotation");
            }
            else
            {
                step_ = 0;        // 기본 navigation
                person_recog = 0; //
            }
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 오도메트리 데이터 추출
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        quaternion_z = msg->pose.pose.orientation.z;
        quaternion_w = msg->pose.pose.orientation.w;
        yaw = atan2(2 * (quaternion_z * quaternion_w), 1 - 2 * (quaternion_z * quaternion_z)); //

        // GUI 오돔 pub
        auto gui_odom_msg = std_msgs::msg::Float64MultiArray();
        gui_odom_msg.data.push_back(x);
        gui_odom_msg.data.push_back(y);

        gui_odom_pub_->publish(gui_odom_msg);
    }

    void timerCallback()
    {
        switch (step_)
        {
        case 0: // 기본 navigation
        {
            serial_step = '0';
            pub_linear_x = nav_linear_x;
            pub_angular_z = nav_angular_z;

            // RCLCPP_INFO(this->get_logger(), "Step: %d, Cam distance: %f, Linear vel: %f, Angular vel: %f, person_recog: %d", step_, cam_distance, pub_linear_x, pub_angular_z, person_recog);
            // Publish the mobile robot velocity command
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = pub_linear_x;
            twist_msg.angular.z = pub_angular_z;
            omo_cmd_vel_pub_->publish(twist_msg);

            break;
        }
        case 1: // 사람 정면 인식
        {
            serial_step = '0';
            // 1) 거리 오차 기반 선속도
            double lin = K_LINEAR * (cam_distance - DESIRED_DISTANCE);
            // 2) 화면 중심 오프셋 기반 각속도
            double ang = -K_ANGULAR * (cam_x - 320);

            // 속도 제한
            pub_linear_x = std::clamp(lin, -MAX_LINEAR, MAX_LINEAR);
            pub_angular_z = std::clamp(ang, -MAX_ANGULAR, MAX_ANGULAR);

            // 사람 인식 경험 여부
            person_recog = 1;

            // RCLCPP_INFO(this->get_logger(), "Step: %d, Cam distance: %f, Linear vel: %f, Angular vel: %f, person_recog: %d", step_, cam_distance, pub_linear_x, pub_angular_z, person_recog);
            // Publish the mobile robot velocity command
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = pub_linear_x;
            twist_msg.angular.z = pub_angular_z;
            omo_cmd_vel_pub_->publish(twist_msg);

            break;
        }
        case 2: // 일정 거리 이하 접근
        {
            serial_step = '0';
            pub_linear_x = 0;
            pub_angular_z = 0;
            person_recog = 1;
            //  RCLCPP_INFO(this->get_logger(), "Step: %d, Cam distance: %f, Linear vel: %f, Angular vel: %f, person_recog: %d", step_, cam_distance, pub_linear_x, pub_angular_z, person_recog);
            // Publish the mobile robot velocity command
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = pub_linear_x;
            twist_msg.angular.z = pub_angular_z;
            omo_cmd_vel_pub_->publish(twist_msg);

            break;
        }
        case 3: // 터치 들어오지 않음
        {
            serial_step = '0';
            pub_linear_x = 0;
            if (yaw < start_yaw - 3.14) // -180도 회전
            {
                pub_angular_z = -0.5;
            }

            else
            {
                pub_angular_z = 0;
                person_recog = 0; // 사람 인식 경험 여부 리셋
            }
            // RCLCPP_INFO(this->get_logger(), "Step: %d, Cam distance: %f, Linear vel: %f, Angular vel: %f, person_recog: %d", step_, cam_distance, pub_linear_x, pub_angular_z, person_recog);
            // Publish the mobile robot velocity command
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = pub_linear_x;
            twist_msg.angular.z = pub_angular_z;
            omo_cmd_vel_pub_->publish(twist_msg);

            break;
        }

        case 4: // 플랩 및 회전판 회전
        {
            // // Publish the elevator command
            // if (serial_port_1.isOpen() && serial_port_2.isOpen())
            // {
            if (serial_step != prev_serial_step) // 이전 단계와 다를 때만 전송
            {
                serial_port_1.write(reinterpret_cast<const uint8_t *>(&serial_step), 1);
                serial_port_2.write(reinterpret_cast<const uint8_t *>(&serial_step), 1);
                rclcpp::sleep_for(std::chrono::milliseconds(50)); // 너무 빠른 연속 전송 방지

                RCLCPP_INFO(this->get_logger(), "Serial step send complete");
                prev_serial_step = serial_step; // 이전 단계 저장
            }

            if (serial_port_1.available() > 0 && serial_port_2.available() > 0) // 시리얼 포트에서 데이터가 수신되었는지 확인
            {
                std::string received1 = serial_port_1.readline(100, "\n");
                std::string received2 = serial_port_2.readline(100, "\n");
                RCLCPP_INFO(this->get_logger(), "Serial 1 입력: %s", received1.c_str());
                RCLCPP_INFO(this->get_logger(), "Serial 2 입력: %s", received2.c_str());

                if (received1.find("Rotate Done") != std::string::npos) // 회전판 연결 아두이노에서 "Rotate done" 메시지를 받으면
                {
                    rotate = true;
                }
                if (received2.find("Flap Down") != std::string::npos) // 플랩 DC 연결 아두이노에서 "Flap down" 메시지를 받으면
                {
                    flap = 1; // 플랩 하강 완료
                }
                if (received1.find("Linear Down") != std::string::npos) // 플랩 DC 연결 아두이노에서 "Flap down" 메시지를 받으면
                {
                    linear = 1;
                }
                if (received2.find("Flap Up") != std::string::npos) // 플랩 DC 연결 아두이노에서 "Flap down" 메시지를 받으면
                {
                    flap = 2;
                }
                if (received1.find("Linear Up") != std::string::npos) // 플랩 DC 연결 아두이노에서 "Flap down" 메시지를 받으면
                {
                    linear = 2;
                }
            }
            // }

            if (send_flag == true)
            {
                now_time = this->now();
                elapsed_time = now_time - start_time; // 현재 시간과 시작 시간의 차이 계산
                if (elapsed_time.seconds() > 3.0)     // 5초 대기
                {
                    serial_step = '1'; // 회전판 회전
                    RCLCPP_INFO(this->get_logger(), "serial step = %c", serial_step);
                    send_flag = false;
                }
            }

            if (flap == 1 && rotate == true)
            {
                serial_step = '2'; // 리니어 하강
                linear_up_time = true;
                start_time = this->now(); // 현재 시간을 시작 시간으로 저장
                flap = 0;          // 플랩 상태 초기화
                rotate = false;    // 회전판 상태 초기화
            }
            //if (linear == 1)
            //{
            //    linear_up_time = true;
            //    start_time = this->now(); // 현재 시간을 시작 시간으로 저장
            //    linear = 0;               // 리니어 상태 초기화
            //}
            if (linear_up_time == true)
            {
                now_time = this->now();
                elapsed_time = now_time - start_time; // 현재 시간과 시작 시간의 차이 계산
                if (elapsed_time.seconds() > 60.0)     // 5초 대기
                {
                    serial_step = '3'; // 리니어 올리기
                    linear = 2;
                    start_time = this->now(); // 현재 시간을 시작 시간으로 저장
                    linear_up_time = false;
                }
            }
            if (linear == 2)
            {
            
                serial_step = '4'; // 플랩 상승
                now_time = this->now();
                elapsed_time = now_time - start_time; // 현재 시간과 시작 시간의 차이 계산
                if (elapsed_time.seconds() > 30.0)     // 5초 대기
                {
                    serial_step = '4'; // flap 올리기
                    linear = 0;
                }
            }
            if (flap == 2)
            {
                step_ = 0; // 기본 navigation으로 복귀
                flap = 0;  // 플랩 상태 초기화
            }

            pub_linear_x = 0;
            pub_angular_z = 0;
            // RCLCPP_INFO(this->get_logger(), "Serial step: %c, Step: %d, Cam distance: %f, Linear vel: %f, Angular vel: %f, person_recog: %d", serial_step, step_, cam_distance, pub_linear_x, pub_angular_z, person_recog);
            // Publish the mobile robot velocity command
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = pub_linear_x;
            twist_msg.angular.z = pub_angular_z;
            omo_cmd_vel_pub_->publish(twist_msg);

            break;
        }
            // case 5: // 플랩 완료 후 리니어 모터 하강
            // {
            //     serial_step = '2';
            //     pub_linear_x = 0;
            //     pub_angular_z = 0;
            //     // RCLCPP_INFO(this->get_logger(), "Serial step: %c, Step: %d, Cam distance: %f, Linear vel: %f, Angular vel: %f, person_recog: %d", serial_step, step_, cam_distance, pub_linear_x, pub_angular_z, person_recog);
            //     // Publish the mobile robot velocity command
            //     auto twist_msg = geometry_msgs::msg::Twist();
            //     twist_msg.linear.x = pub_linear_x;
            //     twist_msg.angular.z = pub_angular_z;
            //     omo_cmd_vel_pub_->publish(twist_msg);

            //     break;
            // }
            // case 6: // 리니어 하강 후 일정 시간 대기 및 리니어 상승
            // {
            //     now_time = this->now();
            //     elapsed_time = now_time - start_time; // 현재 시간과 시작 시간의 차이 계산
            //     if (elapsed_time.seconds() > 5.0)     // 5초 대기
            //     {
            //         serial_step = '3'; // 리니어 올리기
            //     }
            //     // pub_linear_x = 0;
            //     // pub_angular_z = 0;
            //     // // RCLCPP_INFO(this->get_logger(), "Serial step: %c, Step: %d, Cam distance: %f, Linear vel: %f, Angular vel: %f, person_recog: %d", serial_step, step_, cam_distance, pub_linear_x, pub_angular_z, person_recog);
            //     // //  Publish the mobile robot velocity command
            //     // auto twist_msg = geometry_msgs::msg::Twist();
            //     // twist_msg.linear.x = pub_linear_x;
            //     // twist_msg.angular.z = pub_angular_z;
            //     // omo_cmd_vel_pub_->publish(twist_msg);

            //     break;
            // }
            // case 7: // 리니어 상승 후 플랩 상승
            // {
            //     serial_step = '4'; // 플랩 상승
            //     pub_linear_x = 0;
            //     pub_angular_z = 0;
            //     // RCLCPP_INFO(this->get_logger(), "Serial step: %c, Step: %d, Cam distance: %f, Linear vel: %f, Angular vel: %f, person_recog: %d", serial_step, step_, cam_distance, pub_linear_x, pub_angular_z, person_recog);
            //     //  Publish the mobile robot velocity command
            //     auto twist_msg = geometry_msgs::msg::Twist();
            //     twist_msg.linear.x = pub_linear_x;
            //     twist_msg.angular.z = pub_angular_z;
            //     omo_cmd_vel_pub_->publish(twist_msg);

            //     break;
            // }
            // case 8: // 리니어 상승 완료
            // {
            //     serial_step = '4'; // 플랩 상승
            //     pub_linear_x = 0;
            //     pub_angular_z = 0;
            //     RCLCPP_INFO(this->get_logger(), "Step: %d, Cam distance: %f, Linear vel: %f, Angular vel: %f, person_recog: %d", step_, cam_distance, pub_linear_x, pub_angular_z, person_recog);
            //     // Publish the mobile robot velocity command
            //     auto twist_msg = geometry_msgs::msg::Twist();
            //     twist_msg.linear.x = pub_linear_x;
            //     twist_msg.angular.z = pub_angular_z;
            //     omo_cmd_vel_pub_->publish(twist_msg);

            //     break;
            // }

        default:
        {
            RCLCPP_WARN(this->get_logger(), "Invalid step: %d,", step_);
            break;
        }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainRunNode>());
    rclcpp::shutdown();
    return 0;
}
