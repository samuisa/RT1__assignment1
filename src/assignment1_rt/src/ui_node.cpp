#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

// ----------------- NODE CLASS DEFINITION -----------------
class UINode : public rclcpp::Node
{
public:
    UINode() : Node("ui_node")
    {
        // 1. CREATE PUBLISHERS FOR RAW COMMAND TOPICS
        // These commands will be filtered by the distance_node.cpp
        pub_turtle1_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/raw_cmd_vel", 10);
        pub_turtle2_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/raw_cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "UI Node avviato.");

        // 2. START THE USER INTERACTION LOOP
        user_loop();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle2_;

    void user_loop()
    {
        // 3. MAIN LOOP: CONTINUOUSLY PROMPT USER FOR COMMANDS
        while (rclcpp::ok())
        {
            std::string robot;
            double vel_linear;
            double vel_angular;

            // 4. GET USER INPUT (TARGET, LINEAR, ANGULAR VELOCITY)
            std::cout << "Seleziona robot (turtle1 / turtle2): ";
            std::cin >> robot;

            std::cout << "Inserisci velocità lineare: ";
            std::cin >> vel_linear;

            std::cout << "Inserisci velocità angolare: ";
            std::cin >> vel_angular;

            geometry_msgs::msg::Twist msg;
            msg.linear.x = vel_linear;
            msg.angular.z = vel_angular;

            // 5. PUBLISH COMMAND FOR A FIXED DURATION (1 SECOND)
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start < 1s)
            {
                if (robot == "turtle1")
                {
                    pub_turtle1_->publish(msg);
                }
                else if (robot == "turtle2")
                {
                    pub_turtle2_->publish(msg);
                }
                else
                {
                    std::cout << "Robot non valido!" << std::endl;
                    break;
                }

                std::this_thread::sleep_for(100ms);
            }

            // 6. AUTO-STOP: PUBLISH ZERO VELOCITY AFTER 1 SECOND
            geometry_msgs::msg::Twist stop_msg; // Default initialized to zeros
            if (robot == "turtle1")
            {
                pub_turtle1_->publish(stop_msg);
            }
            else if (robot == "turtle2")
            {
                pub_turtle2_->publish(stop_msg);
            }

            std::cout << "Robot fermato. Inserisci un nuovo comando.\n" << std::endl;
        }
    }
};

// ----------------- MAIN EXECUTION BLOCK -----------------
int main(int argc, char *argv[])
{
    // 7. ROS 2 LIFECYCLE MANAGEMENT (INIT, SPIN, SHUTDOWN)
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UINode>();
    // Note: The user_loop runs independently of rclcpp::spin, as it contains its own blocking logic.
    // However, the node must exist until the loop ends.
    rclcpp::shutdown();
    return 0;
}