#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

class UINode : public rclcpp::Node
{
public:
    UINode() : Node("ui_node")
    {
        pub_turtle1_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        pub_turtle2_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "UI Node avviato.");

        user_loop();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turtle2_;

    void user_loop()
    {
        while (rclcpp::ok())
        {
            std::string robot;
            double vel;

            std::cout << "Seleziona robot (turtle1 / turtle2): ";
            std::cin >> robot;

            std::cout << "Inserisci velocità lineare: ";
            std::cin >> vel;

            geometry_msgs::msg::Twist msg;
            msg.linear.x = vel;
            msg.angular.z = 0.0;

            auto start = std::chrono::steady_clock::now();

            // Pubblica per 1 secondo
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

            // Ferma il robot
            geometry_msgs::msg::Twist stop_msg;
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UINode>();
    rclcpp::shutdown();
    return 0;
}
