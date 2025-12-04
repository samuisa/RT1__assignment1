#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using std::placeholders::_1;

class ReactiveDistanceCheck : public rclcpp::Node {
public:
    ReactiveDistanceCheck() : Node("reactive_distance_check") {
        t1_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        t2_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

        t1_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/raw_cmd_vel", 10, std::bind(&ReactiveDistanceCheck::t1_cmd_cb, this, _1));
        t2_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle2/raw_cmd_vel", 10, std::bind(&ReactiveDistanceCheck::t2_cmd_cb, this, _1));

        t1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&ReactiveDistanceCheck::t1_pose_cb, this, _1));
        t2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose", 10, std::bind(&ReactiveDistanceCheck::t2_pose_cb, this, _1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ReactiveDistanceCheck::control_loop, this));

        stop_msg_.linear.x = 0.0;
        stop_msg_.angular.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Reactive Distance Check avviato!");
    }

private:
    turtlesim::msg::Pose t1_pose_, t2_pose_;
    geometry_msgs::msg::Twist t1_cmd_, t2_cmd_;
    geometry_msgs::msg::Twist stop_msg_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr t1_pub_, t2_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr t1_cmd_sub_, t2_cmd_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t1_pose_sub_, t2_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    //---------------- CALLBACK ----------------
    void t1_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg) { t1_cmd_ = *msg; }
    void t2_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg) { t2_cmd_ = *msg; }

    void t1_pose_cb(const turtlesim::msg::Pose::SharedPtr msg) { t1_pose_ = *msg; }
    void t2_pose_cb(const turtlesim::msg::Pose::SharedPtr msg) { t2_pose_ = *msg; }

    //---------------- CONTROL LOOP ----------------
    void control_loop() {
        geometry_msgs::msg::Twist cmd1 = t1_cmd_;
        geometry_msgs::msg::Twist cmd2 = t2_cmd_;

        // Blocca linear.x se troppo vicino a turtle2 o ai bordi
        if (distance(t1_pose_, t2_pose_) < 1.0 || at_wall(t1_pose_)) {
            cmd1.linear.x = 0.0;
        }
        if (distance(t2_pose_, t1_pose_) < 1.0 || at_wall(t2_pose_)) {
            cmd2.linear.x = 0.0;
        }

        t1_pub_->publish(cmd1);
        t2_pub_->publish(cmd2);
    }

    //---------------- UTILITY ----------------
    float distance(const turtlesim::msg::Pose &a, const turtlesim::msg::Pose &b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    bool at_wall(const turtlesim::msg::Pose &pose) {
        return (pose.x < 1.0 || pose.x > 10.0 || pose.y < 1.0 || pose.y > 10.0);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveDistanceCheck>());
    rclcpp::shutdown();
    return 0;
}
