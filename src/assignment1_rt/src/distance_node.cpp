#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

class ReactiveDistanceCheck : public rclcpp::Node {
public:
    ReactiveDistanceCheck() : Node("reactive_distance_check") {

        // Publishers for filtered commands
        t1_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        t2_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

        // Subscriptions for raw desired commands (from ui_node)
        t1_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/raw_cmd_vel", 10, std::bind(&ReactiveDistanceCheck::t1_cmd_cb, this, _1));
        t2_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle2/raw_cmd_vel", 10, std::bind(&ReactiveDistanceCheck::t2_cmd_cb, this, _1));

        // Subscriptions for current poses
        t1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&ReactiveDistanceCheck::t1_pose_cb, this, _1));
        t2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose", 10, std::bind(&ReactiveDistanceCheck::t2_pose_cb, this, _1));

        // Timer for the 20 Hz control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ReactiveDistanceCheck::control_loop, this));

        dt_ = 0.05; // 50 ms time step for prediction

        RCLCPP_INFO(this->get_logger(), "Reactive Distance Check avviato!");
    }

private:
    turtlesim::msg::Pose t1_pose_, t2_pose_;
    geometry_msgs::msg::Twist t1_cmd_, t2_cmd_;
    double dt_;

    bool hit_t1_ = false;
    bool hit_t2_ = false;

    bool t1_pose_ready_ = false;
    bool t2_pose_ready_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr t1_pub_, t2_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr t1_cmd_sub_, t2_cmd_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t1_pose_sub_, t2_pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    //--- CALLBACKS (Store latest data) ---
    void t1_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg) { t1_cmd_ = *msg; }
    void t2_cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg) { t2_cmd_ = *msg; }

    void t1_pose_cb(const turtlesim::msg::Pose::SharedPtr msg) { 
        t1_pose_ = *msg; 
        t1_pose_ready_ = true;
    }
    void t2_pose_cb(const turtlesim::msg::Pose::SharedPtr msg) { 
        t2_pose_ = *msg; 
        t2_pose_ready_ = true;
    }

    //--- CONTROL LOOP (Periodic execution) ---
    void control_loop() {
        // Skip loop until we have valid poses for both turtles
        if (!t1_pose_ready_ || !t2_pose_ready_) return;

        geometry_msgs::msg::Twist cmd1 = t1_cmd_;
        geometry_msgs::msg::Twist cmd2 = t2_cmd_;

        // Apply safety logic to both turtles
        handle_turtle(cmd1, t1_pose_, t2_pose_, hit_t1_);
        handle_turtle(cmd2, t2_pose_, t1_pose_, hit_t2_);

        // Publish filtered commands
        t1_pub_->publish(cmd1);
        t2_pub_->publish(cmd2);
    }

    //--- SAFETY LOGIC ---
    void handle_turtle(geometry_msgs::msg::Twist &cmd,
                       const turtlesim::msg::Pose &self,
                       const turtlesim::msg::Pose &other,
                       bool &hit_flag)
    {
        // 1. Velocity clamping
        cmd.linear.x  = clamp(cmd.linear.x, -5.0, 5.0);
        cmd.angular.z = clamp(cmd.angular.z, -5.0, 5.0);

        // 2. Critical Zone Check
        float dist = distance(self, other);
        bool wall  = at_wall(self);
        bool too_close = (dist <= 1.0 || wall);

        // State Management
        if (too_close && !hit_flag)
            hit_flag = true;
        if (!too_close && hit_flag && dist > 1.2 && !wall)
            hit_flag = false;

        // 3. IMMEDIATE BLOCK in critical zone (only angular movement allowed)
        if (too_close)
        {
            cmd.linear.x = 0.0;
            RCLCPP_INFO(this->get_logger(), "Turtle blocked, you are in a critical section");
            return;
        }

        // 4. PREDICTIVE CHECK: Calculate future pose (after dt_)
        float next_x = self.x + cmd.linear.x * std::cos(self.theta) * dt_;
        float next_y = self.y + cmd.linear.x * std::sin(self.theta) * dt_;

        turtlesim::msg::Pose future_pose = self;
        future_pose.x = next_x;
        future_pose.y = next_y;

        // 5. PREDICTIVE BLOCK: Block linear if future pose is critical
        if (distance(future_pose, other) <= 1.0 || at_wall(future_pose))
        {
            cmd.linear.x = 0.0;
            RCLCPP_INFO(this->get_logger(), "Turtle blocked, you are in a critical section");
        }
    }

    //--- UTILITY FUNCTIONS ---
    float distance(const turtlesim::msg::Pose &a, const turtlesim::msg::Pose &b) {
        return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
    }

    bool at_wall(const turtlesim::msg::Pose &pose) {
        // Small margin to avoid false positives at startup
        return (pose.x <= 0.5 || pose.x >= 10.5 || pose.y <= 0.5 || pose.y >= 10.5);
    }

    float clamp(float v, float min, float max)
    {
        if (v < min) return min;
        if (v > max) return max;
        return v;
    }
};

//--- MAIN ---
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveDistanceCheck>());
    rclcpp::shutdown();
    return 0;
}
