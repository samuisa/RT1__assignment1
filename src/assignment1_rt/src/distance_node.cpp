#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <algorithm>

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

        dt_ = 0.05; // 50 ms

        RCLCPP_INFO(this->get_logger(), "Reactive Distance Check avviato!");
    }

private:
    turtlesim::msg::Pose t1_pose_, t2_pose_;
    geometry_msgs::msg::Twist t1_cmd_, t2_cmd_;
    double dt_;

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

        float dist_turtles = distance(t1_pose_, t2_pose_);
        float wall_t1 = wall_distance(t1_pose_);
        float wall_t2 = wall_distance(t2_pose_);

        RCLCPP_INFO(this->get_logger(),
            "DISTANZA TARTARUGHE: %.2f | T1-MURO: %.2f | T2-MURO: %.2f",
            dist_turtles, wall_t1, wall_t2);

        handle_turtle(cmd1, t1_pose_, t2_pose_);
        handle_turtle(cmd2, t2_pose_, t1_pose_);

        t1_pub_->publish(cmd1);
        t2_pub_->publish(cmd2);
    }

    //---------------- LOGICA DI SICUREZZA ----------------
    void handle_turtle(geometry_msgs::msg::Twist &cmd,
                       const turtlesim::msg::Pose &self,
                       const turtlesim::msg::Pose &other)
    {
        // Limite velocità (evita blocchi improvvisi)
        cmd.linear.x  = clamp(cmd.linear.x, -2.0, 2.0);
        cmd.angular.z = clamp(cmd.angular.z, -4.0, 4.0);

        bool too_close = (distance(self, other) < 1.0 || at_wall(self));

        // Se troppo vicino → niente avanzamento, solo rotazione
        if (too_close)
        {
            cmd.linear.x = 0.0;
            return;
        }

        // Previsione futura
        float next_x = self.x + cmd.linear.x * std::cos(self.theta) * dt_;
        float next_y = self.y + cmd.linear.x * std::sin(self.theta) * dt_;

        turtlesim::msg::Pose future_pose = self;
        future_pose.x = next_x;
        future_pose.y = next_y;

        // Se la prossima posizione causerebbe collisione → blocco solo lineare
        if (distance(future_pose, other) < 1.0 || at_wall(future_pose))
        {
            cmd.linear.x = 0.0;
        }
    }

    //---------------- UTILITY ----------------
    float distance(const turtlesim::msg::Pose &a, const turtlesim::msg::Pose &b) {
        return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
    }

    bool at_wall(const turtlesim::msg::Pose &pose) {
        return (pose.x < 1.0 || pose.x > 10.0 || pose.y < 1.0 || pose.y > 10.0);
    }

    float wall_distance(const turtlesim::msg::Pose &pose)
    {
        float left   = pose.x;
        float right  = 11.0 - pose.x;
        float bottom = pose.y;
        float top    = 11.0 - pose.y;

        return std::min({left, right, bottom, top});
    }

    float clamp(float v, float min, float max)
    {
        if (v < min) return min;
        if (v > max) return max;
        return v;
    }
};

//------------------- MAIN -------------------
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveDistanceCheck>());
    rclcpp::shutdown();
    return 0;
}
