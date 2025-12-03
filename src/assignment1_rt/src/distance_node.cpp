#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;
#include <iostream>
#include <math.h>

class Distance_Check : public rclcpp::Node{ 
    public:
    Distance_Check(): Node("minimal_publisher"){ 
        
        //PUBLISHERS
        t1_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        t2_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
        distance_pub = this->create_publisher<std_msgs::msg::Float32>("/distance", 10);
        evasion_active_pub_ = this->create_publisher<std_msgs::msg::Int8>("/evasion_active", 10);

        //SUBSCRIBERS
        t1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&Distance_Check::turtle1_pose, this, _1));
        t2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10, std::bind(&Distance_Check::turtle2_pose, this, _1));        
        t1_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10, std::bind(&Distance_Check::turtle1_vel, this, _1));
        t2_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10, std::bind(&Distance_Check::turtle2_vel, this, _1));
        id_turtle_managed_sub_ = this->create_subscription<std_msgs::msg::Int8>("/id_turtle_managed", 10, std::bind(&Distance_Check::id_moving_turtle, this, _1));

        //TIMERS
        distance_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Distance_Check::distance_timer_callback, this));
        boundaries_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Distance_Check::boundaries_timer_callback, this));

        //VARIABLES
        stop_turtle.linear.x=0.0;
        stop_turtle.angular.z=0.0;
        id_turtle_moved.data = 0;

        //altrimenti legge subito posa 0.0 e le tartarughe si muovo all'indietro
        t1_pose.x=5.0;
        t1_pose.y=5.0;
        t2_pose.x=5.0;
        t2_pose.y=5.0;
    }
    
    private:

        void publish_evasion_status(int status_id) {
            auto msg = std::make_shared<std_msgs::msg::Int8>();
            msg->data = status_id; 
            evasion_active_pub_->publish(*msg);
        }

        void stop_and_clear_t1() {
            RCLCPP_INFO(this->get_logger(), "Reverse T1 Terminato. Stop.");
            t1_vel_pub_->publish(stop_turtle);
            is_reversing_t1_ = false;
            reverse_timer_t1_.reset();
            publish_evasion_status(0);
        }

        void stop_and_clear_t2() {
            RCLCPP_INFO(this->get_logger(), "Reverse T2 Terminato. Stop.");
            t2_vel_pub_->publish(stop_turtle);
            is_reversing_t2_ = false;
            reverse_timer_t2_.reset();
            publish_evasion_status(0);
        }

        geometry_msgs::msg::Twist check_direction_t1(){
            geometry_msgs::msg::Twist reverse_turtle1_vel;
            if(t1_vel.linear.x < 0){
                reverse_turtle1_vel.linear.x = 1;
            }else{
                reverse_turtle1_vel.linear.x = -1;
            } 
            return reverse_turtle1_vel;
        }

        geometry_msgs::msg::Twist check_direction_t2(){
            geometry_msgs::msg::Twist reverse_turtle2_vel;
            if(t2_vel.linear.x < 0){
                reverse_turtle2_vel.linear.x = 1;
            }else{
                reverse_turtle2_vel.linear.x = -1;
            } 
            return reverse_turtle2_vel;
        }

        void distance_timer_callback(){
            distance.data = sqrt(pow((t2_pose.x-t1_pose.x),2) + pow((t2_pose.y-t1_pose.y),2));
            distance_pub->publish(distance);
            std::cout << "Distanza:" << distance.data <<std::endl;
            if(distance.data < 1.0){
                if(id_turtle_moved.data == 1 && !is_reversing_t1_){
                    is_reversing_t1_ = true;

                    publish_evasion_status(1);

                    t1_vel_pub_->publish(check_direction_t1());

                    reverse_timer_t1_ = this->create_wall_timer(
                        std::chrono::seconds(1),
                        std::bind(&Distance_Check::stop_and_clear_t1, this)
                    );

                }else if(id_turtle_moved.data == 2 && !is_reversing_t2_){
                    is_reversing_t2_ = true;

                    publish_evasion_status(2);

                    t2_vel_pub_->publish(check_direction_t2());
                    
                    reverse_timer_t2_ = this->create_wall_timer(
                        std::chrono::seconds(1),
                        std::bind(&Distance_Check::stop_and_clear_t2, this)
                    );
                }
                id_turtle_moved.data = 0;
            }
        }

        void boundaries_timer_callback(){
            if(t1_pose.x > 10.0 || t1_pose.y > 10.0 || t1_pose.x < 1.0 || t1_pose.y < 1.0){
                std::cout << "Tartaruga 1 troppo vicina al bordo"<<std::endl;
                is_reversing_t1_ = true;
                publish_evasion_status(1);

                t1_vel_pub_->publish(check_direction_t1());

                reverse_timer_t1_ = this->create_wall_timer(
                    std::chrono::seconds(1),
                    std::bind(&Distance_Check::stop_and_clear_t1, this)
                );
            }

            if(t2_pose.x > 10.0 || t2_pose.y > 10.0 || t2_pose.x < 1.0 || t2_pose.y < 1.0){
                std::cout << "Tartaruga 2 troppo vicina al bordo"<<std::endl;
                is_reversing_t2_ = true;
                publish_evasion_status(2);

                t2_vel_pub_->publish(check_direction_t2());

                reverse_timer_t2_ = this->create_wall_timer(
                    std::chrono::seconds(1),
                    std::bind(&Distance_Check::stop_and_clear_t2, this)
                );
            }
        }

        void id_moving_turtle(const std_msgs::msg::Int8::SharedPtr msg){
            id_turtle_moved.data = msg->data;
        }

        void turtle1_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
            if (!is_reversing_t1_){
                t1_vel = *msg;
            }
        }

        void turtle2_vel(const geometry_msgs::msg::Twist::SharedPtr msg){
            if (!is_reversing_t2_){
                t2_vel = *msg;
            }
        }

        void turtle1_pose(const turtlesim::msg::Pose::SharedPtr msg){
            t1_pose.x = msg->x;
            t1_pose.y = msg->y;
        }
        void turtle2_pose(const turtlesim::msg::Pose::SharedPtr msg){
            t2_pose.x = msg->x;
            t2_pose.y = msg->y;
        }

        //PUBLISHERS
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr t1_vel_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr t2_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr evasion_active_pub_;

        //SUBSCRIBERS
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t1_pose_sub_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr t2_pose_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr t1_vel_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr t2_vel_sub_;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr id_turtle_managed_sub_;

        //TIMERS
        rclcpp::TimerBase::SharedPtr distance_timer_;
        rclcpp::TimerBase::SharedPtr boundaries_timer_;
        rclcpp::TimerBase::SharedPtr reverse_timer_t1_;
        rclcpp::TimerBase::SharedPtr reverse_timer_t2_;

        //VARIABLES
        turtlesim::msg::Pose t1_pose;
        turtlesim::msg::Pose t2_pose;
        geometry_msgs::msg::Twist stop_turtle;
        geometry_msgs::msg::Twist t1_vel;
        geometry_msgs::msg::Twist t2_vel;
        std_msgs::msg::Float32 distance;
        std_msgs::msg::Int8 id_turtle_moved;
        bool is_reversing_t1_ = false;
        bool is_reversing_t2_ = false;

};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Distance_Check>());
    rclcpp::shutdown();
    return 0;
}
 