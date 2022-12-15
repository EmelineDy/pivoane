

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>
#include <string>

#include "interfaces/msg/sign_data.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/reaction.hpp"
#include "darknet_ros_msgs/msg/bounding_box.hpp"



#include "../include/odometry/odometry_node.h"

using namespace std;
using placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class odometry : public rclcpp::Node {
  public:
    odometry()
    : Node("odometry_node")
    {
      publisher_reaction_ = this->create_publisher<interfaces::msg::Reaction>("reaction", 10);
   
      subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&odometry::motorsFeedbackCallback, this, _1));
      
      subscription_sign_data_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBox>(
        "sign_data", 10, std::bind(&odometry::signDataCallback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "odometry_node READY");
    }

  private:

    //Publisher
    rclcpp::Publisher<interfaces::msg::Reaction>::SharedPtr publisher_reaction_;

    //Subscriber
    rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBox>::SharedPtr subscription_sign_data_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;

    float totalDistance = 0;
    string pastSignType = "f";
    float pastSignDist = 3000;
    bool reacted = false;


    //Calculate distance travelled by the car based on odometry
    float calculateDistance(int leftRearOdometry, int rightRearOdometry){
      float distanceLeft = (leftRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;
      float distanceRight = (rightRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;

      float distance = (distanceLeft + distanceRight)/2.0;
      return distance;
    }

    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){

      auto reactMsg = interfaces::msg::Reaction();

      //If the distance between the car and the last detected sign is null, the car can react
      if ((pastSignDist - totalDistance ) > 0) {
        //Calculate distance traveled in total
        totalDistance += calculateDistance(motorsFeedback.left_rear_odometry, motorsFeedback.right_rear_odometry);
      } else if (reacted == false){
        reacted = true;
        reactMsg.react = true;
        publisher_reaction_->publish(reactMsg);   
        RCLCPP_INFO(this->get_logger(), "React TRUE");     
      }
    }

    void signDataCallback(const darknet_ros_msgs::msg::BoundingBox & signData){

      auto reactMsg = interfaces::msg::Reaction();

      if (signData.class_id != pastSignType && signData.class_id != "person") {
        pastSignType = signData.class_id;
        pastSignDist = signData.distance;
        totalDistance = 0;
        reactMsg.react = false;
        reacted = false;
        publisher_reaction_->publish(reactMsg);
        RCLCPP_INFO(this->get_logger(), "React FALSE");
      }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<odometry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
