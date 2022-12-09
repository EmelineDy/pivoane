

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>

#include "interfaces/msg/sign_info.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/reaction.hpp"
#include "interfaces/msg/behavior_info.hpp"


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

      subscription_behavior_info_ = this->create_subscription<interfaces::msg::BehaviorInfo>(
        "behavior_info", 10, std::bind(&odometry::behaviorCallback, this, _1));
      
      subscription_sign_info_ = this->create_subscription<interfaces::msg::SignInfo>(
        "sign_info", 10, std::bind(&odometry::signInfoCallback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "odometry_node READY");
    }

  private:

    //Publisher
    rclcpp::Publisher<interfaces::msg::Reaction>::SharedPtr publisher_reaction_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::SignInfo>::SharedPtr subscription_sign_info_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::BehaviorInfo>::SharedPtr subscription_behavior_info_;

    float totalDistance = 0;
    float pastSignDist = 20000;


    //Calculate distance travelled by the car based on odometry
    float calculateDistance(int leftRearOdometry, int rightRearOdometry){
      float distanceLeft = (leftRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;
      float distanceRight = (rightRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;

      float distance = (distanceLeft + distanceRight)/2.0;
      return distance;
    }

    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){

      //Calculate distance traveled in total
      totalDistance += calculateDistance(motorsFeedback.left_rear_odometry, motorsFeedback.right_rear_odometry);

      auto reactMsg = interfaces::msg::Reaction();

      //If the distance between the car and the last detected sign is null, the car can react
      if ((pastSignDist - totalDistance ) <= 0) {
        reactMsg.react = true;
        publisher_reaction_->publish(reactMsg);
      }
    }

    void signInfoCallback(const interfaces::msg::SignInfo & signInfo){

      //auto reactMsg = interfaces::msg::Reaction();

      if (signInfo.sign_distance != pastSignDist) {
        pastSignDist = signInfo.sign_distance;
        totalDistance = 0;
        //reactMsg.react = false;
        //publisher_reaction_->publish(reactMsg);
      }
    }

    void behaviorCallback(const interfaces::msg::BehaviorInfo & behaviorInfo){

      auto reactMsg = interfaces::msg::Reaction();

      if (behaviorInfo.done == true) {
        reactMsg.react = false;
        publisher_reaction_->publish(reactMsg);
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
