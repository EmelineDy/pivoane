

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>

#include "interfaces/msg/sign_data.hpp"
#include "interfaces/msg/motors_feedback.hpp"


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
      publisher_sign_data_ = this->create_publisher<interfaces::msg::SignData>("sign_data", 10);
   
      subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&odometry::motorsFeedbackCallback, this, _1));
      
      subscription_sign_data_ = this->create_subscription<interfaces::msg::SignData>(
        "sign_data", 10, std::bind(&odometry::signDataCallback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "odometry_node READY");
    }

  private:

    //Publisher
    rclcpp::Publisher<interfaces::msg::SignData>::SharedPtr publisher_sign_data_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::SignData>::SharedPtr subscription_sign_data_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;

    float totalDistance = 0;
    float pastSignDist = -1;


    //Calculate distance travelled by the car based on odometry
    float calculateDistance(int leftRearOdometry, int rightRearOdometry){
      float distanceLeft = (leftRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;
      float distanceRight = (rightRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;

      float distance = (distanceLeft + distanceRight)/2.0;
      return distance;
    }

    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){

      //Calculate distance traveled in total
      totalDistance += calculateDistance(motorsFeedback.left_rear_odometry,motorsFeedback.right_rear_odometry);

      auto signMsg = interfaces::msg::SignData();

      //If the distance between the car and the last detected sign is null, the car can react
      if ((pastSignDist - totalDistance ) <= 0) {
        signMsg.react = true;
        publisher_sign_data_->publish(signMsg);
      } else if (signMsg.react) {
        signMsg.react = false;
        publisher_sign_data_->publish(signMsg);
      }
    }

    void signDataCallback(const interfaces::msg::SignData & signData){

      if (signData.sign_distance != pastSignDist) {
          pastSignDist = signData.sign_distance;
          totalDistance = 0;
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
