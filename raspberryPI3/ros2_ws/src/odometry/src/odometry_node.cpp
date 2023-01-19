

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>
#include <string>

#include "interfaces/msg/sign_data.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/reaction.hpp"
#include "darknet_ros_msgs/msg/close_bounding_boxes.hpp"

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

      //subscription_finish_ = this->create_subscription<interfaces::msg::Finish>(
      //  "finish", 10, std::bind(&odometry::finishCallback, this, _1));
      
      subscription_sign_data_ = this->create_subscription<darknet_ros_msgs::msg::CloseBoundingBoxes>(
        "/darknet_ros/close_bounding_boxes", 1, std::bind(&odometry::signDataCallback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "odometry_node READY");
    }

  private:

    //Publisher
    rclcpp::Publisher<interfaces::msg::Reaction>::SharedPtr publisher_reaction_;

    //Subscriber
    rclcpp::Subscription<darknet_ros_msgs::msg::CloseBoundingBoxes>::SharedPtr subscription_sign_data_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    //rclcpp::Subscription<interfaces::msg::Finish>::SharedPtr subscription_finish_;

    float totalDistance = 0;
    string pastSignType = "f";
    float pastSignDist = 3000;
    bool start_reacting = false;
    //bool finished_reacting = true;
    int nb_warning = 0;


    //Calculate distance travelled by the car based on odometry
    float calculateDistance(int leftRearOdometry, int rightRearOdometry){
      float distanceLeft = (leftRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;
      float distanceRight = (rightRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;

      float distance = (distanceLeft + distanceRight)/2.0;
      return distance;
    }

    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){

      //ERROR in case leftRearOdometry or rightRearOdometry are negative 5 times in a row
      if(nb_warning >= 5){
        RCLCPP_ERROR(this->get_logger(), "Error : wrong motors feedback for too long");
      } else if(motorsFeedback.left_rear_odometry < 0 || motorsFeedback.right_rear_odometry < 0 || motorsFeedback.left_rear_odometry > 2 || motorsFeedback.right_rear_odometry > 2){
        //WARNING in case leftRearOdometry or rightRearOdometry are negative
        RCLCPP_WARN(this->get_logger(), "Warning : wrong motors feedback");
        nb_warning += 1;
      } else {
        nb_warning = 0;
        
        auto reactMsg = interfaces::msg::Reaction();

        //If the distance between the car and the last detected sign is null, the car can react
        if ((pastSignDist - totalDistance ) > 0) {
          //Calculate distance traveled in total
          totalDistance += calculateDistance(motorsFeedback.left_rear_odometry, motorsFeedback.right_rear_odometry);
        } else if (start_reacting == false){
          start_reacting = true;
          reactMsg.react = true;
          reactMsg.class_id = pastSignType;
          publisher_reaction_->publish(reactMsg);   
          RCLCPP_INFO(this->get_logger(), "React TRUE");     
        } else {
          RCLCPP_INFO(this->get_logger(), "Started Reacting");
        }
      }
    }

    void signDataCallback(const darknet_ros_msgs::msg::CloseBoundingBoxes & signData){

      auto reactMsg = interfaces::msg::Reaction();

        // Think about trouble to change the behavior of the vehicle after lose the vision of the 
      if (signData.bounding_boxes[0].class_id != pastSignType  && signData.bounding_boxes[0].class_id != "empty") {
        pastSignType = signData.bounding_boxes[0].class_id;
        pastSignDist = signData.bounding_boxes[0].distance;
        totalDistance = 0;
        reactMsg.react = false;
        start_reacting = false;
        publisher_reaction_->publish(reactMsg);
        RCLCPP_INFO(this->get_logger(), "React FALSE");
        }

      if (signData.bounding_boxes[1].class_id != "empty") {
        RCLCPP_INFO(this->get_logger(), "Pedestrian detected");
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
