

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
        "close_bounding_boxes", 1, std::bind(&odometry::signDataCallback, this, _1));
    
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
    int nb_warning = 0;

    int count_stop = 0;
    int count_30 = 0;
    int count_50 = 0;
    int count_bump = 0; 

    int max_count = 30;
    int threshold = 20;
    int counts[4]= {count_stop, count_30, count_50, count_bump};
    int index = -1;
    int max_detect = 0;
    int sure[4] = {0, 0, 0, 0};
    string label[4] = {"stop", "speed30", "speed50", "speedbump"};


    //Calculate distance travelled by the car based on odometry
    float calculateDistance(int leftRearOdometry, int rightRearOdometry){
      float distanceLeft = (leftRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;
      float distanceRight = (rightRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;

      float distance = (distanceLeft + distanceRight)/2.0;
      return distance;
    }


    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
      int x = 0;
      /*
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
        }
      }
      */
    }

    void signDataCallback(const darknet_ros_msgs::msg::CloseBoundingBoxes & signData){

      auto reactMsg = interfaces::msg::Reaction();

      if (signData.close_bounding_boxes[0].class_id == "stop") {
        count_stop = min (count_stop++, max_count);
        count_30 = max (count_30--, 0);
        count_50 = max (count_50--, 0);
        count_bump = max (count_bump--, 0);
      } else if (signData.close_bounding_boxes[0].class_id == "speedbump") {
        count_bump = min (count_bump++, max_count);          
        count_30 = max (count_30--, 0);
        count_50 = max (count_50--, 0);
        count_stop = max (count_stop--, 0);
      } else if (signData.close_bounding_boxes[0].class_id == "speed30") {
        count_30 = min (count_30++, max_count);
        count_stop = max (count_stop--, 0);
        count_50 = max (count_50--, 0);
        count_bump = max (count_bump--, 0);
      } else if (signData.close_bounding_boxes[0].class_id == "speed50") {
        count_50 = min (count_50++, max_count);
        count_30 = max (count_30--, 0);
        count_stop = max (count_stop--, 0);
        count_bump = max (count_bump--, 0);
      } else {
        count_50 = max (count_50--, 0);
        count_30 = max (count_30--, 0);
        count_stop = max (count_stop--, 0);
        count_bump = max (count_bump--, 0);
      }

      counts[0] = count_stop;
      counts[1] = count_30;
      counts[2] = count_50;
      counts[3] = count_bump;

      for (int i = 0; i>4; i++) {
        if (counts[i] > max_detect) {
          max_detect = counts[i];
          index = i;
        }
      }

      if (max_detect >= threshold) {
        sure[index] = 1;
      }

      for (int i = 0; i>4; i++) {
        if (sure[i] == 1 && counts[i] <= 5) {
          reactMsg.class_id = label[i];
          publisher_reaction_->publish(reactMsg);
          RCLCPP_INFO(this->get_logger(), "React TRUE");

          sure[i] = 0;
        }
      }



      // Think about trouble to change the behavior of the vehicle after lose the vision of the 
      /*
    if (signData.close_bounding_boxes[0].class_id != pastSignType  && signData.close_bounding_boxes[0].class_id != "empty") {
      pastSignType = signData.close_bounding_boxes[0].class_id;
      pastSignDist = signData.close_bounding_boxes[0].distance;
      totalDistance = 0;
      reactMsg.react = false;
      start_reacting = false;
      publisher_reaction_->publish(reactMsg);
      RCLCPP_INFO(this->get_logger(), "React FALSE");
      }
      */
      

  
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
