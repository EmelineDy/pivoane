

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>
#include <string>

#include "interfaces/msg/sign_data.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/reaction.hpp"
#include "interfaces/msg/finish.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "darknet_ros_msgs/msg/object_count.hpp"

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

      subscription_finish_ = this->create_subscription<interfaces::msg::Finish>(
        "finish", 10, std::bind(&odometry::finishCallback, this, _1));
      
      subscription_sign_data_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
        "/darknet_ros/bounding_boxes", 1, std::bind(&odometry::signDataCallback, this, _1));

      subscription_number_detection_ = this->create_subscription<darknet_ros_msgs::msg::ObjectCount>(
        "/darknet_ros/found_object", 1, std::bind(&odometry::countCallback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "odometry_node READY");
    }

  private:

    //Publisher
    rclcpp::Publisher<interfaces::msg::Reaction>::SharedPtr publisher_reaction_;

    //Subscriber
    rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr subscription_sign_data_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::Finish>::SharedPtr subscription_finish_;
    rclcpp::Subscription<darknet_ros_msgs::msg::ObjectCount>::SharedPtr subscription_number_detection_;

    float totalDistance = 0;
    string pastSignType = "f";
    float pastSignDist = 3000;
    bool start_reacting = false;
    bool finished_reacting = true;
    int number_obj = 0;
    int nb_warning = 0;


    //Calculate distance travelled by the car based on odometry
    float calculateDistance(int leftRearOdometry, int rightRearOdometry){
      float distanceLeft = (leftRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;
      float distanceRight = (rightRearOdometry * 3.141592 * (WHEEL/10.0)) / 36.0;

      float distance = (distanceLeft + distanceRight)/2.0;
      return distance;
    }

    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){

      if(motorsFeedback.left_rear_odometry < 0 || motorsFeedback.right_rear_odometry < 0 || motorsFeedback.left_rear_odometry > 2 || motorsFeedback.right_rear_odometry > 2){
        if (nb_warning < 5) {
          //WARNING in case leftRearOdometry or rightRearOdometry are negative
          nb_warning += 1;
          RCLCPP_WARN(this->get_logger(), "Warning n°%i: wrong motors feedback", nb_warning);
        } else {
          //ERROR in case leftRearOdometry or rightRearOdometry are negative 5 times in a row
          RCLCPP_ERROR(this->get_logger(), "Error: wrong motors feedback for too long");
        }
      }
      else{
        nb_warning = 0;
        
        auto reactMsg = interfaces::msg::Reaction();

        //If the distance between the car and the last detected sign is null, the car can react
        if ((pastSignDist - totalDistance ) > 0) {
          //Calculate distance traveled in total
          totalDistance += calculateDistance(motorsFeedback.left_rear_odometry, motorsFeedback.right_rear_odometry);
        } else if (start_reacting == false && finished_reacting == false){
          start_reacting = true;
          reactMsg.react = true;
          reactMsg.class_id = pastSignType;
          publisher_reaction_->publish(reactMsg);   
          RCLCPP_INFO(this->get_logger(), "React TRUE");     
        }
      }
    }


    void finishCallback(const interfaces::msg::Finish & finish){
      finished_reacting = finish.ended;
      RCLCPP_INFO(this->get_logger(), "Receive finish value");
    }


    void signDataCallback(const darknet_ros_msgs::msg::BoundingBoxes & signData){

      auto reactMsg = interfaces::msg::Reaction();


      for (int i = 0; i<number_obj; i++){
        // Think about trouble to change the behavior of the vehicle after lose the vision of the 
        if (signData.bounding_boxes[i].class_id != pastSignType && signData.bounding_boxes[i].class_id != "person" && (finished_reacting == true || signData.bounding_boxes[i].distance < pastSignDist - totalDistance)) {
        pastSignType = signData.bounding_boxes[i].class_id;
        pastSignDist = signData.bounding_boxes[i].distance;
        totalDistance = 0;
        reactMsg.react = false;
        finished_reacting = false;
        start_reacting = false;
        publisher_reaction_->publish(reactMsg);
        RCLCPP_INFO(this->get_logger(), "React FALSE");
      }
      }
      
    }

    void countCallback(const darknet_ros_msgs::msg::ObjectCount & objectcount){
        number_obj = objectcount.count;
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
