/* This file contains the code that describes the state machine simulating the behavior of the car and therefore the publishing of 
the message giving the value of the speed that the car needs to reach according to the road sign or pedestrian detected */

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>
#include <string>

#include "interfaces/msg/obstacles.hpp"
#include "interfaces/msg/required_speed.hpp"
#include "interfaces/msg/reaction.hpp"
#include "interfaces/msg/sign_data.hpp"


#include "../include/detection_behavior/detection_behavior_node.h"

using namespace std;
using placeholders::_1;


class detection_behavior : public rclcpp::Node {
  public:
    detection_behavior()
    : Node("detection_behavior_node")
    {
      publisher_required_speed_ = this->create_publisher<interfaces::msg::RequiredSpeed>("required_speed", 10);

      subscription_obstacles_ = this->create_subscription<interfaces::msg::Obstacles>(
        "obstacles", 10, std::bind(&detection_behavior::obsDataCallback, this, _1));
      
      subscription_reaction_ = this->create_subscription<interfaces::msg::Reaction>(
        "reaction", 10, std::bind(&detection_behavior::reactionCallback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "detection behavior READY");

      timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&detection_behavior::updateSpeed, this));

    }

  private:

    //Speed variable
    uint8_t us_detect = 0;
    string ai_detect = "";

    int last_speed = 60;          // last speed recorded
    int current_speed = 60;       // current speed of the car
    int speed_before_obs = 0;     // speed recorded before obstacle
    int speed_before_stop = 0;    // speed recorded before stop sign
    int speed_before_sb = 0;      // speed recorded before speed bump

    int counter = 0;


    //Publisher
    rclcpp::Publisher<interfaces::msg::RequiredSpeed>::SharedPtr publisher_required_speed_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::Obstacles>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<interfaces::msg::Reaction>::SharedPtr subscription_reaction_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

    void updateSpeed(){

      auto speedMsg = interfaces::msg::RequiredSpeed();

      // us sensors send back anormal values
      if(us_detect == 2){
        last_speed = current_speed;
        current_speed = 0;
      }
      // us sensors detect a pedestrian
      else if(us_detect == 1){
        if(current_speed != 0){
          RCLCPP_INFO(this->get_logger(), "Pedestrian: speed is 0");
          speed_before_obs = current_speed;
        }
        current_speed = 0;

      }  else if (speed_before_obs == 0) {

        // detection of stop sign
        if(ai_detect == "stop"){
          if(current_speed != 0 && counter == 0){
            RCLCPP_INFO(this->get_logger(), "Stop sign: speed is 0 rpm");
            speed_before_stop = current_speed;
          }
          // counter so that the car stops moving for 2s
          if(counter!=2000){
            current_speed = 0;
            counter ++;
          }else{
            // the car recovers the speed before the stop
            current_speed = speed_before_stop;
          }

        // detection of a speed bump sign 
        } else if(ai_detect == "speedbump"){
          if(current_speed != 30 && counter == 0){
            RCLCPP_INFO(this->get_logger(), "Speed bump sign: speed is 30 rpm");
            speed_before_sb = current_speed;
          }
          // counter so that the car moves at 30 rpm for 4s
          if(counter!=4000){
            current_speed = 30;
            counter ++;
          }else{
            // the car recovers the speed before the speed bump
            current_speed = speed_before_sb;
          }

        // detection of speed road sign 30  
        } else if (ai_detect == "speed30") {
          if(current_speed != 36){
            RCLCPP_INFO(this->get_logger(), "Low speed sign: speed is 36 rpm");
            last_speed = current_speed;
          }
          current_speed = 36;

        // detection of speed road sign 50 
        } else if (ai_detect == "speed50") {
          last_speed = 0;
          current_speed = 60;
        }

      // nothing is detected by the us sensors    
      } else if (us_detect == 0) {
        if(speed_before_obs != 0){
          current_speed = speed_before_obs;
          speed_before_obs = 0;
          RCLCPP_INFO(this->get_logger(), "pedestrian gone: recovering previous speed %i rpm", speed_before_obs);
          }      
      } 
      speedMsg.speed_rpm = current_speed;

      // publishing the speed message
      publisher_required_speed_->publish(speedMsg);
    }

    // getting the information of us_detect (see us_detection node)
    void obsDataCallback(const interfaces::msg::Obstacles & obstacles){
      if (us_detect != obstacles.us_detect) {
        us_detect = obstacles.us_detect;
      }  
    }

    // getting the information of road sign reached by the car (see odometry node)
    void reactionCallback(const interfaces::msg::Reaction & reaction) {
      counter = 0;
      ai_detect = reaction.class_id; 
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<detection_behavior>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
