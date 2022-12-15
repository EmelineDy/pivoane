

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>
#include <string>

#include "interfaces/msg/obstacles.hpp"
#include "interfaces/msg/required_speed.hpp"
#include "interfaces/msg/reaction.hpp"
#include "interfaces/msg/sign_data.hpp"
#include "darknet_ros_msgs/msg/bounding_box.hpp"


#include "../include/detection_behavior/detection_behavior_node.h"

using namespace std;
using placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

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

      subscription_sign_data_ = this->create_subscription<darknet_ros_msgs::msg::BoundingBox>(
      "sign_data", 10, std::bind(&detection_behavior::signDataCallback, this, _1));
    
    
      RCLCPP_INFO(this->get_logger(), "detection behavior READY");

      timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&detection_behavior::updateSpeed, this));

    }

  private:

    //Speed variable
    uint8_t us_detect = 0;
    uint8_t lidar_detect = 0;
    string ai_detect = "";

    int last_speed = 60;
    int speed_before_obs = 0;
    int speed_before_stop = 0;
    int current_speed = 60;
    int speed_before_sb = 0;
    int speed_before_yield = 0;

    int counter = 0;

    bool state = false;

    //Publisher
    rclcpp::Publisher<interfaces::msg::RequiredSpeed>::SharedPtr publisher_required_speed_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::Obstacles>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<interfaces::msg::Reaction>::SharedPtr subscription_reaction_;
    rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBox>::SharedPtr subscription_sign_data_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;


    void updateSpeed(){

      auto speedMsg = interfaces::msg::RequiredSpeed();

      if(us_detect == 1){ //Si le Lidar ou les capteurs US détectent un piéton à 50cm
        if(current_speed != 0){
          RCLCPP_INFO(this->get_logger(), "pieton : vitesse de 0");
          speed_before_obs = current_speed;
        }
        current_speed = 0;
      }  else if (state == true && speed_before_obs == 0) {
        if(ai_detect == "stop"){ //Si panneau stop
          if(current_speed != 0 && counter == 0){
            RCLCPP_INFO(this->get_logger(), "panneau stop : vitesse de 0");
            speed_before_stop = current_speed;
          }
          if(counter!=2000){  //être à l'arrêt pendant 2s
            current_speed = 0;
            counter ++;
          }else{
            current_speed = speed_before_stop;
          }
        } else if(ai_detect == "pass"){ //Si panneau cédez-le-passage
          if(current_speed != 20){
            RCLCPP_INFO(this->get_logger(), "panneau cedez le passage : vitesse de 20");
            speed_before_yield = current_speed;
          }
          if(counter!=2000){  //ralentir pendant 2s
            current_speed = 20;
            counter ++;
          }else{
            current_speed = speed_before_yield;
          }
        } else if(ai_detect == "speedbump"){ //Si panneau dos d'âne
          if(current_speed != 30 && counter == 0){
            RCLCPP_INFO(this->get_logger(), "panneau dos ane : vitesse 30");
            speed_before_sb = current_speed;
          }
          if(counter!=2000){  //ralentir pendant 2s
            current_speed = 30;
            counter ++;
          }else{
            current_speed = speed_before_sb;
          }
        } else if (ai_detect == "speed30") { //Si détection de panneau vitesse basse, et pas de panneau de travaux détecté avant
          if(current_speed != 36){
            RCLCPP_INFO(this->get_logger(), "panneau vitesse basse : vitesse 36");
            last_speed = current_speed;
          }
          current_speed = 36;
        } else if (ai_detect == "endspeed") { //Si détection de panneau fin de limitation
            if(last_speed != 0){
              RCLCPP_INFO(this->get_logger(), "panneau fin limitation : vitesse 60");
              current_speed = last_speed;
            }
            last_speed = 0;
        }else if (ai_detect == "speed50") { //Si détection de panneau vitesse haute, et pas de panneau de travaux détecté avant
          last_speed = 0;
          current_speed = 60;
        }  
      } else if (us_detect == 0) {
        if(speed_before_obs != 0){
          current_speed = speed_before_obs;
          speed_before_obs = 0;
          RCLCPP_INFO(this->get_logger(), "pieton : reprise vitesse %i", speed_before_obs);
          }      
      } 
      speedMsg.speed_rpm = current_speed; 
      publisher_required_speed_->publish(speedMsg);
      
    }

    void obsDataCallback(const interfaces::msg::Obstacles & obstacles){

      if (us_detect != obstacles.us_detect || lidar_detect != obstacles.lidar_detect) {
        us_detect = obstacles.us_detect;
        lidar_detect = obstacles.lidar_detect;
      }  
    }

    void signDataCallback(const darknet_ros_msgs::msg::BoundingBox & signData){
      if (ai_detect != signData.class_id) {
        ai_detect = signData.class_id; 
        counter = 0;
      }  
    }


    void reactionCallback(const interfaces::msg::Reaction & reaction) {
      if (state == false && reaction.react == true) {
        counter = 0;
      }
      state = reaction.react;
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
