

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>

#include "interfaces/msg/obstacles.hpp"
#include "interfaces/msg/reaction.hpp"
#include "interfaces/msg/ai_info.hpp"

#include "interfaces/msg/behavior_info.hpp"

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
      publisher_behavior_info_ = this->create_publisher<interfaces::msg::BehaviorInfo>("behavior_info", 10);

      subscription_obstacles_ = this->create_subscription<interfaces::msg::Obstacles>(
        "obstacles", 10, std::bind(&detection_behavior::obsDataCallback, this, _1));
      
      subscription_reaction_ = this->create_subscription<interfaces::msg::Reaction>(
        "reaction", 10, std::bind(&detection_behavior::reactionCallback, this, _1));

      subscription_ai_info_ = this->create_subscription<interfaces::msg::AiInfo>(
      "ai_info", 10, std::bind(&detection_behavior::aiCallback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "detection behavior READY");

      timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&detection_behavior::updateSpeed, this));

    }

  private:

    //Speed variable
    uint8_t us_detect = 0;
    uint8_t lidar_detect = 0;
    uint8_t ai_detect = 0;

    int last_speed = 60;
    int speed_before_obs = 0;
    int speed_before_stop = 0;
    int current_speed = 60;
    int speed_before_sb = 0;
    int speed_before_yield = 0;

    int counter = 0;

    bool start = false;
    bool finished = false;

    //Publisher
    rclcpp::Publisher<interfaces::msg::BehaviorInfo>::SharedPtr publisher_behavior_info_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::Obstacles>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<interfaces::msg::Reaction>::SharedPtr subscription_reaction_;
    rclcpp::Subscription<interfaces::msg::AiInfo>::SharedPtr subscription_ai_info_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;


    void updateSpeed(){

      auto behaviorMsg = interfaces::msg::BehaviorInfo();

      if(us_detect == 1 || lidar_detect == 1){ //Si le Lidar ou les capteurs US détectent un piéton à 50cm
        if(current_speed != 0){
          RCLCPP_INFO(this->get_logger(), "pieton : vitesse de 0");
          speed_before_obs = current_speed;
        }
        current_speed = 0;
      } else if (start == true) {
        if(ai_detect == 1){ //Si panneau stop
          if(current_speed != 0 && counter == 0){
            RCLCPP_INFO(this->get_logger(), "panneau stop : vitesse de 0");
            speed_before_stop = current_speed;
          }
          if(counter!=2000){  //être à l'arrêt pendant 2s
            current_speed = 0;
            counter ++;
          }else{
            current_speed = speed_before_stop;
            finished = true;
          }
        } else if(ai_detect == 2){ //Si panneau cédez-le-passage
          if(current_speed != 20 && counter == 0){
            RCLCPP_INFO(this->get_logger(), "panneau cedez le passage : vitesse de 20");
            speed_before_yield = current_speed;
          }
          if(counter!=2000){  //ralentir pendant 2s
            current_speed = 20;
            counter ++;
          }else{
            current_speed = speed_before_yield;
            finished = true;
          }
        } else if(ai_detect == 3){ //Si panneau dos d'âne
          if(current_speed != 30 && counter == 0){
            RCLCPP_INFO(this->get_logger(), "panneau dos ane : vitesse 30");
            speed_before_sb = current_speed;
          }
          if(counter!=2000){  //ralentir pendant 2s
            current_speed = 30;
            counter ++;
          }else{
            current_speed = speed_before_sb;
            finished = true;
          }
        } else if (ai_detect == 4) { //Si détection de panneau vitesse basse
          if(current_speed != 36){
            RCLCPP_INFO(this->get_logger(), "panneau vitesse basse : vitesse 36");
            last_speed = current_speed;
            current_speed = 36;
            finished = true;
          }
        } else if (ai_detect == 6) { //Si détection de panneau fin de limitation
            if(last_speed != 0){
              RCLCPP_INFO(this->get_logger(), "panneau fin limitation : vitesse 36");
              current_speed = last_speed;
              last_speed = 0;
              finished = true;
            }
        }else if (ai_detect == 7) { //Si détection de panneau vitesse haute
          last_speed = 0;
          current_speed = 60;
          finished = true;
        } else if (ai_detect == 0 && lidar_detect == 0 && us_detect == 0){ //Si rien n'est détecté (situation de départ)
          if(speed_before_obs != 0){
            current_speed = speed_before_obs;
            speed_before_obs = 0;
          } else if(speed_before_stop != 0) {
            current_speed = speed_before_stop;
            speed_before_stop = 0;
          } else if(speed_before_sb != 0){
            current_speed = speed_before_sb;
            speed_before_sb = 0;
          } else if(speed_before_yield != 0){
            current_speed = speed_before_yield;
            speed_before_yield = 0;
          }

        } 
      }  
      behaviorMsg.speed_rpm = current_speed; 
      behaviorMsg.done = finished; 
      publisher_behavior_info_->publish(behaviorMsg);
      
    }

    void obsDataCallback(const interfaces::msg::Obstacles & obstacles){

      if (us_detect != obstacles.us_detect || lidar_detect != obstacles.lidar_detect) {
        us_detect = obstacles.us_detect;
        lidar_detect = obstacles.lidar_detect;
        counter = 0;
      }  
    }

    void reactionCallback(const interfaces::msg::Reaction & reaction) {
      if (start == false && reaction.react == true) {
        counter = 0;
        finished = false;
      }
      start = reaction.react;
    }

    void aiCallback(const interfaces::msg::AiInfo & aiInfo) {
      if (ai_detect != aiInfo.sign_type) {
        ai_detect = aiInfo.sign_type;
      }
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
