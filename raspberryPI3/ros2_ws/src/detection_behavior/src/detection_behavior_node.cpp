#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>
#include <string>

#include "interfaces/msg/obstacles.hpp"
#include "interfaces/msg/required_speed.hpp"
#include "interfaces/msg/reaction.hpp"
#include "interfaces/msg/finish.hpp"
#include "interfaces/msg/sign_data.hpp"


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

      publisher_finish_ = this->create_publisher<interfaces::msg::Finish>("finish", 10);

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

    int last_speed = 60;
    int speed_before_obs = 0;
    int speed_before_stop = 0;
    int current_speed = 60;
    int speed_before_sb = 0;
    int speed_before_yield = 0;

    int counter = 0;

    bool start_reacting = false;
    bool finished_reacting = true;

    //Publisher
    rclcpp::Publisher<interfaces::msg::RequiredSpeed>::SharedPtr publisher_required_speed_;
    rclcpp::Publisher<interfaces::msg::Finish>::SharedPtr publisher_finish_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::Obstacles>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<interfaces::msg::Reaction>::SharedPtr subscription_reaction_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;


    void updateSpeed(){

      auto speedMsg = interfaces::msg::RequiredSpeed();
      auto finishMsg = interfaces::msg::Finish();

      if(us_detect == 2){ // Si les ultrasons renvoient une valeur etrange
        last_speed = current_speed;
        current_speed = 0;
      }
      else if(us_detect == 1){ //Si les capteurs US détectent un piéton à 50cm
        if(current_speed != 0){
          RCLCPP_INFO(this->get_logger(), "pieton : vitesse de 0");
          speed_before_obs = current_speed;
        }
        current_speed = 0;
      }  else if (start_reacting == true && speed_before_obs == 0) {
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
            finishMsg.ended = true;
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
            finishMsg.ended = true;
          }
        } else if (ai_detect == "speed30" && current_speed != 36) { //Si détection de panneau vitesse basse
            RCLCPP_INFO(this->get_logger(), "panneau vitesse basse : vitesse 36");
            last_speed = current_speed;
            current_speed = 36;
            finishMsg.ended = true;
        } else if (ai_detect == "speed50" && current_speed != 60) { //Si détection de panneau vitesse haute
          RCLCPP_INFO(this->get_logger(), "panneau vitesse haute : vitesse 60");
          last_speed = 0;
          current_speed = 60;
          finishMsg.ended = true;
        }  
      } else if (us_detect == 0 && speed_before_obs != 0) {
        current_speed = speed_before_obs;
        speed_before_obs = 0;
        RCLCPP_INFO(this->get_logger(), "pieton : reprise vitesse %i", speed_before_obs);    
      } 
      speedMsg.speed_rpm = current_speed; 
      publisher_required_speed_->publish(speedMsg);

      if (finishMsg.ended != finished_reacting) {
        finished_reacting = finishMsg.ended;
        publisher_finish_->publish(finishMsg);
      }
      
    }

    void obsDataCallback(const interfaces::msg::Obstacles & obstacles){

      if (us_detect != obstacles.us_detect) {
        us_detect = obstacles.us_detect;
      }  
    }

    void reactionCallback(const interfaces::msg::Reaction & reaction) {
      if (start_reacting == false && reaction.react == true) {
        counter = 0;
        finished_reacting = false;
        ai_detect = reaction.class_id; 
      }
      start_reacting = reaction.react;
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
