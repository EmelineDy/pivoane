

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>

#include "interfaces/msg/obstacles.hpp"
#include "interfaces/msg/required_speed.hpp"


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
    
      RCLCPP_INFO(this->get_logger(), "detection behavior READY");
    }

  private:

    //Speed variable
    uint8_t us_detect;
    uint8_t lidar_detect;
    uint8_t ai_detect;

    int work_sign = 0;
    int last_speed = 60;

    //Publisher
    rclcpp::Publisher<interfaces::msg::RequiredSpeed>::SharedPtr publisher_required_speed_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::Obstacles>::SharedPtr subscription_obstacles_;

    void obsDataCallback(const interfaces::msg::Obstacles & obstacles){
      
      auto speedMsg = interfaces::msg::RequiredSpeed();

      us_detect = obstacles.us_detect;
      lidar_detect = obstacles.lidar_detect;
      ai_detect = obstacles.ai_detect;

      if(us_detect == 2 || lidar_detect == 2){ //Si le Lidar ou les capteurs US détectent un piéton proche
        last_speed = speedMsg.speed_rpm;
        speedMsg.speed_rpm = 0;
      }else if(us_detect == 1 || lidar_detect == 1){ //Si le Lidar ou les capteurs US détectent un piéton loin
        last_speed = speedMsg.speed_rpm;
        speedMsg.speed_rpm = 30;
      } else if (ai_detect == 4 && work_sign == 0) { //Si détection de panneau vitesse basse, et pas de panneau de travaux détecté avant
         speedMsg.speed_rpm = 30;
      } else if (ai_detect == 5) { //Si détection de panneau travaux
         last_speed = speedMsg.speed_rpm;
         speedMsg.speed_rpm = 20;
         work_sign = 1;
      } else if (ai_detect == 6) { //Si détection de panneau fin de limitation
         speedMsg.speed_rpm = last_speed;
         work_sign = 0;
      }else if (ai_detect == 7 && work_sign == 0) { //Si détection de panneau vitesse haute, et pas de panneau de travaux détecté avant
         speedMsg.speed_rpm = 60;
      } else if (ai_detect == 0 && lidar_detect == 0 && us_detect == 0 && work_sign == 0){ //Si rien n'est détecté (situation de départ)
        speedMsg.speed_rpm = 60;
      }
    
      publisher_required_speed_->publish(speedMsg);      
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
