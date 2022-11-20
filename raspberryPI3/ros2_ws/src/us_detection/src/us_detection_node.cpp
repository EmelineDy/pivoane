

#include "rclcpp/rclcpp.hpp"
#include <cstdio>

#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/obstacles.hpp"


#include "../include/us_detection/us_detection_node.h"

using namespace std;
using placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class us_detection : public rclcpp::Node {
  public:
    us_detection()
    : Node("us_detection_node")
    {
      publisher_obstacle_ = this->create_publisher<interfaces::msg::Obstacles>("Obstacles", 10);

      subscription_ultrasonic_sensor_ = this->create_subscription<interfaces::msg::Ultrasonic>(
        "us_data", 10, std::bind(&us_detection::usDataCallback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "car_control_node READY");
    }

  private:
    int a = 0;

    //Ultrasonic feedback variables
    float LeftObstacle;
    float RightObstacle;
    float CenterObstacle;

    //Speed variable
    uint8_t speed_order;

    rclcpp::TimerBase::SharedPtr timer_;

    //Publisher
    rclcpp::Publisher<interfaces::msg::Obstacles>::SharedPtr publisher_obstacle_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_ultrasonic_sensor_;


  void usDataCallback(const interfaces::msg::Ultrasonic & ultrasonic){
    
    auto ObstaclesMsg = interfaces::msg::Obstacles();

    if ((ultrasonic.front_center <= 50.0)){
      if(a!=1){
          RCLCPP_INFO(this->get_logger(), "Front obstacle near = %d cm", ultrasonic.front_center);
          a = 1;
      }
      ObstaclesMsg.speed_order = 0;
    } else if((ultrasonic.front_left <= 20.0)){
        if(a!=4){
            RCLCPP_INFO(this->get_logger(), "Obstacle on the left = %d cm", ultrasonic.front_left);
            a = 4;
        }
      ObstaclesMsg.speed_order = 0;
    } else if((ultrasonic.front_right <= 20.0)){
      RCLCPP_INFO(this->get_logger(), "Obstacle on the right = %d cm", ultrasonic.front_right);
      a = 5;
      ObstaclesMsg.speed_order = 0; 
    } else if(ultrasonic.front_center > 50.0 && ultrasonic.front_center <= 100.0){
      if(a!=2){
          RCLCPP_INFO(this->get_logger(), "Front obstacle far = %d cm", ultrasonic.front_center);
          a = 2;
      }
      ObstaclesMsg.speed_order = 1;
    } else{
      if(a!=3){
          RCLCPP_INFO(this->get_logger(), "No obstacle");
          a = 3;
      }
      ObstaclesMsg.speed_order = 2;
    }
    //Obstacles.speed_order = speed_order;

    RCLCPP_INFO(this->get_logger(), "Speed order = %d ", ObstaclesMsg.speed_order);
    publisher_obstacle_->publish(ObstaclesMsg);
  }

  /*
  void Object_detection() {

    auto Obstacles = interfaces::msg::Obstacles();

    
    if ((CenterObstacle <= 50.0)){
      if(a!=1){
          RCLCPP_INFO(this->get_logger(), "Front obstacle near = %f cm", CenterObstacle);
          a = 1;
      }
      
      RCLCPP_INFO(this->get_logger(), "Front obstacle near = %f cm", CenterObstacle);
      speed_order = 0;
    }

  //obstacle à gauche à moins de 20 cm

    else if((LeftObstacle <= 20.0)){
        if(a!=4){
            RCLCPP_INFO(this->get_logger(), "Obstacle on the left = %f cm", LeftObstacle);
            a = 4;
        }
      speed_order = 0;
      RCLCPP_INFO(this->get_logger(), "Obstacle on the left = %f cm", LeftObstacle);
    }

  //obstacle à droite à moins de 20 cm

    else if((RightObstacle <= 20.0)){
      RCLCPP_INFO(this->get_logger(), "Obstacle on the right = %f cm", RightObstacle);
      a = 5;
      speed_order = 0; 
    }

  //obstacle au centre entre 50cm et 1m : half speed   

    else if(CenterObstacle > 50.0 && CenterObstacle <= 100.0){
      if(a!=2){
          RCLCPP_INFO(this->get_logger(), "Front obstacle far = %f cm", CenterObstacle);
          a = 2;
      }
      RCLCPP_INFO(this->get_logger(), "Front obstacle far = %f cm", CenterObstacle);
      speed_order = 1;
    }

  //pas d'obstacle à moins d'1m

    else{
      if(a!=3){
          RCLCPP_INFO(this->get_logger(), "No obstacle");
          a = 3;
      }
      RCLCPP_INFO(this->get_logger(), "No obstacle");
      speed_order = 2;
    }
    

    //RCLCPP_INFO(this->get_logger(), "a = %i", a);

  //Obstacles.speed_order = speed_order;

  Obstacles.speed_order = 2;

  publisher_->publish(Obstacles);

  }
  */
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<us_detection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
