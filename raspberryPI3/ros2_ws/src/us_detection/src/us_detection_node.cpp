

#include "rclcpp/rclcpp.hpp"

#include <chrono>
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
      publisher_obstacle_ = this->create_publisher<interfaces::msg::Obstacles>("obstacles", 10);

      subscription_ultrasonic_sensor_ = this->create_subscription<interfaces::msg::Ultrasonic>(
        "us_data", 10, std::bind(&us_detection::usDataCallback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "car_control_node READY");
    }

  private:
    int a = 0;

    //Speed variable
    uint8_t speed_order;

    //Publisher
    rclcpp::Publisher<interfaces::msg::Obstacles>::SharedPtr publisher_obstacle_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_ultrasonic_sensor_;

    void usDataCallback(const interfaces::msg::Ultrasonic & ultrasonic){
      
      auto obstacleMsg = interfaces::msg::Obstacles();

      if ((ultrasonic.front_center <= 50.0)){
        if(a!=1){
          RCLCPP_INFO(this->get_logger(), "Front obstacle near = %d cm", ultrasonic.front_center);
          a = 1;
        }
        obstacleMsg.speed_order = 0;
      } 
      else if((ultrasonic.front_left <= 20.0)){
          if(a!=4){
            RCLCPP_INFO(this->get_logger(), "Obstacle on the left = %d cm", ultrasonic.front_left);
            a = 4;
          }
        obstacleMsg.speed_order = 0;
      } 
      else if((ultrasonic.front_right <= 20.0)){
        if (a!=5){
          RCLCPP_INFO(this->get_logger(), "Obstacle on the right = %d cm", ultrasonic.front_right);
          a = 5;
        }
        obstacleMsg.speed_order = 0; 
      } 
      else if(ultrasonic.front_center > 50.0 && ultrasonic.front_center <= 100.0){
        if(a!=2){
          RCLCPP_INFO(this->get_logger(), "Front obstacle far = %d cm", ultrasonic.front_center);
          a = 2;
        }
        obstacleMsg.speed_order = 1;
      } 
      else{
        if(a!=3){
          RCLCPP_INFO(this->get_logger(), "No obstacle");
          a = 3;
        }
        obstacleMsg.speed_order = 2;
      }
    
      publisher_obstacle_->publish(obstacleMsg);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<us_detection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
