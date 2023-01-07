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
    
      RCLCPP_INFO(this->get_logger(), "us_detection_node READY");
    }

  private:
    int a = 0;
    int nb_warning = 0;

    //Speed variable
    uint8_t last_us_detect = 0;

    //Publisher
    rclcpp::Publisher<interfaces::msg::Obstacles>::SharedPtr publisher_obstacle_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_ultrasonic_sensor_;

    void usDataCallback(const interfaces::msg::Ultrasonic & ultrasonic){
      
      auto obstacleMsg = interfaces::msg::Obstacles();

      //WARNING if strange value of the us_data (too far or negative value)
      if(ultrasonic.front_center > 600 || ultrasonic.front_left > 600 || ultrasonic.front_right > 600 || ultrasonic.front_center < 0 || ultrasonic.front_left < 0 || ultrasonic.front_right < 0){
        RCLCPP_WARN(this->get_logger(), "Warning : wrong us data");
        nb_warning += 1;
      }

      //ERROR if strange value of the us_data (too far or negative value) 5 times in a row
      else if(nb_warning >= 5){
        RCLCPP_ERROR(this->get_logger(), "Error : wrong us data for too long");
        obstacleMsg.us_detect = 2; //we add the case when there is a problem with the us sensors
      }else{
        nb_warning = 0;
        if ((ultrasonic.front_center <= 50.0)){
        obstacleMsg.us_detect = 1;
        } 
        else if((ultrasonic.front_left <= 20.0)){
          obstacleMsg.us_detect = 1;
        } 
        else if((ultrasonic.front_right <= 20.0)){
          obstacleMsg.us_detect = 1; 
        }
        else{
          obstacleMsg.us_detect = 0;
        }

        if (last_us_detect != obstacleMsg.us_detect) {   
          last_us_detect = obstacleMsg.us_detect; 
          publisher_obstacle_->publish(obstacleMsg);
        }
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
