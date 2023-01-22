/* This file contains the code that pusblishes the messages depending on the us sensors data */

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

    //Counter of warnings
    int nb_warning = 0;

    //Speed variable
    uint8_t last_us_detect = 0;

    //Publisher
    rclcpp::Publisher<interfaces::msg::Obstacles>::SharedPtr publisher_obstacle_;

    //Subscriber
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_ultrasonic_sensor_;

    void usDataCallback(const interfaces::msg::Ultrasonic & ultrasonic){
      
      auto obstacleMsg = interfaces::msg::Obstacles();

      // Checking that the data sent by us sensors are normal
      if(ultrasonic.front_center < 600 && ultrasonic.front_left < 600 && ultrasonic.front_right < 600 && ultrasonic.front_center > 0 && ultrasonic.front_left > 0 && ultrasonic.front_right > 0){
        nb_warning = 0;
        // Message of obstacle if there is one in front of the car at less than 75 cm
        if ((ultrasonic.front_center <= 75.0)){
        obstacleMsg.us_detect = 1;
        } 
        // Message of obstacle if there is one in at the left of the car at less than 20 cm
        else if((ultrasonic.front_left <= 20.0)){
          obstacleMsg.us_detect = 1;
        } 
        // Message of obstacle if there is one in at the right of the car at less than 20 cm
        else if((ultrasonic.front_right <= 20.0)){
          obstacleMsg.us_detect = 1; 
        }
        // No message of obstacle if none of those cases
        else{
          obstacleMsg.us_detect = 0;
        }
        // Changing last value of us_detect and publishing message of obstacle
        if (last_us_detect != obstacleMsg.us_detect) {   
          last_us_detect = obstacleMsg.us_detect; 
          publisher_obstacle_->publish(obstacleMsg);
        }
 
      } else if(nb_warning >= 5){
        //ERROR if strange value of the us_data (too far or negative value) 5 times in a row
        RCLCPP_ERROR(this->get_logger(), "Error : wrong us data for too long");
        // Adding the message when there is a problem with the us sensors
        obstacleMsg.us_detect = 2;
      } else{
        //WARNING if strange value of the us_data (too far or negative value)
        RCLCPP_WARN(this->get_logger(), "Warning : wrong us data");
        nb_warning += 1;
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
