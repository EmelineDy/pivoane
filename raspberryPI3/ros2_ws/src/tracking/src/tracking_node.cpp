

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdio>
#include <string>

#include "interfaces/msg/reaction.hpp"
#include "darknet_ros_msgs/msg/close_bounding_boxes.hpp"

#include "../include/tracking/tracking_node.h"

using namespace std;
using placeholders::_1;

class tracking : public rclcpp::Node {
  public:
    tracking()
    : Node("tracking_node")
    {
      publisher_reaction_ = this->create_publisher<interfaces::msg::Reaction>("reaction", 10);
   
     
      subscription_sign_data_ = this->create_subscription<darknet_ros_msgs::msg::CloseBoundingBoxes>(
        "close_bounding_boxes", 1, std::bind(&tracking::signDataCallback, this, _1));
    
      RCLCPP_INFO(this->get_logger(), "tracking_node READY");
    }

  private:

    //Publisher
    rclcpp::Publisher<interfaces::msg::Reaction>::SharedPtr publisher_reaction_;

    //Subscriber
    rclcpp::Subscription<darknet_ros_msgs::msg::CloseBoundingBoxes>::SharedPtr subscription_sign_data_;

    string pastSignType = "";

    //Create counters to track signs detection
    int count_stop = 0;
    int count_30 = 0;
    int count_50 = 0;
    int count_bump = 0;
    int counts[4]= {count_stop, count_30, count_50, count_bump}; 
    string label[4] = {"stop", "speed30", "speed50", "speedbump"};

    // Define a max for the counter, and a threshold
    int max_count = 60;
    int threshold = 40;

    int index = -1;
    int max_detect = 0;

    //Create an array to register which signs is sure to be here
    int sure[4] = {0, 0, 0, 0};
    


    void signDataCallback(const darknet_ros_msgs::msg::CloseBoundingBoxes & signData){

      auto reactMsg = interfaces::msg::Reaction();

      //The AI model sends continuous data, if we detect a sign we increase the corresponding counter, and decrease the others
      if (signData.close_bounding_boxes[0].class_id == "stop") {
        count_stop = min (count_stop + 1, max_count);
        count_30 = max (count_30 - 1, 0);
        count_50 = max (count_50 - 1, 0);
        count_bump = max (count_bump - 1, 0);
      } else if (signData.close_bounding_boxes[0].class_id == "speedbump") {
        count_bump = min (count_bump + 1, max_count);          
        count_30 = max (count_30 - 1, 0);
        count_50 = max (count_50 - 1, 0);
        count_stop = max (count_stop - 1, 0);
      } else if (signData.close_bounding_boxes[0].class_id == "speed30") {
        count_30 = min (count_30 + 1, max_count);
        count_stop = max (count_stop - 1, 0);
        count_50 = max (count_50 - 1, 0);
        count_bump = max (count_bump - 1, 0);
      } else if (signData.close_bounding_boxes[0].class_id == "speed50") {
        count_50 = min (count_50 + 1, max_count);
        count_30 = max (count_30 - 1, 0);
        count_stop = max (count_stop - 1, 0);
        count_bump = max (count_bump - 1, 0);
      } else {
        //If the camera detects no road signs, we decrease every counters
        count_50 = max (count_50 - 1, 0);
        count_30 = max (count_30 - 1, 0);
        count_stop = max (count_stop - 1, 0);
        count_bump = max (count_bump - 1, 0);
      }

      counts[0] = count_stop;
      counts[1] = count_30;
      counts[2] = count_50;
      counts[3] = count_bump;

      // Determine the highest counter
      for (int i = 0; i<4; i++) {
        if (counts[i] > max_detect) {
          max_detect = counts[i];
          index = i;
        }
      }

      //If the highest counter is higher than the fixed threshold, the sign is 'really' here
      if (max_detect >= threshold && sure[index] == 0) {
        sure[index] = 1;
        RCLCPP_INFO(this->get_logger(), "threshold passed for %s", label[j].c_str())
      }

      //When the counter which was at his highest suddenly goes down, the car doesn't see the sign anymore, meaning it's near it and has to react
      for (int j = 0; j<4; j++) {
        if (sure[j] == 1 && counts[j] <= 10) {
          reactMsg.class_id = label[j];
          publisher_reaction_->publish(reactMsg);
          RCLCPP_INFO(this->get_logger(), "React TRUE to %s", label[j].c_str());
          sure[j] = 0;
        }
      }

      //Re initialize the variables
      max_detect = 0;
      index = -1;
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tracking>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
