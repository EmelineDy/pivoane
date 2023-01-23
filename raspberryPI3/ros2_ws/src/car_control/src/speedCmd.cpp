/* This file contains the funciton that calculate the PWM of each wheel according to the speed that needs to be reached*/

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include "../include/car_control/speedCmd.h"

using namespace std;

// Speed variables
float leftPwmCmd ;
float rightPwmCmd;
float speedErrorLeft;
float speedErrorRight;

void calculateSpeed(float cmd_RearSpeed, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd, float currentRPM_L, float currentRPM_R, float& sumIntegralLeft, float& sumIntegralRight){
    
    // Calcul of the error for gain Kp
    speedErrorLeft = cmd_RearSpeed - currentRPM_L;
    speedErrorRight = cmd_RearSpeed - currentRPM_R;

    // Calcul of the error for gain Ki
    sumIntegralLeft += speedErrorLeft;
    sumIntegralRight += speedErrorRight;

    // Calcul of the command to be sent to each motor (right and left)
    leftPwmCmd = min( max(0.0, (speedErrorLeft * 1 + sumIntegralLeft * 0.01)), 50.0);
    rightPwmCmd = min( max(0.0, (speedErrorRight * 1 + sumIntegralRight * 0.01)), 50.0);
    leftPwmCmd += 50;
    rightPwmCmd += 50;

    leftRearPwmCmd = leftPwmCmd;
    rightRearPwmCmd = rightPwmCmd;
}
