#include "rclcpp/rclcpp.hpp"
#include <functional>
#include "../include/car_control/speedCmd.h"

using namespace std;

float leftPwmCmd ;
float rightPwmCmd;
float speedErrorLeft;
float speedErrorRight;

void calculateSpeed(float cmd_RearSpeed, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd, float currentRPM_L, float currentRPM_R, float& sumIntegralLeft, float& sumIntegralRight){
    
    //Calcul de l'erreur pour le gain Kp
    speedErrorLeft = cmd_RearSpeed - currentRPM_L;
    speedErrorRight = cmd_RearSpeed - currentRPM_R;

    //Calcul de l'erreur pour le gain Ki
    sumIntegralLeft += speedErrorLeft;
    sumIntegralRight += speedErrorRight;

    //Calcul de l'erreur pour le gain Kd
    /*
    deltaErrorLeft = speedErrorLeft - previousSpeedErrorLeft;
    deltaErrorRight = speedErrorRight - previousSpeedErrorRight;
    previousSpeedErrorLeft = speedErrorLeft;
    previousSpeedErrorRight = speedErrorRight;
    */

    //Calcul de la commande à envoyer à chacun des moteurs (gauche et droite)
    leftPwmCmd = min( max(0.0, (speedErrorLeft * 1 + sumIntegralLeft * 0.01)), 50.0);
    rightPwmCmd = min( max(0.0, (speedErrorRight * 1 + sumIntegralRight * 0.01)), 50.0);
    leftPwmCmd += 50;
    rightPwmCmd += 50;


    leftRearPwmCmd = leftPwmCmd;
    rightRearPwmCmd = rightPwmCmd;
}

void adaptSpeed(uint8_t speedRequest, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd, float currentRPM_L, float currentRPM_R, float& sumIntegralLeft, float& sumIntegralRight){
    if ((speedRequest == 0)){
        calculateSpeed(0.0, leftRearPwmCmd, rightRearPwmCmd, currentRPM_L, currentRPM_R, sumIntegralLeft, sumIntegralRight);
    }
    else if((speedRequest == 1)){
        calculateSpeed(30.0, leftRearPwmCmd, rightRearPwmCmd, currentRPM_L, currentRPM_R, sumIntegralLeft, sumIntegralRight);
    }
    else if((speedRequest == 2)){
        calculateSpeed(60.0, leftRearPwmCmd, rightRearPwmCmd, currentRPM_L, currentRPM_R, sumIntegralLeft, sumIntegralRight); 
    }
}