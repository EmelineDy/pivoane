#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/joystick_order.hpp"

#include "std_srvs/srv/empty.hpp"

#include "../include/car_control/steeringCmd.h"
#include "../include/car_control/propulsionCmd.h"
#include "../include/car_control/car_control_node.h"

using namespace std;
using placeholders::_1;


class car_control : public rclcpp::Node {

public:
    car_control()
    : Node("car_control_node")
    {
        start = false;
        mode = 0;
        requestedThrottle = 0;
        requestedSteerAngle = 0;
    

        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);

        

        subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
        "joystick_order", 10, std::bind(&car_control::joystickOrderCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
        "steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));


        

        server_calibration_ = this->create_service<std_srvs::srv::Empty>(
                            "steering_calibration", std::bind(&car_control::steeringCalibration, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_control::updateCmd, this));

        
        RCLCPP_INFO(this->get_logger(), "car_control_node READY");
    }

    
private:

    /* Update start, mode, requestedThrottle, requestedSteerAngle and reverse from joystick order [callback function]  :
    *
    * This function is called when a message is published on the "/joystick_order" topic
    * 
    */

    int compteur = 0;
    int iErrorL = 0;
    int iErrorR = 0;
    float deltaErrorRight;
    float deltaErrorLeft;
    float rpmErrorLeft;
    float rpmErrorRight;
    float previousrpmErrorLeft ;
    float previousrpmErrorRight;

    float previousSpeedErrorLeft;
    float previousSpeedErrorRight;
    float sumIntegralLeft;
    float sumIntegralRight;

    void joystickOrderCallback(const interfaces::msg::JoystickOrder & joyOrder) {

        if (joyOrder.start != start){
            start = joyOrder.start;

            if (start)
                RCLCPP_INFO(this->get_logger(), "START");
            else 
                RCLCPP_INFO(this->get_logger(), "STOP");
        }
        

        if (joyOrder.mode != mode && joyOrder.mode != -1){ //if mode change
            mode = joyOrder.mode;

            if (mode==0){
                RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            }else if (mode==1){
                RCLCPP_INFO(this->get_logger(), "Switching to AUTONOMOUS Mode");
            }else if (mode==2){
                RCLCPP_INFO(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
                startSteeringCalibration();
            }
        }
        
        if (mode == 0 && start){  //if manual mode -> update requestedThrottle, requestedSteerAngle and reverse from joystick order
            requestedThrottle = joyOrder.throttle;
            requestedSteerAngle = joyOrder.steer;
            reverse = joyOrder.reverse;
        }
    }

    /* added functions by team Beth*/

    void speed(float rpm_target) {
        
        //int n_micro_step = 5; 
        //float micro_step_rpm_target = rpm_target/static_cast<float>(n_micro_step);

        //
        RCLCPP_INFO(this->get_logger(), "Begin speed");


        //Calcul de l'erreur pour le gain Kp
        rpmErrorLeft = rpm_target - currentRPM_L;
        rpmErrorRight = rpm_target - currentRPM_R;

        //Calcul de l'erreur pour le gain Ki
        iErrorL += rpmErrorLeft;
        iErrorR += rpmErrorRight;

        //Calcul de l'erreur pour le gain Kd
        deltaErrorLeft = rpmErrorLeft - previousrpmErrorLeft;
        deltaErrorRight = rpmErrorRight - previousrpmErrorRight;
        previousrpmErrorLeft = rpmErrorLeft;
        previousrpmErrorRight = rpmErrorRight;

        //Calcul de la commande à envoyer à chacun des moteurs (gauche et droite)
        leftRearPwmCmd = min(50, max (0, int(rpmErrorLeft * 1 + iErrorL * 0.01)));
        rightRearPwmCmd = min(50, max(0, int(rpmErrorRight * 1 + iErrorR * 0.01)));

        leftRearPwmCmd += 50;
        rightRearPwmCmd += 50;

        RCLCPP_INFO(this->get_logger(), "End speed");


    }


    void go_forward(int speed_goal){
        if(compteur<=2*TIME){
            leftRearPwmCmd = speed_goal;
            rightRearPwmCmd = speed_goal;
            steeringPwmCmd = 50;
            compteur+=1;
        }
        else{
            leftRearPwmCmd = STOP;
            rightRearPwmCmd = STOP;
            steeringPwmCmd = 50;
            compteur = 0;
        }
    }

    void go_backward(){
        if(compteur<=10*TIME){
            leftRearPwmCmd = 25;
            rightRearPwmCmd = 25;
            steeringPwmCmd = 50;
            compteur+=1;
        }
        else{
            leftRearPwmCmd = STOP;
            rightRearPwmCmd = STOP;
            steeringPwmCmd = 50;
            compteur = 0;
        }  
    }

    void accel_decel_stop(){
        
        if(compteur <= 5*TIME){
            speed(60);
            compteur+=1;
        }
        else if((5*TIME < compteur) && (compteur <= 10*TIME)){
            speed(30);
            compteur+=1;            
        }
        else{
            speed(0);
            compteur = 0;
        }     
    }
    /*
    void straight_Traj(float RPM_R, float RPM_L, float rpm_target) {
        int pwm_max = 100;
        float rpm_max_l = 62.169998;
        float rpm_max_r = 61.27;

            float error_l = rpm_target - RPM_L;
            float error_r = rpm_target - RPM_R;

            float correction_l = ((error_l / rpm_max_l) /2) * float(pwm_max)*0.3;
            float correction_r = ((error_r / rpm_max_r) /2) * float(pwm_max)*0.3;

            leftRearPwmCmd = uint8_t(min(float(100), max(float(50), float(leftRearPwmCmd)+correction_l)));
            rightRearPwmCmd = uint8_t(min(float(100), max(float(50), float(rightRearPwmCmd)+correction_r)));
            steeringPwmCmd = 50;
    }  
    */

   
    /* Update currentAngle from motors feedback [callback function]  :
    *
    * This function is called when a message is published on the "/motors_feedback" topic
    * 
    */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
        currentAngle = motorsFeedback.steering_angle;
        currentRPM_R = motorsFeedback.right_rear_speed;
        currentRPM_L = motorsFeedback.left_rear_speed;
    }


    /* Update PWM commands : leftRearPwmCmd, rightRearPwmCmd, steeringPwmCmd
    *
    * This function is called periodically by the timer [see PERIOD_UPDATE_CMD in "car_control_node.h"]
    * 
    * In MANUAL mode, the commands depends on :
    * - requestedThrottle, reverse, requestedSteerAngle [from joystick orders]
    * - currentAngle [from motors feedback]
    */
    void updateCmd(){

        auto motorsOrder = interfaces::msg::MotorsOrder();

        float leftPwmCmd;
        float rightPwmCmd;
        float speedErrorLeft;
        float speedErrorRight;

        if (!start){    //Car stopped
            leftRearPwmCmd = STOP;
            rightRearPwmCmd = STOP;
            steeringPwmCmd = STOP;

        }else{ //Car started

            //Manual Mode
            if (mode==0){
                RCLCPP_INFO(this->get_logger(), "Manual mode");
                manualPropulsionCmd(requestedThrottle, reverse, leftRearPwmCmd,rightRearPwmCmd);
                steeringCmd(requestedSteerAngle,currentAngle, steeringPwmCmd);

                compteur = 0;

            //Autonomous Mode
            } else if (mode==1){
                RCLCPP_INFO(this->get_logger(), "Autonomous mode");
                //speed(40);

                float cmd_RearSpeed = 40;
                //Calcul de l'erreur pour le gain Kp
                speedErrorLeft = cmd_RearSpeed - currentRPM_L;
                speedErrorRight = cmd_RearSpeed - currentRPM_R;

                //Calcul de l'erreur pour le gain Ki
		        sumIntegralLeft += speedErrorLeft;
                sumIntegralRight += speedErrorRight;

                //Calcul de l'erreur pour le gain Kd
                deltaErrorLeft = speedErrorLeft - previousSpeedErrorLeft;
                deltaErrorRight = speedErrorRight - previousSpeedErrorRight;
                previousSpeedErrorLeft = speedErrorLeft;
                previousSpeedErrorRight = speedErrorRight;

                //Calcul de la commande à envoyer à chacun des moteurs (gauche et droite)
                leftPwmCmd = speedErrorLeft * 1 + sumIntegralLeft * 0.01;
                rightPwmCmd = speedErrorRight * 1 + sumIntegralRight * 0.01;

                //Pour éviter de casser le moteur,
                // on évite de le retour en arrière du moteur en empêchant une commande < 50
                if(leftPwmCmd < 0)
                    leftPwmCmd = 0;
                else if(leftPwmCmd > 50)
                    leftPwmCmd = 50;

                if(rightPwmCmd < 0)
                    rightPwmCmd = 0;
                else if(rightPwmCmd > 50)
                    rightPwmCmd = 50;

                leftPwmCmd += 50;
                rightPwmCmd += 50;


                leftRearPwmCmd = leftPwmCmd;
                rightRearPwmCmd = rightPwmCmd;
            }

        }
        

        //Send order to motors
        motorsOrder.left_rear_pwm = leftRearPwmCmd;
        motorsOrder.right_rear_pwm = rightRearPwmCmd;
        motorsOrder.steering_pwm = steeringPwmCmd;

        publisher_can_->publish(motorsOrder);

    }


    /* Start the steering calibration process :
    *
    * Publish a calibration request on the "/steering_calibration" topic
    */
    void startSteeringCalibration(){

        auto calibrationMsg = interfaces::msg::SteeringCalibration();
        calibrationMsg.request = true;

        RCLCPP_INFO(this->get_logger(), "Sending calibration request .....");
        publisher_steeringCalibration_->publish(calibrationMsg);
    }


    /* Function called by "steering_calibration" service
    * 1. Switch to calibration mode
    * 2. Call startSteeringCalibration function
    */
    void steeringCalibration([[maybe_unused]] std_srvs::srv::Empty::Request::SharedPtr req,
                            [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr res)
    {

        mode = 2;    //Switch to calibration mode
        RCLCPP_WARN(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
        startSteeringCalibration();
    }
    

    /* Manage steering calibration process [callback function]  :
    *
    * This function is called when a message is published on the "/steering_calibration" topic
    */
    void steeringCalibrationCallback (const interfaces::msg::SteeringCalibration & calibrationMsg){

        if (calibrationMsg.in_progress == true && calibrationMsg.user_need == false){
        RCLCPP_INFO(this->get_logger(), "Steering Calibration in progress, please wait ....");

        } else if (calibrationMsg.in_progress == true && calibrationMsg.user_need == true){
            RCLCPP_WARN(this->get_logger(), "Please use the buttons (L/R) to center the steering wheels.\nThen, press the blue button on the NucleoF103 to continue");
        
        } else if (calibrationMsg.status == 1){
            RCLCPP_INFO(this->get_logger(), "Steering calibration [SUCCESS]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        
        } else if (calibrationMsg.status == -1){
            RCLCPP_ERROR(this->get_logger(), "Steering calibration [FAILED]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        }
    
    }
    
    // ---- Private variables ----

    //General variables
    bool start;
    int mode;    //0 : Manual    1 : Auto    2 : Calibration

    
    //Motors feedback variables
    float currentAngle;
    float currentRPM_R;
    float currentRPM_L;

    //Manual Mode variables (with joystick control)
    bool reverse;
    float requestedThrottle;
    float requestedSteerAngle;

    //Control variables
    uint8_t leftRearPwmCmd;
    uint8_t rightRearPwmCmd;
    uint8_t steeringPwmCmd;

    //Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steering_calibration_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Steering calibration Service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_calibration_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_control>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}