#include "rclcpp/rclcpp.hpp"
#include <unistd.h>
#include <functional>
#include <memory>
#include <string>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
//#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "interfaces/msg/system_check.hpp"

#include "interfaces/msg/gnss.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/general_data.hpp"

#include "../include/system_check/system_check_node.hpp"

using namespace std;
using placeholders::_1;


class system_check : public rclcpp::Node {

public:
    system_check()
    : Node("system_check_node")
    {
    

        publisher_system_check_= this->create_publisher<interfaces::msg::SystemCheck>("system_check", 10);
        


        subscription_system_check_ = this->create_subscription<interfaces::msg::SystemCheck>(
        "system_check", 10, std::bind(&system_check::systemCheckCallback, this, _1));

        subscription_us_ = this->create_subscription<interfaces::msg::Ultrasonic>(
        "us_data", 10, std::bind(&system_check::usCallback, this, _1));

        subscription_general_data_ = this->create_subscription<interfaces::msg::GeneralData>(
        "general_data", 10, std::bind(&system_check::generalDataCallback, this, _1));

        subscription_gnss_ = this->create_subscription<interfaces::msg::Gnss>(
        "gnss_data", 10, std::bind(&system_check::gnssCallback, this, _1));

        subscription_imu_raw_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10, std::bind(&system_check::imuRawCallback, this, _1));

        subscription_imu_mag_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "imu/mag", 10, std::bind(&system_check::imuMagCallback, this, _1));

        //subscription_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
        //"image_raw", 10, std::bind(&system_check::cameraCallback, this, _1));

        subscription_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&system_check::lidarCallback, this, _1));


        timer_update_checking_ = this->create_wall_timer(PERIOD_UPDATE_CHECKING, std::bind(&system_check::systemCheck, this));
        timer_seconds_ = this->create_wall_timer(1000ms, std::bind(&system_check::incrementTime, this));

        sleep(TIME_BEFORE_REPORT);   //Wait for all nodes to start
    }


private:
    void incrementTime(){
        seconds++;

        if (seconds>=CHECK_TIMEOUT){
            timeout=true;
        }
    }

    void resetTime(){
        seconds=0;
    }

    void resetCheckingVariables(){
        //Communications
        commJetson = false;
        commL476 = false;
        commF103 = false;

        //Sensors
        for (int i=0;i<6;i++){
            usDistance[i]=-1;
        }

        gpsDetect = false;
        usDetect = false;
        lidarDetect = false;
        cameraDetect = false;
        batteryDetect = false;
    }

    /* Print a report in the launch terminal
    *  Called when the node start OR when the DPAD Left buuton is pressed (on the joystick)
    */
    void printReport(){

        RCLCPP_WARN(this->get_logger(), "----- System check report -----\n");

        if (batteryReport == "OK") RCLCPP_INFO(this->get_logger(), "BATTERY : %.2fV [OK]\n", batteryLevel);
        else if (batteryReport == "No data") RCLCPP_ERROR(this->get_logger(), "BATTERY [No data received]\n");
        else RCLCPP_ERROR(this->get_logger(), "BATTERY : : %.2fV [%s]\n",batteryLevel,batteryReport.c_str());

        RCLCPP_WARN(this->get_logger(), "Communications status :");
        if (commJetsonReport == "OK")
            RCLCPP_INFO(this->get_logger(), "JETSON [OK]");
        else
            RCLCPP_FATAL(this->get_logger(), "JETSON [FAILED]");
        
        if (commF103Report == "OK")
            RCLCPP_INFO(this->get_logger(), "NucleoF103 [OK]");
        else
            RCLCPP_FATAL(this->get_logger(), "NucleoF103 [FAILED]");

        if (commL476Report == "OK")
            RCLCPP_INFO(this->get_logger(), "NucleoL476 [OK]\n");
        else
            RCLCPP_FATAL(this->get_logger(), "NucleoL476 [FAILED]\n");

        
        RCLCPP_WARN(this->get_logger(), "Sensors status :");

        //Ultrasonics
        if (usReport[0]=="OK" && usReport[1]=="OK" && usReport[2]=="OK" && usReport[3]=="OK" && usReport[4]=="OK" && usReport[5]=="OK")
            RCLCPP_INFO(this->get_logger(), "US [OK]");

        else if (usReport[0]=="No data" && usReport[1]=="No data" && usReport[2]=="No data" 
                && usReport[3]=="No data" && usReport[4]=="No data" && usReport[5]=="No data")
        {

            RCLCPP_ERROR(this->get_logger(), "US [No data received]");
        }else {

            if (usReport[0] == "OK") RCLCPP_INFO(this->get_logger(), "US - Front left [OK]");
            else RCLCPP_ERROR(this->get_logger(), "US - Front left [Out of range]");

            if (usReport[1] == "OK") RCLCPP_INFO(this->get_logger(), "US - Front center [OK]");
            else RCLCPP_ERROR(this->get_logger(), "US - Front center [Out of range]");

            if (usReport[2] == "OK") RCLCPP_INFO(this->get_logger(), "US - Front right [OK]");
            else RCLCPP_ERROR(this->get_logger(), "US - Front right [Out of range]");

            if (usReport[3] == "OK") RCLCPP_INFO(this->get_logger(), "US - Rear left [OK]");
            else RCLCPP_ERROR(this->get_logger(), "US - Rear left [Out of range]");

            if (usReport[4] == "OK") RCLCPP_INFO(this->get_logger(), "US - Rear center [OK]");
            else RCLCPP_ERROR(this->get_logger(), "US - Rear center [Out of range]");

            if (usReport[5] == "OK") RCLCPP_INFO(this->get_logger(), "US - Rear right [OK]");
            else RCLCPP_ERROR(this->get_logger(), "US - Rear right [Out of range]");
        }


        //GPS
        if (gpsReport == "No fix")
            RCLCPP_ERROR(this->get_logger(), "GPS [No Fix]");
        else if (gpsReport == "Autonomous GNSS fix")
            RCLCPP_WARN(this->get_logger(), "GPS [Autonomous GNSS fix]");
        else if (gpsReport == "Differential GNSS fix")
            RCLCPP_WARN(this->get_logger(), "GPS [Differential GNSS fix]");
        else if (gpsReport == "GPS [RTK fixed]")
            RCLCPP_INFO(this->get_logger(), "GPS [RTK fixed]");
        else if (gpsReport == "RTK float")
            RCLCPP_INFO(this->get_logger(), "GPS [RTK float]");
        else if (gpsReport == "Estimated/dead reckoning fix")
            RCLCPP_WARN(this->get_logger(), "GPS [Estimated/dead reckoning fix]");
        else if (gpsReport == "No data")
            RCLCPP_ERROR(this->get_logger(), "GPS [No data received]");

        //IMU
        if (imuReport == "OK") RCLCPP_INFO(this->get_logger(), "IMU [OK]");
        else RCLCPP_ERROR(this->get_logger(), "IMU [No data received]");


        //Lidar
        if (lidarReport == "OK") RCLCPP_INFO(this->get_logger(), "LIDAR [OK]");
        else RCLCPP_ERROR(this->get_logger(), "LIDAR [No data received]");

        //Camera
        if (cameraReport == "OK") RCLCPP_INFO(this->get_logger(), "CAMERA [OK]\n");
        else RCLCPP_ERROR(this->get_logger(), "CAMERA [No data received]\n");


        RCLCPP_WARN(this->get_logger(), "----- End of report -----\n");
    }

    /* Publish a report message on the /system_check topic [callback function]
    *  Called at the end of each system check (see "systemCheck" function)
    */
    void sendReport(){
        auto reportMsg = interfaces::msg::SystemCheck();
        reportMsg.report = true;
        reportMsg.comm_jetson = commJetsonReport;
        reportMsg.comm_l476 = commL476Report;
        reportMsg.comm_f103 = commF103Report;
        reportMsg.battery = batteryReport;

        for (int i=0 ; i<6 ; i++)
            reportMsg.ultrasonics[i] = usReport[i]; 
        
        reportMsg.imu = imuReport;
        reportMsg.gps = gpsReport;
        reportMsg.lidar = lidarReport;
        reportMsg.camera = cameraReport;

        publisher_system_check_ -> publish(reportMsg); //Publish report message
    }

    void checkCommunications(){
        if (commF103) commF103Report = "OK";
        else commF103Report = "Failed";

        if (commJetson) commJetsonReport = "OK";
        else commJetsonReport = "Failed";

        if (commL476) commL476Report = "OK";
        else commL476Report = "Failed";
    }


    void checkBattery(){

        if (batteryDetect){
            if (batteryLevel<=11) {
                batteryReport = "Low Voltage";
            }else if (batteryLevel>=14){
                batteryReport = "High Voltage";
            }else {
                batteryReport = "OK";
            }
        } else{
            batteryReport = "No data";
        }
    }

    void checkSensors(){
        if (usDetect){

            if (usDistance[0] > 536) {
                usReport[0] = "Out of range";

            }else usReport[0] = "OK";

            if (usDistance[1] > 536){
                usReport[1] = "Out of range";
        
            } else usReport[1] = "OK";

            if (usDistance[2] > 536){
                usReport[2] = "Out of range";
        
            } else usReport[2] = "OK";

            if (usDistance[3] > 536){
                usReport[3] = "Out of range";
  
            } else usReport[3] = "OK";

            if (usDistance[4] > 536){
                usReport[4] = "Out of range";
        
            } else usReport[4] = "OK";

            if (usDistance[5] > 536){
                usReport[5] = "Out of range";
    

            } else usReport[5] = "OK";
            
            
        } else {
            for (int i=0 ; i<6 ; i++) usReport[i] = "No data";
        }

        if (gpsQuality>=GPS_NO_FIX){

            if (gpsQuality == GPS_NO_FIX)
                gpsReport = "No fix";
 
            else if (gpsQuality == AUTO_GNSS_FIX)
                gpsReport = "Autonomous GNSS fix";
    
            else if (gpsQuality == DIFF_GNSS_FIX)
                gpsReport = "Differential GNSS fix";
      
            else if (gpsQuality == RTK_FIX)
                gpsReport = "GPS [RTK fixed]";
             
            else if (gpsQuality == RTK_FLOAT)
                gpsReport = "RTK float";
          
            else if (gpsQuality == RECKONING_FIX)
                gpsReport = "Estimated/dead reckoning fix";
             

        } else{
            gpsReport = "No data";
        }

        if (imuRawDetect && imuMagDetect)
            imuReport = "OK";
        else
            imuReport = "No data";
        
        

        if (lidarDetect)
            lidarReport = "OK";
        else
            lidarReport = "No data";
      
        

        if (cameraDetect)
            cameraReport = "OK";
        else
            cameraReport = "No data";
        
        

    }


    /* Check the system, send a report, and print a review (only one time, when the node start) [PERIODIC function]
    *  This function is called periodically by the timer "timer_update_checking_"
    */
    void systemCheck(){
        
        if (startChecking){

            if (nodeStart) {
                RCLCPP_WARN(this->get_logger(), "System check in progress ........\n");
            }
            
            resetCheckingVariables();

            //Send communication checking request 

            auto commMsg = interfaces::msg::SystemCheck();
            commMsg.request = true;
            publisher_system_check_ -> publish(commMsg);

            
            checkingInProgress = true;
            startChecking = false;

        }else if (checkingInProgress){

            if (timeout || (commF103 && commJetson && commL476 && usDetect && (gpsQuality==RTK_FIX) && lidarDetect && cameraDetect && batteryDetect)){

                checkBattery();

                checkCommunications();

                checkSensors();

                if (nodeStart){
                    printReport();  //Print a report at node startup 
                    nodeStart = false;
                }               

                
              
                sendReport();

                timeout=false;
                checkingInProgress = false;
                startChecking = true;
                resetTime();
            }
        }
    }

    /* This function is called when a message is received on the /system_check topic [Callback function]
    *   -> If the message is a communication ACK, it updates the corresponding variables
    *   -> If the message is a print request, it calls the "printReport" function
    */
    void systemCheckCallback(const interfaces::msg::SystemCheck & commMsg) {
        
        if (checkingInProgress && commMsg.response){
            if (commMsg.jetson) commJetson = true;
            if (commMsg.l476) commL476 = true;
            if (commMsg.f103) commF103 = true;
        }

        if (commMsg.print) 
            printReport();
        

    }

    

    /* Update gpsQuality from /gnss_data [callback function]
    * 
    */
    void gnssCallback(const interfaces::msg::Gnss & gnssMsg){

        if (checkingInProgress){
            gpsQuality = gnssMsg.quality;
        }
    }

    /* Update imurawDetect from /imu/raw_data [callback function]
    * 
    */
    void imuRawCallback([[maybe_unused]] const sensor_msgs::msg::Imu & imuRawMsg){

        if (checkingInProgress && !imuRawDetect){
            imuRawDetect = true;
        }
    }



    /* Update imuMagDetect from /imu/mag [callback function]
    * 
    */
    void imuMagCallback([[maybe_unused]] const sensor_msgs::msg::MagneticField & imuMagMsg){

        if (checkingInProgress && !imuMagDetect){
            imuMagDetect = true;
        }
    }

    /* Update usDetect and usDistance from /us_data [callback function]  :
    * 
    */
    void usCallback(const interfaces::msg::Ultrasonic & usMsg){

        if (checkingInProgress){
            usDetect = true;

            usDistance[0] = usMsg.front_left;
            usDistance[1] = usMsg.front_center;
            usDistance[2] = usMsg.front_right;
            usDistance[3] = usMsg.rear_left;
            usDistance[4] = usMsg.rear_center;
            usDistance[5] = usMsg.rear_right;

        }
    }


    /* Update batteryDetect and batteryLevel from /general_data [callback function]
    * 
    */
    void generalDataCallback(const interfaces::msg::GeneralData & generalMsg){
        
        if (checkingInProgress){
            
            batteryDetect = true;
            batteryLevel = generalMsg.battery_level;
        }
    }

    /* Update lidarDetect from /scan [callback function]
    * 
    */
    void lidarCallback([[maybe_unused]] const sensor_msgs::msg::LaserScan & lidarMsg){

        if (checkingInProgress && !lidarDetect){
            lidarDetect = true;
        }
    }

    /* Update cameraDetect from /image_raw [callback function]
    * 
    
    void cameraCallback([[maybe_unused]] const sensor_msgs::msg::Image & cameraMsg){
        if (checkingInProgress && !cameraDetect){
            cameraDetect = true;
        }
    }
    */


    // ----- Private Variables ------
    int nodeStart = true;

    //Time
    int seconds = 0;
    bool timeout = false;

    //Checking's status
    bool startChecking = true;
    bool checkingInProgress = false;

    //Communications
    bool commJetson = false;
    bool commL476 = false;
    bool commF103 = false;

    //Sensors
    int usDistance[6] = {-1,-1,-1,-1,-1,-1};
    int gpsQuality = -1;
    float batteryLevel = -1;
    bool usDetect = false;
    bool gpsDetect = false;
    bool imuRawDetect = false;
    bool imuMagDetect = false;
    bool lidarDetect = false;
    bool cameraDetect = false;
    bool batteryDetect = false;

    //Report
    string usReport[6];
    string commF103Report, commL476Report, commJetsonReport, batteryReport, gpsReport, imuReport, lidarReport, cameraReport;

   
    //Publishers
    rclcpp::Publisher<interfaces::msg::SystemCheck>::SharedPtr publisher_system_check_;

    //Subscribers

    rclcpp::Subscription<interfaces::msg::SystemCheck>::SharedPtr subscription_system_check_;

    rclcpp::Subscription<interfaces::msg::Gnss>::SharedPtr subscription_gnss_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu_raw_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr subscription_imu_mag_;

    rclcpp::Subscription<interfaces::msg::GeneralData>::SharedPtr subscription_general_data_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_us_;

    //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_camera_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_lidar_;

    //Timers
    rclcpp::TimerBase::SharedPtr timer_seconds_;
    rclcpp::TimerBase::SharedPtr timer_update_checking_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<system_check>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}