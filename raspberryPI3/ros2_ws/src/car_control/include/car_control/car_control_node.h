#ifndef __car_control_node_H
#define __car_control_node_H

#include <stdint.h>
#include <string.h>  


#define PERIOD_UPDATE_CMD 1ms //Period to update proupulsion and steering command in [ms]

#define STOP 50 //PWM value to stop motors

#define TIME 1000 //we suppose 1000 is equal to 1s

void go_forward(); //test : car moving forward for 20 meters

void go_backward(); //test : car moving backward for 20 meters

void accel_decel_stop(); //test : acceleration 5 meters full speed - deceleration 5 meters half speed - stop

#endif /*__ car_control_node_H */