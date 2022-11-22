#ifndef __speedCmd_H
#define __speedCmd_H

#include <cstdint>
#include <stdint.h>
#include <string.h>  



/* Calculate the PWM based on a wanted speed in RPM */
void calculateSpeed(float cmd_RearSpeed, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd, float currentRPM_L, float currentRPM_R, float& sumIntegralLeft, float& sumIntegralRight);

#endif /*__speedCmd_H */