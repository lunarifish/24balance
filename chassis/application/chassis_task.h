
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"

#define MAX_3508_MOTOR_CAN_CURRENT 16384.0f
#define MIN_3508_MOTOR_CAN_CURRENT -16384.0f
#define MAX_6020_MOTOR_CAN_CURRENT 30000.0f
#define MIN_6020_MOTOR_CAN_CURRENT -30000.0f

#define LEFT_WHEEL_REV  1
#define RIGHT_WHEEL_REV 0

#define PID_RAMP_IN_TIME 5000.0f

#define FALL_ANGLE_THRESHOLD 450.0f

#define MOVE_INPUT_MULTIPLIER 0.0005f


// chassis linear speed PID parameters
#define CHASSIS_LINEAR_SPEED_PID_KP 10.0f
#define CHASSIS_LINEAR_SPEED_PID_KI 1.0f
#define CHASSIS_LINEAR_SPEED_PID_KD 0.0f
#define CHASSIS_LINEAR_SPEED_PID_MAX_OUT 240.0f
#define CHASSIS_LINEAR_SPEED_PID_MAX_IOUT 200.f

// chassis stand ring PID parameters
#define CHASSIS_UPRIGHT_PID_KP 102.0f
#define CHASSIS_UPRIGHT_PID_KI 0.0f
#define CHASSIS_UPRIGHT_PID_KD 1150.0f
#define CHASSIS_UPRIGHT_PID_MAX_OUT MAX_3508_MOTOR_CAN_CURRENT
#define CHASSIS_UPRIGHT_PID_MAX_IOUT 2000.0f

// chassis yaw PID parameters
#define CHASSIS_YAW_PID_KP 3.0f
#define CHASSIS_YAW_PID_KI 0.0f
#define CHASSIS_YAW_PID_KD 0.0f
#define CHASSIS_YAW_PID_MAX_OUT (MAX_3508_MOTOR_CAN_CURRENT * 0.5f)
#define CHASSIS_YAW_PID_MAX_IOUT 2000.0f

// low pass filter struct
typedef struct
{
  fp32 k;      // filter factor
  fp32 lVal;   // last value
} rc_lpf_t;

void chassisTask(void const *pvParameter);

#endif
