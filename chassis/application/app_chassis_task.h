
#ifndef BALANCE_CHASSIS_APP_CHASSIS_TASK_H
#define BALANCE_CHASSIS_APP_CHASSIS_TASK_H

#include "struct_typedef.h"

#define MAX_3508_MOTOR_CAN_CURRENT       16384.0f
#define MIN_3508_MOTOR_CAN_CURRENT      -16384.0f
#define MOTOR_3508_SCALING_FACTOR        0.42724609375f
#define MAX_6020_MOTOR_CAN_CURRENT       30000.0f
#define MIN_6020_MOTOR_CAN_CURRENT      -30000.0f
#define MOTOR_6020_MAX_ECD               8191.0f
#define CHASSIS_PID_OUTPUT_CURVE_FACTOR -1.f

#define LEFT_WHEEL_REV  1
#define RIGHT_WHEEL_REV 0

#define PID_RAMP_IN_TIME      5000.0f
#define FALL_ANGLE_THRESHOLD  450.0f
#define MOVE_INPUT_MULTIPLIER 0.0015f

// chassis linear speed PID parameters
#define CHASSIS_LINEAR_SPEED_PID_KP 16.0f
#define CHASSIS_LINEAR_SPEED_PID_KI 0.1f
#define CHASSIS_LINEAR_SPEED_PID_KD 0.0f
#define CHASSIS_LINEAR_SPEED_PID_MAX_OUT 240.0f
#define CHASSIS_LINEAR_SPEED_PID_MAX_IOUT 200.f

// chassis stand ring PID parameters
#define CHASSIS_UPRIGHT_PID_KP 35.0f
#define CHASSIS_UPRIGHT_PID_KI 0.0f
#define CHASSIS_UPRIGHT_PID_KD 1100.0f
#define CHASSIS_UPRIGHT_PID_MAX_OUT MAX_3508_MOTOR_CAN_CURRENT
#define CHASSIS_UPRIGHT_PID_MAX_IOUT 2000.0f

// chassis yaw PID parameters
#define CHASSIS_YAW_PID_KP 3.0f
#define CHASSIS_YAW_PID_KI 0.0f
#define CHASSIS_YAW_PID_KD 0.0f
#define CHASSIS_YAW_PID_MAX_OUT (MAX_3508_MOTOR_CAN_CURRENT * 0.5f)
#define CHASSIS_YAW_PID_MAX_IOUT 2000.0f

// chassis left trim PID parameters
#define CHASSIS_LEFT_TRIM_PID_KP 0.0f
#define CHASSIS_LEFT_TRIM_PID_KI 0.0f
#define CHASSIS_LEFT_TRIM_PID_KD 0.0f
#define CHASSIS_LEFT_TRIM_PID_MAX_OUT 0.0f
#define CHASSIS_LEFT_TRIM_PID_MAX_IOUT 0.0f

// chassis right trim PID parameters
#define CHASSIS_RIGHT_TRIM_PID_KP 0.0f
#define CHASSIS_RIGHT_TRIM_PID_KI 0.0f
#define CHASSIS_RIGHT_TRIM_PID_KD 0.0f
#define CHASSIS_RIGHT_TRIM_PID_MAX_OUT 0.0f
#define CHASSIS_RIGHT_TRIM_PID_MAX_IOUT 0.0f


// low pass filter struct
typedef struct
{
  fp32 k;      // filter factor
  fp32 lVal;   // last value
} rc_lpf_t;

void chassisTask(void const *pvParameter);

#endif
