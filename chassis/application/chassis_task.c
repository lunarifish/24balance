
#include "chassis_task.h"

#include "main.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdbool.h>

#include "pid.h"
#include "app_can.h"
#include "BMI088driver.h"
#include "bsp_buzzer.h"



extern motor_measure_t motor_measure[5];

//         -        +
// [0]  CW <- yaw   -> CCW
// [1]   L <- roll  -> R 
// [2]   L <- pitch -> H
extern bmi088_real_data_t bmi088_real_data;

extern fp32 INS_angle[3];
extern RC_ctrl_t RC_ctrl;

pid_type_def chassis_pid[3];                // 0 for linear speed, 1 for upright, 2 for yaw position

fp32 left_compensate = 1.03f;               // compensate different damping between two wheels
                                            // left wheel has more friction
fp32 chassis_linear_speed;
fp32 rpm_left, rpm_right;
fp32 output_left, output_right;
fp32 rotation;
uint16_t pid_init_ramp = 0;

// NOTE don't use rc_lpf[0](which is for linear speed) because it will cause chassis to shake
rc_lpf_t rc_lpf[2] = {{0.05f, 0.0f}, {0.1f, 0.0f}};
fp32 move_input_multiplier = MOVE_INPUT_MULTIPLIER;         // multiply fwd/bwd input from remote by this value
                                                            // then pass it to linear speed pid

static void pidInit() {
    // linear speed pid
    PID_init(chassis_pid, 
             PID_POSITION,
             (fp32[3]){CHASSIS_LINEAR_SPEED_PID_KP, CHASSIS_LINEAR_SPEED_PID_KI, CHASSIS_LINEAR_SPEED_PID_KD},
             CHASSIS_LINEAR_SPEED_PID_MAX_OUT, CHASSIS_LINEAR_SPEED_PID_MAX_IOUT);
    // upright pid
    PID_init(chassis_pid + 1,
             PID_POSITION,
             (fp32[3]){CHASSIS_UPRIGHT_PID_KP, CHASSIS_UPRIGHT_PID_KI, CHASSIS_UPRIGHT_PID_KD},
             CHASSIS_UPRIGHT_PID_MAX_OUT, CHASSIS_UPRIGHT_PID_MAX_IOUT);
    // yaw pid
    PID_init(chassis_pid + 2, 
             PID_POSITION,
             (fp32[3]){CHASSIS_YAW_PID_KP, CHASSIS_YAW_PID_KI, CHASSIS_YAW_PID_KD},
             CHASSIS_YAW_PID_MAX_OUT, CHASSIS_YAW_PID_MAX_IOUT);
}


static inline fp32 limitOutput(fp32 input, fp32 max, fp32 min) {
    if (input > max) {
        return max;
    } else if (input < min) {
        return min;
    } else {
        return input;
    }
}


static inline float lpfCalc(rc_lpf_t *rc_lpf, float val) {
    rc_lpf->lVal = ((float)val * rc_lpf->k + rc_lpf->lVal * (1 - rc_lpf->k));
    return rc_lpf->lVal;
}


static bool isSlipping(uint16_t wheel_speed) {
    return (wheel_speed > 0xf000);
}

/***
 * @note recover sequence
 * @note left switch: down -> middle -> up -> middle -> up
 */
static void emergencyLoop() {
    uint8_t sequence[5] = {0x2, 0x3, 0x1, 0x3, 0x1};
    uint8_t sequence_index = 0;
    while (1) {
        osDelay(1);
        canCmdChassis3508(0, 0);

        if (RC_ctrl.s[1] == sequence[sequence_index]) {
            ++sequence_index;
            if (sequence_index == 5) {
                break;
            }
            while (RC_ctrl.s[1] == sequence[sequence_index]) {
                osDelay(1);
            }
        } else {
            sequence_index = 0;
        }
    }
}

void chassisTask(void const *pvParameter) {
    osDelay(9000);  // wait for IMU data to stabilize(this will take about 11 seconds)
                    // 9sec + pid ramp in time(5sec) should be fine
    pidInit();

    while (1) {
        osDelay(1);
        
        // pause if switch is in middle
        if (switch_is_mid(RC_ctrl.s[1])) {
            buzzer_beep(2, 100);
            PID_clear(chassis_pid);
            PID_clear(chassis_pid + 1);
            PID_clear(chassis_pid + 2);
            continue;
        }

        // pause if lose control
        // in order to recover from this state, left switch must be set in a sequence
        if (fabs(INS_angle[2] * 1000) > FALL_ANGLE_THRESHOLD) {
            emergencyLoop();
        }

        // pid ramp in on startup
        if (pid_init_ramp < PID_RAMP_IN_TIME) {
            ++pid_init_ramp;
            chassis_pid[0].Kp = CHASSIS_LINEAR_SPEED_PID_KP * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[0].Ki = CHASSIS_LINEAR_SPEED_PID_KI * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[0].Kd = CHASSIS_LINEAR_SPEED_PID_KD * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[1].Kp = CHASSIS_UPRIGHT_PID_KP * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[1].Ki = CHASSIS_UPRIGHT_PID_KI * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[1].Kd = CHASSIS_UPRIGHT_PID_KD * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[2].Kp = CHASSIS_YAW_PID_KP * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[2].Ki = CHASSIS_YAW_PID_KI * pid_init_ramp / PID_RAMP_IN_TIME;
            chassis_pid[2].Kd = CHASSIS_YAW_PID_KD * pid_init_ramp / PID_RAMP_IN_TIME;
        }

        rpm_left = LEFT_WHEEL_REV ? -motor_measure[0].speed_rpm : motor_measure[0].speed_rpm;
        rpm_right = RIGHT_WHEEL_REV ? -motor_measure[1].speed_rpm : motor_measure[1].speed_rpm;
        chassis_linear_speed = (rpm_left + rpm_right) * 0.5f;

        rotation = bmi088_real_data.gyro[1] * 1000;

        // low pass filter calculations
        // bypassed, will cause chassis to shake
        // chassis_linear_speed = lpfCalc(&rc_lpf[0], chassis_linear_speed);
        rotation = lpfCalc(&rc_lpf[1], rotation);

        // pid calculations
        PID_calc(&chassis_pid[0], chassis_linear_speed / 1000, RC_ctrl.ch[1] * move_input_multiplier);
        PID_calc(&chassis_pid[1], INS_angle[2] * 1000, chassis_pid[0].out);
        PID_calc(&chassis_pid[2], rotation, -RC_ctrl.ch[0] * 2);
        
        output_left = chassis_pid[1].out + chassis_pid[2].out;
        output_right = chassis_pid[1].out - chassis_pid[2].out;

        // post processing
        output_left = limitOutput(output_left, MAX_3508_MOTOR_CAN_CURRENT, MIN_3508_MOTOR_CAN_CURRENT);
        output_right = limitOutput(output_right, MAX_3508_MOTOR_CAN_CURRENT, MIN_3508_MOTOR_CAN_CURRENT);
        
#if LEFT_WHEEL_REV
        output_left = -output_left;
#endif
#if RIGHT_WHEEL_REV
        output_right = -output_right;
#endif

        canCmdChassis3508(output_left, output_right);
    }

    // thread end
    while (1) {
        osDelay(1);
    }
}
