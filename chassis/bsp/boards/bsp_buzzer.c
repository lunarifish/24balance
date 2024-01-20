#include "bsp_buzzer.h"
#include "main.h"

#include "cmsis_os.h"

extern TIM_HandleTypeDef htim4;

void buzzer_on(uint16_t psc, uint16_t pwm) {
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}

void buzzer_off(void) {
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void buzzer_beep(uint8_t times, uint8_t interval) {
    for (uint8_t i = 0; i < times; i++) {
        buzzer_on(1, 30000);
        osDelay(interval);
        buzzer_off();
        osDelay(interval);
    }
}
