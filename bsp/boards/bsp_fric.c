#include "bsp_fric.h"
#include "main.h"
extern TIM_HandleTypeDef htim1;
void fric_off(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, FRIC_OFF);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, FRIC_OFF);
}
void fric1_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, cmd);
}
void fric2_on(uint16_t cmd)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, cmd);
}

void door_open(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1000); // Í£Ö¹¶æ»ú
}

void door_close(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 2000); // Í£Ö¹¶æ»ú
}

void door_stop(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0); // Í£Ö¹¶æ»ú
}