/*
 * motor.c
 *
 *  Created on: Mar 8, 2024
 *      Author: sgkim
 */

#include "main.h"



void mot_all_stop()
{
    TIM8->CCR1 = 0;
    TIM8->CCR2 = 0;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Motor Drive off
    HAL_Delay(1);

    TIM8->CCR1 = 0;
    TIM8->CCR2 = 0;

}


void mota_dir_pwm(int dir, uint16_t pwm)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // Motor Drive on

    if(dir >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // go
        TIM8->CCR1 = pwm;

    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // back
        TIM8->CCR1 = pwm;

    }

}


void motb_dir_pwm(int dir, uint16_t pwm)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // Motor Drive on

    if(dir >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // go
        TIM8->CCR2 = pwm;

    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // back
        TIM8->CCR2 = pwm;
    }

}


int mota_enc_diff(void)
{
    int cnt=0;
    cnt = (int16_t)TIM4->CNT; TIM4->CNT = 0;
    return cnt * -1; // reverse value return go forward
}

int motb_enc_diff(void)
{
    int cnt=0;
    cnt = (int16_t)TIM5->CNT; TIM5->CNT = 0;
    return cnt * -1;  // reverse value return go forward
}




