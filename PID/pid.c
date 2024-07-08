/*
 * pid.c
 *
 *  Created on: Mar 9, 2024
 *      Author: sgkim
 */

#ifndef PID_C_
#define PID_C_

#include "main.h"
#include "../MOT/motor.h"
#include <math.h>
#include <stdlib.h>
#include "./pid.h"

mot_para mota = {};
mot_para motb = {};

car_move_t car_move;


#define DEBUG 1
#if DEBUG
#define dprintf printf
#else
#define dprintf
#endif



void pid_para_init()
{
    memset(&mota, 0 , sizeof(mot_para));

    mota.Kp = 1.0f;          // 비례 상수
    mota.Ki = 0.1f;          // 적분 상수
    mota.Kd = 0.05f;         // 미분 상수

    memset(&motb, 0 , sizeof(mot_para));

    motb.Kp = 1.0f;          // 비례 상수
    motb.Ki = 0.1f;          // 적분 상수
    motb.Kd = 0.05f;         // 미분 상수

}


void pid_set_target_speed(unsigned int mot_no, int speed)
{
    if(mot_no == 0)
        mota.target_speed = speed;
    if(mot_no == 1)
        motb.target_speed = speed;
}


void get_report_speed(uint16_t tdelta)
{
    car_move.sum_cnt_l = 0;
    car_move.sum_cnt_r = 0;

    car_move.sum_cnt_l = mota.sum_enc_cnt; mota.sum_enc_cnt = 0;
    float mota_speed =  ( ((float)car_move.sum_cnt_l) / MOT_ENC_WHEEL_CNT ) * ( MOT_WHEEL_DIA * M_PI) / (float)tdelta * 1000; //tdelata_ms

    car_move.sum_cnt_r = motb.sum_enc_cnt; motb.sum_enc_cnt = 0;
    float motb_speed =  ( ((float)car_move.sum_cnt_r) / MOT_ENC_WHEEL_CNT ) * ( MOT_WHEEL_DIA * M_PI) / (float) tdelta * 1000; //tdelata_ms

    car_move.cur_speed_vel = (mota_speed + motb_speed) / 2.0f;

    // 각도 계산 (여기서는 각속도를 기반으로 계산)
    float speed_difference = (mota_speed - motb_speed) * -1;
    float angular_velocity = speed_difference / (MOT_WHEEL_DIST) * (180.0f /M_PI ) * 1000;  // 각속도 = 속도 차이 / (바퀴 간의 거리 / 2) * 1000 배
    car_move.cur_angle_vel = angular_velocity;

    return;

}



// PID 제어 함수
void pid_controller_a(uint16_t tdelta) {

    static float previous_error = 0; // 이전 오차
    static float integral = 0;       // 적분


    int diff_no = mota_enc_diff();
    mota.sum_enc_cnt += diff_no;

    float cur_speed =  ( ((float)diff_no) / MOT_ENC_WHEEL_CNT ) * ( MOT_WHEEL_DIA * M_PI) / (float)tdelta * 1000; //tdelata_ms
    mota.cur_speed = cur_speed;


    // 현재 오차 계산
    float error = mota.target_speed - mota.cur_speed;

    // 비례 제어
    float proportional = mota.Kp * error;

    // 적분 제어
    integral += error;
    float integral_control = mota.Ki * integral;

    // 미분 제어
    float derivative = mota.Kd * (error - previous_error);
    previous_error = error;

    // PID 제어 값 계산
    int control = (int)(proportional + integral_control + derivative);

    // 제어값 범위 설정
    if (control > MAX_SPEED) {
        control = MAX_SPEED;
    } else if (control < MIN_SPEED) {
        control = MIN_SPEED;
    }


    if(control >= 0) // ahead direction
    {
        uint16_t pwm = (uint16_t)abs(control);
        mota_dir_pwm(1, pwm);
    }
    else // back direction
    {
        uint16_t pwm = (uint16_t)abs(control);
        mota_dir_pwm(-1, pwm);
    }


    if( (int)mota.target_speed == 0 )
    {
        control = 0;
        previous_error = 0;
        integral = 0;
    }


}


// PID 제어 함수
void pid_controller_b(uint16_t tdelta) {

    static float previous_error = 0; // 이전 오차
    static float integral = 0;       // 적분


    int diff_no = motb_enc_diff();
    motb.sum_enc_cnt += diff_no;

    float cur_speed =  ( ((float)diff_no) / MOT_ENC_WHEEL_CNT ) * ( MOT_WHEEL_DIA * M_PI) / (float)tdelta * 1000; //tdelata_ms

    motb.cur_speed = cur_speed;


    // 현재 오차 계산
    float error = motb.target_speed - motb.cur_speed;

    // 비례 제어
    float proportional = motb.Kp * error;

    // 적분 제어
    integral += error;
    float integral_control = motb.Ki * integral;

    // 미분 제어
    float derivative = motb.Kd * (error - previous_error);
    previous_error = error;

    // PID 제어 값 계산
    int control = (int)(proportional + integral_control + derivative);

    // 제어값 범위 설정
    if (control > MAX_SPEED) {
        control = MAX_SPEED;
    } else if (control < MIN_SPEED) {
        control = MIN_SPEED;
    }


    if(control >= 0) // ahead direction
    {
        uint16_t pwm = (uint16_t)abs(control);
        motb_dir_pwm(1, pwm);
    }
    else // back direction
    {
        uint16_t pwm = (uint16_t)abs(control);
        motb_dir_pwm(-1, pwm);

    }

    if( (int)motb.target_speed == 0 )
   {
       control = 0;
       previous_error = 0;
       integral = 0;
   }

}




#endif /* PID_C_ */
