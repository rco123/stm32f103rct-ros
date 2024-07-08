/*
 * pid.h
 *
 *  Created on: Mar 9, 2024
 *      Author: sgkim
 */

#ifndef PID_H_
#define PID_H_

#define MAX_SPEED 900 // 최대 속도
#define MIN_SPEED -900    // 최소 속도


typedef struct mot_para_
{
    float target_speed;
    float cur_speed;
    int32_t sum_enc_cnt;

    float Kp;          // 비례 상수
    float Ki;          // 적분 상수
    float Kd;         // 미분 상수


} mot_para;


typedef struct {

    float cur_angle_vel;
    float cur_speed_vel;
    int32_t sum_cnt_l;
    int32_t sum_cnt_r;

} car_move_t;


void pid_para_init();
void pid_set_target_speed(unsigned int mot_no, int speed);

// PID 제어 함수
void pid_controller_a(uint16_t tdelta);
void pid_controller_b(uint16_t tdelta);


void get_report_speed(uint16_t delta_t);


#endif /* PID_H_ */
