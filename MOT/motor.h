/*
 * motor.h
 *
 *  Created on: Mar 8, 2024
 *      Author: sgkim
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"

#define MOT_ENC_WHEEL_CNT 3432 // 11(모터 1회전   CNT) *  78(Reduction ratio) * 4(4phase)  = 3432
#define MOT_WHEEL_DIA 65  // wheel dia meter 65mm
#define MOT_WHEEL_DIST 150  // distance between 2 wheel




void mot_all_stop();
void mota_dir_pwm(int dir, uint16_t pwm);
void motb_dir_pwm(int dir, uint16_t pwm);
int mota_enc_diff(void);
int motb_enc_diff(void);


#endif /* MOTOR_H_ */
