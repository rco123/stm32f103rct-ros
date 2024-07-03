/*
 * cmd.c
 *
 *  Created on: Mar 8, 2024
 *      Author: sgkim
 */

#include <stdio.h>
#include "../CJSON/cJSON.h"
#include "../MOT/motor.h"
#include "../PID/pid.h"
#include <math.h>
#include <string.h>

extern mot_para mota;
extern mot_para motb;


#define DEBUG 1
#if DEBUG
#define dprintf printf
#else
#define dprintf
#endif

// cmd set
//  {"cmd":"stop_mt" }
//  {"cmd":"mot", "mot_no": 1 , "dir" : 1, "pwm" : 400 }
//  {"cmd":"enc"}

//  {"cmd":"stop" }
//  {"cmd":"set_mtsp", "mt0": 40, "mt1": 40 }  //set motor speed
//  {"cmd":"get_cnts"}



void process_json(const char *json_string)
{
    // JSON 문자열을 파싱하여 cJSON 객체를 생성합니다.
    cJSON *json = cJSON_Parse(json_string);

    // cJSON_Parse()가 실패하면 NULL을 반환합니다.
    if (json == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            fprintf(stderr, "Error before: %s\n", error_ptr);
        }
        return;
    }

    char cmd_str[20];
    int mot_no = 0;
    int dir_val = 0;
    int pwm_val = 0;
    int mt0_val = 0;
    int mt1_val = 0;
    int xdir_val = 0;
    int zang_val = 0;

    // cJSON 객체를 순회하면서 각 키와 값을 처리합니다.
    cJSON *item = NULL;

    cJSON_ArrayForEach(item, json) {
        // 키와 값을 출력합니다.
        dprintf("Key: %s\n", item->string);

        if(!strcmp(item->string, "cmd"))  // main command
        {
            memset( cmd_str, 0, sizeof(cmd_str));
            strcpy( cmd_str,  item->valuestring );
        }

        if(!strcmp(item->string, "mot_no"))
        {
            mot_no = (int)item->valuedouble;
        }

        if(!strcmp(item->string, "dir"))
        {
            dir_val = (int)item->valuedouble;
        }
        if(!strcmp(item->string, "pwm"))
        {
              pwm_val = (int)item->valuedouble;
        }

        // =========================
        if(!strcmp(item->string, "mt0"))
        {
              mt0_val = (int)item->valuedouble;
        }
        if(!strcmp(item->string, "mt1"))
        {
              mt1_val = (int)item->valuedouble;
        }


        // =========================
        if(!strcmp(item->string, "xdir"))
        {
              xdir_val = (int)item->valuedouble;
        }
        if(!strcmp(item->string, "zang"))
        {
              zang_val = (int)item->valuedouble;
        }



    }
    // cJSON 객체를 해제합니다.
    cJSON_Delete(json);



    // Tread Json Command

    if(!strcmp(cmd_str,"mot" ))
    {
        if(mot_no == 0)
        {
            mota_dir_pwm(dir_val, pwm_val);
        }
        else if( mot_no == 1)
        {
            motb_dir_pwm(dir_val, pwm_val);
        }

    }

    if(!strcmp(cmd_str,"stop_mt" ))
    {
        mot_all_stop();
    }

    //====================================
    if(!strcmp(cmd_str,"enc" ))
    {
        int no;
        no = mota_enc_diff();
        dprintf("diff enc a = %d \n", no );

        no = motb_enc_diff();
        dprintf("diff enc b = %d \n", no );
    }

    //cmd set_mtsp ========================
    if(!strcmp(cmd_str,"set_mtsp" ))
    {
        dprintf("==> mt0 mt1 %d %d \n", mt0_val, mt1_val);
        pid_set_target_speed(0, mt0_val);
        pid_set_target_speed(1, mt1_val);
    }

    if(!strcmp(cmd_str,"stop" ))
    {
       mot_all_stop();
       pid_set_target_speed(0, 0);
       pid_set_target_speed(1, 0);
    }

    if(!strcmp(cmd_str,"set_vel" ))
    {
        // xdir_val, zang_val convert
        float mt0_val =  (float)xdir_val -  ( ((float)zang_val / 180 * M_PI) * MOT_WHEEL_DIST ) / 2;
        pid_set_target_speed(0, (int)mt0_val);

        float mt1_val =  (float)xdir_val +  ( ((float)zang_val / 180 * M_PI) * MOT_WHEEL_DIST ) / 2;
        pid_set_target_speed(1, (int)mt1_val);

    }

    if(!strcmp(cmd_str,"get_cnt"))
    {
        int mt0_cnt = 0;
        int mt1_cnt = 0;

        mt0_cnt = mota.sum_enc_cnt; mota.sum_enc_cnt = 0;
        mt1_cnt = motb.sum_enc_cnt; motb.sum_enc_cnt = 0;

        printf("{ \"cmd\":\"get_cnt\", \"mt0\": %d,\"mt1\":%d }\n", mt0_cnt, mt1_cnt);

    }


}



