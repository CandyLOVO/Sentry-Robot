#ifndef CHASSIS_TASK_H
#define  CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "struct_typedef.h"
#include  "drv_can.h"
//#include "rc_potocal.h"
#include "main.h"
#include "judge.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "rc_potocal.h"
#include "motor.h"
#include "pid.h"

#define angle_valve 5		//角度阈值
#define angle_weight 55		//角度权重

typedef enum {
    CHAS_LF,
    CHAS_RF,
    CHAS_RB,
    CHAS_LB,
} chassis_motor_cnt_t;

extern float Drifting_yaw;
extern float Down_ins_yaw;
extern float Up_ins_yaw; 

void Chassis_task(void const *pvParameters);
void RC_to_Vector(void);
void chassis_motol_speed_calculate(void);
void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed);
void chassis_current_give(void);

#ifdef __cplusplus
}
#endif

#endif
