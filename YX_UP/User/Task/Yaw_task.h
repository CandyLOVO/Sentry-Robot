#ifndef YAW_TASK_H
#define YAW_TASK_H

#include "INS_task.h"
#include "pid.h"
#include "struct_typedef.h"
#include "remote_control.h"

//����һЩ�����ļ���ȫ�ֱ���
extern int16_t mouse_x;		//��������(���Բ���ϵͳ)
extern ins_data_t ins_data;		//���������ǽ������������
extern RC_ctrl_t rc_ctrl;		//ң��������

//���庯��
void Yaw_task(void const *pvParameters);
#endif