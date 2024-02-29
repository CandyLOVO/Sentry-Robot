#ifndef YAW_TASK_H
#define YAW_TASK_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
#include "struct_typedef.h"
#include "PID.h"
#include "remote_control.h"
#include "handle_value.h"
#include "Motor.h"
#include "Can_user.h"
#include "MF_User.h"
#include "Exchange_task.h"

#define Yaw_sita_weight 0.5f 		//ң��������λ�û�Ȩ��
	 
extern float Yaw_middle_c;	//һ����̨yaw(ֻ�о�������)
extern float Yaw_left_c;	//����ʱ�����Դ���yaw��������꣩
extern float Yaw_right_c;
extern float Yaw_left;
extern float Yaw_right;
	 
//���庯��
void Yaw_task(void const *pvParameters);
static void Yaw_can_send();

#ifdef __cplusplus
}
#endif

#endif