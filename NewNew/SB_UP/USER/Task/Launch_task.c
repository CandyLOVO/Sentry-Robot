#include "Launch_task.h"
#include "cmsis_os.h"
#include "can_user.h"
#include "pid_user.h"
#include "imu_temp_ctrl.h"
#include "Exchange_task.h"

pidTypeDef pid_mocalun_s[4];
pidTypeDef pid_bopan_s[2];
float target_speed = 19 * 380;
float target_mocalun[4];
float target_bopan[2];
int16_t mocalun_output[4];
int16_t bopan_output[2];

Shooter_t Shooter_L;
Shooter_t Shooter_R;

extern FDCAN_HandleTypeDef hfdcan2;
extern motor_info motor_friction[8];
extern RC_ctrl_t rc_ctrl;
extern int8_t flag;
extern receive_vision Rx_vision;

#define mocalun_speed         19 * 380 // Ħ����ת��(����ʵ��������Ŀ��ٵ������٣�
#define C_bopan_reversal_time 0.4f     // ���̷�תʱ��(s)
#define K_shoot_rate_correct  1        // ��Ƶ��������������ʵ��������Ŀ��ٵ�����Ƶ��
#define C_bopan_block_I       12000    // ���̶�ת���������Ժ���ģ�
#define K_rc_to_bopanSpeed    8        // ң��ͨ��ֵ�л��������ٶȣ����Ŀɿ��ٵ���1-1ģʽ��ң���벦��ӳ���ϵ��

// PID��ʼ��
static void Friction_init(void);

// Ħ���ּ��ٸ�ֵ
static void Friction_calc(void);

// Ħ���ּ��ٸ�ֵ
static void Friction_down(void);

// ���̶�ת���
static void Bopan_judge(void);

// Ħ����Pid���ֵ����
void can_send_mocalun(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
// ����Pid���ֵ�ͷ���
void can_send_bopan(int16_t motor1, int16_t motor2);

// ����PId����
static void Bopan_calc(void);

// ��������
static void Bopan_speed_calc_L(int speed_rate_high, int speed_rate_low, int speed_rate_test);
static void Bopan_speed_calc_R(int speed_rate_high, int speed_rate_low, int speed_rate_test);

// ң�ؿ���
static void DataUpGrade(void);
static void re_shoot_rate_calc(void);

//===============================================ȫ�ֱ���================================================//
int16_t bopan_shoot_rate_max      = 10 ;                  // �����Ƶ����/s��
int16_t bopan_shoot_rate_min      = 7 ;                   // �����Ƶ
int16_t bopan_shoot_rate_test     = 5;                      // �޲���ϵͳ��Ƶ
float bopan_shoot_rate_visiontest =0.5;										//���������Ƶ
int16_t bopan_reversal_shoot_rate = -10;                     // ���̷�ת��Ƶ
uint8_t bopan_reversal_flag_L = 0, bopan_reversal_flag_R = 0; // ���̷�ת��־λ��0Ϊ��ת��1Ϊ��ת
int16_t remote_mode;                                          // ң��ģʽ��1-1ΪFire Control ģʽ��Ħ������ת��ң������ҡ�˿��Ʋ�����ת��2-2Ϊ�ϳ�ģʽ��Ħ������ת�������Ӿ�ʶ��λ����
float Sys_time;
int Vision_Test_Flag=0;
// ��ͷ����Ħ����ID��1��2������ID��6����ͷ���ң�Ħ���֣�3��4�����̣�5
void Launch_Task(void *argument)
{
    Friction_init(); // PID��ʼ��

    for (;;) {
		
        if (flag == 1) {			
            DataUpGrade();
            re_shoot_rate_calc();
            //===============================================Ħ����================================================//
            // ����Ħ����
            if (remote_mode == 22 || remote_mode == 11) // Ħ���ֿ�������
            {
                Friction_calc(); // ת��->����
            } else               // Ħ���ֹر�
            {
                Friction_down();
            }
            can_send_mocalun(mocalun_output[0], mocalun_output[1], mocalun_output[2], mocalun_output[3]); // Ħ���ֵ�������
            //===============================================����================================================//
            //===========================================�Զ�ģʽ begin================================//
            if (remote_mode == 22) {

                if (Shooter_L.Fire_Flag == 1) // ��ǹ�ܷ���
                {	
                    if (bopan_reversal_flag_L == 1) {
                        Bopan_speed_calc_L(bopan_reversal_shoot_rate, bopan_reversal_shoot_rate, bopan_reversal_shoot_rate);
                    } else if (bopan_reversal_flag_L == 0) {
                        Bopan_speed_calc_L(bopan_shoot_rate_max, bopan_shoot_rate_min, bopan_shoot_rate_test); // �����Ƶ�������Ƶ���޲���ϵͳ��Ƶ
                    }
                }

                else {
                    Bopan_speed_calc_L(0, 0, 0);
                }

                if (Shooter_R.Fire_Flag == 1) // ��ǹ�ܷ���
                {
                    if (bopan_reversal_flag_R == 1) {
                        Bopan_speed_calc_R(bopan_reversal_shoot_rate, bopan_reversal_shoot_rate, bopan_reversal_shoot_rate); // �����Ƶ�������Ƶ���޲���ϵͳ��Ƶ
                    } else if (bopan_reversal_flag_R == 0) {
                        Bopan_speed_calc_R(bopan_shoot_rate_max, bopan_shoot_rate_min, bopan_shoot_rate_test); // �����Ƶ�������Ƶ���޲���ϵͳ��Ƶ
                    }
                }

                else {
                    Bopan_speed_calc_R(0, 0, 0);
                }

            }

            //========================ң��ģʽ==============================//

            else if (remote_mode == 11) {
                re_shoot_rate_calc(); // ң�ؿ��Ʋ��̣�����ҡ������
            }

            else {
                Bopan_speed_calc_L(0, 0, 0);
                Bopan_speed_calc_R(0, 0, 0);
            }

            Bopan_judge();                                    // ���̶�ת���
            Bopan_calc();                                     // ת��-->����
            can_send_bopan(bopan_output[0], bopan_output[1]); // ���̵�������
            osDelay(1);
        }      
    }
}
static void DataUpGrade()
{	
		Shooter_L.Fire_Flag=Rx_vision.L_tracking || Rx_vision.M_tracking;
		Shooter_R.Fire_Flag=Rx_vision.R_tracking || Rx_vision.M_tracking;
		Shooter_L.Fire_Flag=0;
		Shooter_R.Fire_Flag=0;
	
    if ((rc_ctrl.rc.s[1] == 2 && rc_ctrl.rc.s[0] == 2) ) {
        remote_mode = 22;
    } else if (rc_ctrl.rc.s[1] == 1 && rc_ctrl.rc.s[0] == 1) {
        remote_mode = 11;
    } else {
        remote_mode = 0;
    }
	
		if(rc_ctrl.rc.ch[0]==-660){
				Vision_Test_Flag=1;
			} else {
				Vision_Test_Flag=0;
			}
}

static void re_shoot_rate_calc()
{
    target_bopan[1] = rc_ctrl.rc.ch[3] * K_rc_to_bopanSpeed;
    target_bopan[0] = rc_ctrl.rc.ch[1] * K_rc_to_bopanSpeed;
}

//===============================================PID��ʼ��================================================//
static void Friction_init()
{
    pid_init(&pid_mocalun_s[0], 40, 0.8, 1,16384,10000); // Ħ����
    pid_init(&pid_mocalun_s[1], 40, 0.8, 1,16384,10000);
    pid_init(&pid_mocalun_s[2], 40, 0.8, 1,16384,10000);
    pid_init(&pid_mocalun_s[3], 40, 0.8, 1,16384,10000);

    pid_init(&pid_bopan_s[0], 20, 0.03, 0.5,16384,10000); // ����()
    pid_init(&pid_bopan_s[1], 20, 0.03, 0.5,16384,10000); // 20��0.03��0.5

    Shooter_L.shooter_heat = 1025;
    Shooter_R.shooter_heat = 1025;
  
    target_mocalun[0]  = 0;
    target_mocalun[1]  = 0;
    target_mocalun[2]  = 0;
    target_mocalun[3]  = 0;
}

//==============================================Ħ����ת��->����================================================//
static void Friction_calc()
{
    if (target_mocalun[0] > -target_speed) {
        target_mocalun[0] -= 10;
    } else {
        target_mocalun[0] = -target_speed;
    }

    if (target_mocalun[1] < target_speed) {
        target_mocalun[1] += 10;
    } else {
        target_mocalun[1] = target_speed;
    }

    if (target_mocalun[2] > -target_speed) {
        target_mocalun[2] -= 10;
    } else {
        target_mocalun[2] = -target_speed;
    }

    if (target_mocalun[3] < target_speed) {
        target_mocalun[3] += 10;
    } else {
        target_mocalun[3] = target_speed;
    }

				mocalun_output[0] = pid_cal_s(&pid_mocalun_s[0], motor_friction[0].speed, target_mocalun[0]);
        mocalun_output[1] = pid_cal_s(&pid_mocalun_s[1], motor_friction[1].speed, target_mocalun[1]);
        mocalun_output[2] = pid_cal_s(&pid_mocalun_s[2], motor_friction[2].speed, target_mocalun[2]);
        mocalun_output[3] = pid_cal_s(&pid_mocalun_s[3], motor_friction[3].speed, target_mocalun[3]);
}

//===============================================Ħ���ּ��ٵ���================================================//
static void Friction_down()
{
    target_mocalun[0] = 0;
    target_mocalun[1] = 0;
    target_mocalun[2] = 0;
    target_mocalun[3] = 0;

    mocalun_output[0] = 0;
    mocalun_output[1] = 0;
    mocalun_output[2] = 0;
    mocalun_output[3] = 0;
}

//===============================================����PID����================================================//
static void Bopan_calc()
{
    bopan_output[0]   = pid_cal_s(&pid_bopan_s[0], motor_friction[4].speed, -target_bopan[0]);
    bopan_output[1]   = pid_cal_s(&pid_bopan_s[1], motor_friction[5].speed, -target_bopan[1]);
}
//==========================================�����ٶȼ��㣨�������ƣ�======================================//

static void Bopan_speed_calc_L(int speed_rate_high, int speed_rate_low, int speed_rate_test) // �����������Ƶ�������Ƶ���޲���ϵͳ��Ƶ
{
    if (Shooter_L.shooter_heat <= 360) // ����С��300���������Ƶ����
    {
        target_bopan[1] = speed_rate_high*60 / 8 * 36 * K_shoot_rate_correct; // ���ת�٣�rpm��=��Ƶ����/s��/8(һȦ����8������*36��������̼��ٱ� 36��1��
    } 
    else if (Shooter_L.shooter_heat > 360 && Shooter_L.shooter_heat < 400) // 360<����<400,�������Ƶ����
    {
        target_bopan[1] = speed_rate_low *60/ 8 * 36 * K_shoot_rate_correct;
    } else if (Shooter_L.shooter_heat == 1025) // �޲���ϵͳʱ��ʼ��ǹ������Ϊ1025�����ٷ���
    {
        target_bopan[1] = speed_rate_test *60/ 8 * 36 * K_shoot_rate_correct;
    } else // �����������ϵͳ���ϣ�������ͣ
    {
       target_bopan[1] = 0;
    }
}

static void Bopan_speed_calc_R(int speed_rate_high, int speed_rate_low, int speed_rate_test) // �����������Ƶ�������Ƶ���޲���ϵͳ��Ƶ
{
    if (Shooter_R.shooter_heat <= 360) // ����С��300���������Ƶ����
    {
        target_bopan[0] = speed_rate_high*60 / 8 * 36 * K_shoot_rate_correct; // ���ת�٣�rpm��=��Ƶ����/s��/8(һȦ����8������*36��������̼��ٱ� 36��1��
    } 
    else if (Shooter_R.shooter_heat > 360 && Shooter_L.shooter_heat < 400) // 360<����<400,�������Ƶ����
    {
        target_bopan[0] = speed_rate_low *60/ 8 * 36 * K_shoot_rate_correct;
    } else if (Shooter_R.shooter_heat == 1025) // �޲���ϵͳʱ��ʼ��ǹ������Ϊ1025�����ٷ���
    {
        target_bopan[0] = speed_rate_test *60/ 8 * 36 * K_shoot_rate_correct;
    } else // �����������ϵͳ���ϣ�������ͣ
    {
       target_bopan[0] = 0;
    }
}
//=====================================================���̶�ת���=======================================//
static void Bopan_judge()
{
    if (motor_friction[5].tor_current < -C_bopan_block_I) {
        bopan_reversal_flag_L        = 1;
        Shooter_L.tim_reversal_begin = Sys_time;
    } else if (Sys_time - Shooter_L.tim_reversal_begin > C_bopan_reversal_time) {
        bopan_reversal_flag_L        = 0;
        Shooter_L.tim_reversal_begin = 0;
    }

    if (motor_friction[4].tor_current < -C_bopan_block_I) {
        bopan_reversal_flag_R        = 1;
        Shooter_R.tim_reversal_begin = Sys_time;
    } else if (Sys_time - Shooter_R.tim_reversal_begin > C_bopan_reversal_time) {
        bopan_reversal_flag_R        = 0;
        Shooter_R.tim_reversal_begin = 0;
    }
}
void can_send_mocalun(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint8_t mocalun_control[8];
    mocalun_control[0] = (motor1 >> 8) & 0xff;
    mocalun_control[1] = motor1 & 0xff;
    mocalun_control[2] = (motor2 >> 8) & 0xff;
    mocalun_control[3] = motor2 & 0xff;
    mocalun_control[4] = (motor3 >> 8) & 0xff;
    mocalun_control[5] = motor3 & 0xff;
    mocalun_control[6] = (motor4 >> 8) & 0xff;
    mocalun_control[7] = motor4 & 0xff;
    canx_send_data(&hfdcan2, 0x200, mocalun_control, 8);
}

void can_send_bopan(int16_t motor1, int16_t motor2)
{
    uint8_t bopan_control[8];
    bopan_control[0] = (motor1 >> 8) & 0xff;
    bopan_control[1] = motor1 & 0xff;
    bopan_control[2] = (motor2 >> 8) & 0xff;
    bopan_control[3] = motor2 & 0xff;
    canx_send_data(&hfdcan2, 0x1ff, bopan_control, 8);
}