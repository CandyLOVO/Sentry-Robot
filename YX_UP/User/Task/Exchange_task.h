#ifndef EXCHANGE_TASK_H
#define EXCHANGE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "INS_task.h"
#include "Can_user.h"
<<<<<<< HEAD
#include "SolveTrajectory.h"

#define BUFFER_SIZE 100
=======
>>>>>>> parent of 52bf27f (ä¿®æ”¹äº†å’Œè§†è§‰çš„æ¥å£)

void Exchange_task(void const * argument);

//================================================·¢ËÍÎ»Óò½á¹¹Ìå================================================//
typedef struct
{
	uint8_t detect_color : 1;	//´ÓµÍÎ»¿ªÊ¼
	uint8_t reset_tracker : 1;    // ·¢0
	uint8_t reserved : 6; 
}		pByte_t;	//Î»Óò
		
typedef struct
{
	uint8_t header;
	pByte_t official;
	float roll;                // rad ·¢0
  float pitch;               // rad       
  float yaw;                 // rad
  float aim_x;               // ·¢0.5
  float aim_y;               // ·¢0.5
  float aim_z;               // ·¢0.5
  uint16_t checksum;     // crc16Ğ£ÑéÎ» 	
} 	Vision_t; //ÊÓ¾õÍ¨ĞÅ½á¹¹Ìå

<<<<<<< HEAD
//================================================ minipc -> stm32 (½ÓÊÕ½á¹¹Ìå)================================================//
typedef struct
{
  uint8_t header;
	pByte_t official;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum;
} vision_receive_t;

//================================================Ò£¿ØÆ÷¼°¼üÅÌ½âËã½á¹¹Ìå================================================//
=======
>>>>>>> parent of 52bf27f (ä¿®æ”¹äº†å’Œè§†è§‰çš„æ¥å£)
typedef __packed struct
{
		__packed struct
	{
		uint16_t r : 1;//´ÓµÍÎ»¿ªÊ¼
		uint16_t f : 1;
		uint16_t g : 1;
		uint16_t z : 1;
		uint16_t x : 1;
		uint16_t c : 1;
		uint16_t v : 1;
		uint16_t b : 1;
		uint16_t w : 1;
		uint16_t s : 1;
		uint16_t a : 1;
		uint16_t d : 1;
		uint16_t q : 1;
		uint16_t e : 1;
		uint16_t shift : 1;
		uint16_t ctrl : 1;
	} key;
	
		__packed struct
	{
		uint16_t press_left;
		uint16_t press_right;
		uint16_t x;
		uint16_t y;
	}	mouse;
	
} 	remote_flag_t; //¼üÅÌÊı¾İ»ñÈ¡

<<<<<<< HEAD
//================================================µç»ú×·×Ù½á¹¹Ìå================================================//
//´¢´æµÄÊÇÒÑ¾­ÊÓ¾õ½âËã³É¹¦µÄÊÓ¾õÊı¾İ
typedef struct
{
	float yaw;
	float pitch;
} Chase_t;	//´«Êä¸øµç»úµÄÖµ

//================================================¹Ù·½²ÃÅĞÏµÍ³µÄÖµ´æ´¢£¬ÒÔ¼°ÉÚ±øµÄÒ»Ğ©×´Ì¬Á¿================================================//
typedef struct
{
	uint8_t foe_flag;		//ÊÓ¾õ¼ì²â±êÖ¾Î»£¬1Îª¼ì²â³É¹¦£¬0ÎªÎ´¼ì²âµ½×°¼×°å
	uint8_t foe_count;	//ÊÓ¾õÍ£Áô¼ÆÊıÆ÷
	uint8_t Flag_progress;	//²ÃÅĞÏµÍ³±ÈÈü½ø³ÌÊı¾İ
	uint8_t Flag_judge;	//ºìÀ¶·½¼ì²â
} Sentry_t;


=======
extern uint8_t foe_flag;
extern remote_flag_t remote;
	
>>>>>>> parent of 52bf27f (ä¿®æ”¹äº†å’Œè§†è§‰çš„æ¥å£)
extern volatile uint8_t rx_len_uart1;  //½ÓÊÕÒ»Ö¡Êı¾İµÄ³¤¶È
extern volatile uint8_t recv_end_flag_uart1; //Ò»Ö¡Êı¾İ½ÓÊÕÍê³É±êÖ¾

extern Chase_t chase;	//¸³Óèµç»ú×·×ÙµÄÊı¾İ½á¹¹Ìå
extern remote_flag_t remote;	//¼üÅÌ°´¼ü¶ÁÈ¡(½á¹¹Ìå)
extern Sentry_t Sentry;	//ÉÚ±ø×´Ì¬Á¿ºÍ²ÃÅĞÏµÍ³Êı¾İ½á¹¹Ìå
	




#endif