//================================================遥控器数据接收================================================//
#include "remote_control.h"
#include "Exchange_task.h"
#include "iwdg.h"
extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern uint8_t Rx_1[128];
extern uint8_t Rx_2[128];
extern uint8_t Rx_3[128];
extern uint8_t Rx_4[128];
extern Vision_receive_t vision_receive;
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
	
//================================================遥控器数据解析函数================================================//
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥控器控制全局变量
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame, but set 36 bytes 
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
	
//================================================遥控器初始化(使能DMA和串口中断)================================================//
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//================================================遥控器串口接收中断================================================//
//涓插彛涓柇锛岄渶瑕佹坊鍔犲埌stm32f4xx_it.c鐨勪腑鏂腑
void USART3_IRQHandler_remote(void)
{

    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {							
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {	
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);			

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);

            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)//经判断从这里进的
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);


            }
        }
    }
}


/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */

//================================================遥控器数据解析函数================================================//
extern UART_HandleTypeDef huart4;
//extern float target_speed[7];
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0  中值为1024，最大值1684，最小值364，波动范围：660
    rc_ctrl->rc.ch[1] = (((sbus_buf[1] >> 3)&0xff) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = (((sbus_buf[2] >> 6)&0xff) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = (((sbus_buf[4] >> 1)&0xff) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch leftd！！！这尼玛是右
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;    		//!< Switch right！！！这才是左
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value	
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
		
    rc_ctrl->rc.ch[0] -= 1024;//这里把回中值调成0了
    rc_ctrl->rc.ch[1] -= 1024;
    rc_ctrl->rc.ch[2] -= 1024;
    rc_ctrl->rc.ch[3] -= 1024;
    rc_ctrl->rc.ch[4] -= 1024;
		
		
	}

	int debugl = 0;
	void UART5_IRQHandler_remote(void)
{

    if(huart5.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {							
        __HAL_UART_CLEAR_PEFLAG(&huart5);
    }
    else if(UART5->SR & UART_FLAG_IDLE)
    {	
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart5);			

        if (hdma_uart5_rx.Instance->CR & DMA_SxCR_CT)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_uart5_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 19 - hdma_uart5_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_uart5_rx.Instance->NDTR = 19;

            //set memory buffer 1
            //设定缓冲区1
            hdma_uart5_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_uart5_rx);
						debugl = this_time_rx_len;
            if(Rx_1[0] == 0xA5)
            {
                //sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
							memcpy(&vision_receive.R_tracking,&Rx_1[1],1);
							memcpy(&vision_receive.R_shoot,&Rx_1[2],1);
							memcpy(&vision_receive.yaw_R,&Rx_1[3],4);
							memcpy(&vision_receive.R_chase_yaw,&Rx_1[7],4);
							memcpy(&vision_receive.R_chase_pitch,&Rx_1[11],4);
							memcpy(&vision_receive.R_distance,&Rx_1[15],4);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_uart5_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 19 - hdma_uart5_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_uart5_rx.Instance->NDTR = 19;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_uart5_rx);

            if(Rx_2[0] == 0xA5)//经判断从这里进的
            {
                //处理遥控器数据
                //sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
							memcpy(&vision_receive.R_tracking,&Rx_2[1],1);
							memcpy(&vision_receive.R_shoot,&Rx_2[2],1);
							memcpy(&vision_receive.yaw_R,&Rx_2[3],4);
							memcpy(&vision_receive.R_chase_yaw,&Rx_2[7],4);
							memcpy(&vision_receive.R_chase_pitch,&Rx_2[11],4);
							memcpy(&vision_receive.R_distance,&Rx_2[15],4);

            }
        }
    }
}

void UART4_IRQHandler_remote(void)
{

    if(huart4.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {							
        __HAL_UART_CLEAR_PEFLAG(&huart4);
    }
    else if(UART4->SR & UART_FLAG_IDLE)
    {	
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart4);			

        if (hdma_uart4_rx.Instance->CR & DMA_SxCR_CT)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_uart4_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 28 - hdma_uart5_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_uart4_rx.Instance->NDTR = 28;

            //set memory buffer 1
            //设定缓冲区1
            hdma_uart4_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_uart4_rx);
						debugl = this_time_rx_len;
            if(Rx_3[0] == 0xA5)
            {
                //sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
							memcpy(&vision_receive.L_tracking,&Rx_3[1],1);
							memcpy(&vision_receive.L_shoot,&Rx_3[2],1);
							memcpy(&vision_receive.yaw_L,&Rx_3[3],4);
							memcpy(&vision_receive.L_chase_yaw,&Rx_3[7],4);
							memcpy(&vision_receive.L_chase_pitch,&Rx_3[11],4);
							memcpy(&vision_receive.L_distance,&Rx_3[15],4);
							memcpy(&vision_receive.naving,&Rx_3[19],1);
							memcpy(&vision_receive.nav_vx,&Rx_3[20],4);
							memcpy(&vision_receive.nav_vy,&Rx_3[24],4);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_uart4_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 28 - hdma_uart4_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_uart4_rx.Instance->NDTR = 28;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
               
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_uart4_rx);

            if(Rx_4[0] == 0xA5)//经判断从这里进的
            {
                //处理遥控器数据
                //sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
								memcpy(&vision_receive.L_tracking,&Rx_4[1],1);
								memcpy(&vision_receive.L_shoot,&Rx_4[2],1);
								memcpy(&vision_receive.yaw_L,&Rx_4[3],4);
								memcpy(&vision_receive.L_chase_yaw,&Rx_4[7],4);
								memcpy(&vision_receive.L_chase_pitch,&Rx_4[11],4);
								memcpy(&vision_receive.L_distance,&Rx_4[15],4);
								memcpy(&vision_receive.naving,&Rx_4[19],1);
								memcpy(&vision_receive.nav_vx,&Rx_4[20],4);
								memcpy(&vision_receive.nav_vy,&Rx_4[24],4);
							
            }
        }
    }
}

