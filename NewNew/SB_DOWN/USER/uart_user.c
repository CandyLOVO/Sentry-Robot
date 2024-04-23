#include "uart_user.h"
#include "rc_potocal.h" //ң���� DBUS USART3
#include "judge.h" //����ϵͳ UART5
#include "string.h"
#include "Exchange_Task.h"
#include "CRC.h"

int value = 0;
int count = 0; //�������ݱ�־λ
uint8_t Rx[128]; //���ջ�������
uint32_t length = 0;
int16_t checksum_Rx;
Rx_naving Rx_nav;

extern uint8_t Rx_flag;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart5_rx;
extern DMA_HandleTypeDef hdma_usart5_tx;
#define USART3_RX_DATA_FRAME_LEN	(18u)	// ����3����֡����
#define USART3_RX_BUF_LEN			(USART3_RX_DATA_FRAME_LEN + 6u)	// ����3���ջ���������
#define UART5_RX_BUF_LEN   (200)
uint8_t usart3_dma_rxbuf[2][USART3_RX_BUF_LEN];
volatile uint8_t judge_dma_buffer[2][UART5_RX_BUF_LEN] ={0}  ;
uint8_t judge_receive_length=0;

void USART1_Init(void)
{
	HAL_GPIO_WritePin(DIR_2_GPIO_Port,DIR_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DIR_1_GPIO_Port,DIR_1_Pin,GPIO_PIN_RESET);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,(uint8_t *)Rx,sizeof(Rx));
}

void DRV_USART1_IRQHandler(UART_HandleTypeDef *huart) //���Ӿ�ͨ�� //��stm32f4xx_it.c�ļ�USART1_IRQHandler����
{
	if(huart->Instance == USART1)
	{
		if(RESET != __HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)){
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
			HAL_UART_DMAStop(&huart1);
			length = 128 - (__HAL_DMA_GET_COUNTER(&hdma_usart1_rx)); //DMA��δ��������ݸ���
			
			Rx_flag = 2; //�յ�һ֡����
			count = 0;
//			memset(Rx,0x00,sizeof(Rx)); //��ջ��棬���½���
			HAL_GPIO_WritePin(DIR_2_GPIO_Port,DIR_2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIR_1_GPIO_Port,DIR_1_Pin,GPIO_PIN_RESET);
			HAL_UART_Receive_DMA(&huart1,(uint8_t *)Rx,sizeof(Rx));
			
			if(Rx[0] == 0xA5)
			{
				checksum_Rx = Get_CRC16_Check_Sum(Rx, 14, 0xffff);
				memcpy(&Rx_nav.checksum, &Rx[14], 2);
				if(Rx_nav.checksum == checksum_Rx)
				{
					Rx_nav.naving = Rx[1];
					memcpy(&Rx_nav.nav_x, &Rx[2], 4);
					memcpy(&Rx_nav.nav_y, &Rx[6], 4);
					memcpy(&Rx_nav.sentry_decision, &Rx[10], 4);
				}
			}
		}
	}
}

void USART3_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);       //��������жϱ�־λ������ֹ�����ж�ʱ��������ж�
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); //ʹ�ܿ����ж�
	
	// Enable the DMA transfer for the receiver request
	SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);	//�����ڶ�Ӧ��DMA��
	
	DMAEx_MultiBufferStart_NoIT(huart3.hdmarx, \
							    (uint32_t)&huart3.Instance->DR, \
							    (uint32_t)usart3_dma_rxbuf[0], \
							    (uint32_t)usart3_dma_rxbuf[1], \
							    USART3_RX_DATA_FRAME_LEN);     //����DMA˫����ģʽ
}

void DRV_USART3_IRQHandler(UART_HandleTypeDef *huart)  //��stm32f4xx_it.c�ļ�USART3_IRQHandler����
{
    // �ж��Ƿ�Ϊ�����ж�
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart); //�����жϻص�����
		value ++;
	}
}

void UART5_Init(void)     //���������жϣ�����DMA��ز�����ʹ��DMA����
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);    //����������жϱ�־λ����ֹ�����ж�ʱ��������ж�
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);//ʹ�ܿ����ж�

	SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR); //�����ڶ�Ӧ��DMA��
	DMAEx_MultiBufferStart_NoIT(huart5.hdmarx, \
							    (uint32_t)&huart5.Instance->DR, \
							    (uint32_t)judge_dma_buffer[0], \
							    (uint32_t)judge_dma_buffer[1], \
							    UART5_RX_BUF_LEN);  
}

void DRV_UART5_IRQHandler(UART_HandleTypeDef *huart)  //��stm32f4xx_it.c�ļ�UART5_IRQHandler����   
{
	if( __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))  //�ж��Ƿ�Ϊ�����ж�
	{
		uart_rx_idle_callback(huart);
	}
}

static void uart_rx_idle_callback(UART_HandleTypeDef* huart) 
{
    __HAL_UART_CLEAR_IDLEFLAG(huart);	
	/* handle received data in idle interrupt */
		 if(huart == &huart3)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* handle dbus data dbus_buf from DMA */
		//uint32_t status = taskENTER_CRITICAL_FROM_ISR();
		if ((USART3_RX_BUF_LEN - huart->hdmarx->Instance->NDTR) == USART3_RX_DATA_FRAME_LEN)
		{
			if(huart->hdmarx->Instance->CR & DMA_SxCR_CT)
				huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
			else
				huart->hdmarx->XferCpltCallback(huart->hdmarx);
		}
	
		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, USART3_RX_BUF_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);	  
	}
  
  
    if(huart == &huart5)
	{
		__HAL_DMA_DISABLE(huart->hdmarx);
		judge_receive_length = UART5_RX_BUF_LEN - huart->hdmarx->Instance->NDTR;
	
		if(huart->hdmarx->Instance->CR & DMA_SxCR_CT)
			huart->hdmarx->XferM1CpltCallback(huart->hdmarx);
		else
			huart->hdmarx->XferCpltCallback(huart->hdmarx);
		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, UART5_RX_BUF_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);	  
	}
}



static void dma_m0_rxcplt_callback(DMA_HandleTypeDef *hdma)
{

		if(hdma== huart3.hdmarx)
		{
				hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);	 // ����ǰĿ���ڴ�����ΪMemory1
				USART3_rxDataHandler(usart3_dma_rxbuf[0]);
		}
			
		else if(hdma == huart5.hdmarx)
		{
			hdma->Instance->CR |= (uint32_t)(DMA_SxCR_CT);	 // ����ǰĿ���ڴ�����ΪMemory1
			JUDGE_Receive(judge_dma_buffer[0],judge_receive_length);
		}

}


static void dma_m1_rxcplt_callback(DMA_HandleTypeDef *hdma)
{
	
	if(hdma== huart3.hdmarx)
	{
		hdma->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);	 // ����ǰĿ���ڴ�����ΪMemory0
		USART3_rxDataHandler(usart3_dma_rxbuf[1]);
	}
	
	else if(hdma == huart5.hdmarx)
	{
		hdma->Instance->CR &=~ (uint32_t)(DMA_SxCR_CT);	 // ����ǰĿ���ڴ�����ΪMemory0
		JUDGE_Receive(judge_dma_buffer[1],judge_receive_length);
	}
}


static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef *hdma, \
                                                    uint32_t SrcAddress, \
                                                    uint32_t DstAddress, \
                                                    uint32_t SecondMemAddress, \
                                                    uint32_t DataLength)//HAL��stm32f4xx_hal_dma_ex.c���и����ƺ���HAL_DMAEx_MultiBufferStart_IT,���������޸ģ��ر���DMA���ж�
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Memory-to-memory transfer not supported in double buffering mode */
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
		hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
		return HAL_ERROR;
    }   

	/* Set the UART DMA transfer complete callback */
	/* Current memory buffer used is Memory 1 callback */
	hdma->XferCpltCallback   = dma_m0_rxcplt_callback;
	/* Current memory buffer used is Memory 0 callback */
	hdma->XferM1CpltCallback = dma_m1_rxcplt_callback;	

	/* Check callback functions */
	if ((NULL == hdma->XferCpltCallback) || (NULL == hdma->XferM1CpltCallback))
	{
	hdma->ErrorCode = HAL_DMA_ERROR_PARAM;
	return HAL_ERROR;
	}
	
	/* Process locked */
	__HAL_LOCK(hdma);
	
	if(HAL_DMA_STATE_READY == hdma->State)
	{	
		/* Change DMA peripheral state */
		hdma->State = HAL_DMA_STATE_BUSY;

		/* Initialize the error code */
		hdma->ErrorCode = HAL_DMA_ERROR_NONE;

		/* Enable the Double buffer mode */
		hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;

		/* Configure DMA Stream destination address */
		hdma->Instance->M1AR = SecondMemAddress;		

		/* Configure DMA Stream data length */
		hdma->Instance->NDTR = DataLength;		
		
		/* Peripheral to Memory */
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{   
			/* Configure DMA Stream destination address */
			hdma->Instance->PAR = DstAddress;

			/* Configure DMA Stream source address */
			hdma->Instance->M0AR = SrcAddress;
		}
		/* Memory to Peripheral */
		else
		{
			/* Configure DMA Stream source address */
			hdma->Instance->PAR = SrcAddress;

			/* Configure DMA Stream destination address */
			hdma->Instance->M0AR = DstAddress;
		}		
		
		/* Clear TC flags */
		__HAL_DMA_CLEAR_FLAG (hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
		/* Enable TC interrupts*/
//		hdma->Instance->CR  |= DMA_IT_TC;
		
		/* Enable the peripheral */
		__HAL_DMA_ENABLE(hdma); 
	}
	else
	{
		/* Process unlocked */
		__HAL_UNLOCK(hdma);	  

		/* Return error status */
		status = HAL_BUSY;		
	}
	/* Process unlocked */
	__HAL_UNLOCK(hdma);

	return status; 	
}
