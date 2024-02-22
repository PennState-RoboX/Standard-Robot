/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

 /**
  * Modified the "RM referee system data solve" into CV_data receiving and processing
  * Commented out the original referee_usart_task function and added the UART7_CommandRoute function
 */

#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "INS_task.h"

#include "bsp_usart.h"
#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"




/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
// static void referee_unpack_fifo_data(void);

 
extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
char dma_buf[50];

// 用于接收的DMA数据缓存
// DMA data buffer for reception
unsigned char rx_data[50];

// define a complete packet
// 定义一个完整的数据包
uint8_t packet[CV_PACKET_LENGTH];

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void referee_usart_task(void const * argument)
{

  memset(dma_buf, 0, 200);
	memset(rx_data, 0, 50);
	const fp32* imu = get_INS_angle_point();
	while(1){
		sprintf((char *)dma_buf, "A5%f,%f,%f", imu[0],imu[1],imu[2]);
	HAL_UART_Transmit_DMA(&huart6, (uint8_t*)dma_buf, strlen((const char*)dma_buf));
	HAL_UART_Receive_DMA(&huart6, (uint8_t*)rx_data, sizeof(rx_data)); 
		
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)  
    {
			//HAL_UART_Receive_DMA(&huart6, (uint8_t*)rx_rdata, 9);
			HAL_UART_Transmit_DMA(&huart6, (uint8_t*)dma_buf, strlen((const char*)dma_buf));
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	if (huart->Instance == USART6)  
  {
		UART7_CommandRoute(); 
		HAL_UART_Receive_DMA(&huart6, (uint8_t*)rx_data, sizeof(rx_data));
		
	}
}

// use to find the first complete packet from rx_data, if find A5 and come with 5A, then 
void find_CpltPacket(void){
  for (size_t i = 0; i < sizeof(rx_data); i++){
    // find A55A
    if (rx_data[i] == 0xA5 && rx_data[i+1] == 0x5A){
      // find FF
      if (rx_data[i+sizeof(packet)-1] == 0xFF){
        // copy the packet
        memcpy(packet, &rx_data[i], sizeof(packet));
      }
    }
  }
}

cv_Data_TypeDef cv_Data;
void UART7_CommandRoute(void){
			// get a complete packet by checking the rx_data
      find_CpltPacket();
      
      HexToFloat yaw;
      HexToFloat pitch;

      // get Yaw
      uint32_t temp_Yaw = (uint32_t)packet[3] << 24 |
                          (uint32_t)packet[4] << 16 |
                          (uint32_t)packet[5] << 8  |
                          (uint32_t)packet[6];
      yaw.hex = temp_Yaw;
      cv_Data.yaw = yaw.floatValue;

      // get Pitch
      uint32_t temp_Pitch = (uint32_t)packet[7] << 24 |
                            (uint32_t)packet[8] << 16 |
                            (uint32_t)packet[9] << 8  |
                            (uint32_t)packet[10];
      pitch.hex = temp_Pitch;
      cv_Data.pitch = pitch.floatValue;
	}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
//void referee_unpack_fifo_data(void)
//{
//  uint8_t byte = 0;
//  uint8_t sof = HEADER_SOF;
//  unpack_data_t *p_obj = &referee_unpack_obj;

//  while ( fifo_s_used(&referee_fifo) )
//  {
//    byte = fifo_s_get(&referee_fifo);
//    switch(p_obj->unpack_step)
//    {
//      case STEP_HEADER_SOF:
//      {
//        if(byte == sof)
//        {
//          p_obj->unpack_step = STEP_LENGTH_LOW;
//          p_obj->protocol_packet[p_obj->index++] = byte;
//        }
//        else
//        {
//          p_obj->index = 0;
//        }
//      }break;
//      
//      case STEP_LENGTH_LOW:
//      {
//        p_obj->data_len = byte;
//        p_obj->protocol_packet[p_obj->index++] = byte;
//        p_obj->unpack_step = STEP_LENGTH_HIGH;
//      }break;
//      
//      case STEP_LENGTH_HIGH:
//      {
//        p_obj->data_len |= (byte << 8);
//        p_obj->protocol_packet[p_obj->index++] = byte;

//        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
//        {
//          p_obj->unpack_step = STEP_FRAME_SEQ;
//        }
//        else
//        {
//          p_obj->unpack_step = STEP_HEADER_SOF;
//          p_obj->index = 0;
//        }
//      }break;
//      case STEP_FRAME_SEQ:
//      {
//        p_obj->protocol_packet[p_obj->index++] = byte;
//        p_obj->unpack_step = STEP_HEADER_CRC8;
//      }break;

//      case STEP_HEADER_CRC8:
//      {
//        p_obj->protocol_packet[p_obj->index++] = byte;

//        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
//        {
//          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
//          {
//            p_obj->unpack_step = STEP_DATA_CRC16;
//          }
//          else
//          {
//            p_obj->unpack_step = STEP_HEADER_SOF;
//            p_obj->index = 0;
//          }
//        }
//      }break;  
//      
//      case STEP_DATA_CRC16:
//      {
//        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
//        {
//           p_obj->protocol_packet[p_obj->index++] = byte;  
//        }
//        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
//        {
//          p_obj->unpack_step = STEP_HEADER_SOF;
//          p_obj->index = 0;

//          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
//          {
//            referee_data_solve(p_obj->protocol_packet);
//          }
//        }
//      }break;

//      default:
//      {
//        p_obj->unpack_step = STEP_HEADER_SOF;
//        p_obj->index = 0;
//      }break;
//    }
//  }
//}


//void USART6_IRQHandler(void)
//{
//    static volatile uint8_t res;
//    if(USART6->SR & UART_FLAG_IDLE)
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart6);

//        static uint16_t this_time_rx_len = 0;

//        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            __HAL_DMA_DISABLE(huart6.hdmarx);
//            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
//            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(huart6.hdmarx);
//            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
//            detect_hook(REFEREE_TOE);
//        }
//        else
//        {
//            __HAL_DMA_DISABLE(huart6.hdmarx);
//            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
//            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
//            __HAL_DMA_ENABLE(huart6.hdmarx);
//            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
//            detect_hook(REFEREE_TOE);
//        }
//    }
//}


