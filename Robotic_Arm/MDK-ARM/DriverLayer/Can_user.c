#include "stm32f4xx.h"                  // Device header
#include "Can_user.h"
#include "can.h"
#include "bsp_can.h"

motor_info_t motor_info[4];
 
uint8_t Data_Enable[8]     ={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//电机使能命令
uint8_t Data_Failure[8]    ={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};		//电机失能命令
uint8_t Data_Save_zero[8]  ={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};	  //电机保存零点命令
uint8_t Data_Error_clear[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};	  //电机清除错误

//============================电机基础命令===============================================

//电机命令
/*
    Data_Enable        电机使能命令
		Data_Failure			 电机失能命令
		Data_Save_zero     电机保存零点命令
		Data_Error_clear   电机清除错误
*/
void motor_commend(motor_info_t motor,uint8_t *pData)
{
	if(motor.mode== 0)  //MIT模式
	{
		CANx_SendStdData(&motor.hcan,motor.can_id,pData,8);	
		HAL_Delay(10);
	}
	if(motor.mode== 1)  //位置速度模式
	{
		CANx_SendStdData(&motor.hcan,motor.can_id + 0x100,pData,8);	
		HAL_Delay(10);
	}
	if(motor.mode== 2) //速度模式
	{
		CANx_SendStdData(&motor.hcan,motor.can_id + 0x200,pData,8);	
		HAL_Delay(10);
	}
}
//============================can相关===============================================
//can滤波器
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}



/**
 * @brief  发送标准ID的数据帧
 * @param  hcan     CAN的句柄
 * @param  ID       数据帧ID
 * @param  pData    数组指针
 * @param  Len      字节数0~8
 */
uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len)
{
  static CAN_TxHeaderTypeDef   Tx_Header;
	
	Tx_Header.StdId=ID;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=0;
	Tx_Header.RTR=0;
	Tx_Header.DLC=Len;
	
	
        /*找到空的发送邮箱，把数据发送出去*/
	if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	{
		if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
        }
    }
    return 0;
}

//======================================================================================


