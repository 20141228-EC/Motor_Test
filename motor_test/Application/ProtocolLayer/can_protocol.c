#include "can_protocol.h"

/**
 *  @brief  CAN1 接收数据
 */
void CAN1_rxDataHandler(uint32_t rxId, uint8_t *rxBuf)
{
	
	KT_test_id(rxId,rxBuf);
	DM_test_id(rxId,rxBuf);
	switch (rxId)
	{
//		case 0x0b:
//		{
//			L_Wheel.rx(&L_Wheel, rxBuf);
//			
//			break;
//		}
//		case 0x011://接收ID
//		{
//			Yaw_Motor.rx(&Yaw_Motor, rxBuf);
//			break;
//		}
		case 0x141:
		{
			TEST_KT_motor.get_info(&TEST_KT_motor,rxBuf);
			break;
		}
		case 0x003:
		{
			TEST_DM_Motor.rx(&TEST_DM_Motor,rxBuf);
		}
		break;
		
		case 0x201:
			rm3508.rx(&rm3508,rxBuf);
		  break;
		
		case 0x0B:
			ht.rx(&ht,rxBuf);
		  break;
		
		case 0x206:
		  gm6020.rx(&gm6020,rxBuf);
	    break;
		default:
			break;
	}
}
/**
 *  @brief  CAN2 接收数据
 */
void CAN2_rxDataHandler(uint32_t canId, uint8_t *rxBuf)
{
	
	switch (canId)
	{
		
		default:
			break;
	}
}
