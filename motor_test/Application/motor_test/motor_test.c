#include "motor_test.h"
/*------------------------------------User Guide------------------------------------------------*/
//作用：闭环控制测试
//测试电机时，更改结构体base_info内容，并在can_protocol.c中更改can接收Id
//接收函数为
//    	达妙： 	 	Test_DM_Motor.rx(&TEST_DM_Motor,rxBuf);
//    	RM：		rm3508.rx(&rm3508,rxBuf);gm6020.rx(&gm6020,rxBuf);
//		HT：		ht.rx(&ht,rxBuf);
//    	KT：		Test_KT_motor.get_info(&TEST_KT_motor,rxBuf);
// 电机会先后进行控速，控位测试，后停止

// 瓴控电机id未知时,motor_Id填入0x140开启id检测，需在回调函数中调用	KT_test_id(rxId,rxBuf);
/*----------------------------------------------------------------------------------------------*/


static void Motor_Operate(Motor_base_info_t *info, MotorOpType_e op);

Motor_base_info_t base_info={
	.motor_type=HT8108 ,
	
	.drive_type=M_CAN1,
		
	.motor_Id=0x009,//发送Id
	
	.rxId=0 ,  //rm电机序号，其他电机无需更改
	
};
/*--------------------手动更改内容end----------------------*/


//调用的主要函数
void Pid_Motor_Init(){
	Motor_Operate(&base_info, MOTOR_OP_INIT);
	Test_Config.start_tick = HAL_GetTick();
}


void Pid_Ctrl(){
	Motor_Operate(&base_info, MOTOR_OP_CTRL);
}


void Pid_Send(){
	Motor_Operate(&base_info, MOTOR_OP_SEND);
}


/*-----------------------------------------*/

//模式切换函数：初始输出为0，一秒后控速，停止一秒后控位,循环
static void Mode_change(){
	if (HAL_GetTick() - Test_Config.start_tick>= 6000) { 
       Test_Config.start_tick=HAL_GetTick();
    }
	else if(HAL_GetTick() - Test_Config.start_tick>= 4000){
		Test_Config.mode=PID_angle_mode;
	}
	else if(HAL_GetTick() - Test_Config.start_tick>= 3000){
		Test_Config.mode=PID_sleep_mode;
	}
	else if(HAL_GetTick() - Test_Config.start_tick>= 1000){
		Test_Config.mode=PID_speed_mode;
	}
}

//电机判断并执行操作
static void Motor_Operate(Motor_base_info_t *info, MotorOpType_e op) {
    if (info == NULL) {
        return; 
    }
	Mode_change();
    switch (info->motor_type) {
        case DM_MOTOR:
            switch (op) {
                case MOTOR_OP_INIT:
                    DM_Motor_Init(info);
                    break;
                case MOTOR_OP_CTRL:
                    DM_Motor_work();
                    break;
                case MOTOR_OP_SEND:
                    DM_Motor_send();
                    break;
            }
            break;
			
        case KT_MOTOR:
            switch (op) {
                case MOTOR_OP_INIT:
                    KT_Motor_Init(info);
                    break;
                case MOTOR_OP_CTRL:
                    KT_Motor_work();
                    break;
                case MOTOR_OP_SEND:
                    KT_Motor_send();
                    break;
			}
						break;
			
			case SERVO :{
					Servo_work(info);}
			    break;
					
			case RM3508:
			case GM6020:
				     switch (op) {
                case MOTOR_OP_INIT:
                    m_RM_Motor_Init(info);
                    break;
                case MOTOR_OP_CTRL:
                    RM_Motor_work(info);
								    break;
                case MOTOR_OP_SEND:
                 
                     RM_Motor_send(info);								
                     break;
							}
				break;
					
			case HT8108:
				  switch (op) {
                case MOTOR_OP_INIT:
                    HT_Motor_Init(info);
                    break;
                case MOTOR_OP_CTRL:
                    HT_Motor_work();
								    break;
                case MOTOR_OP_SEND:
                   															
                    break;
							}
				break;
			
      default:          
       break;
		}
  }
