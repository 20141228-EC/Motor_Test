#ifndef __TEST_CONFIG_H
#define __TEST_CONFIG_H

#include "stm32f4xx_hal.h"
#include "math.h"
#include "drv_tim.h"
#include "DM_motor.h"
#include "KT_motor.h"
#include "HT_MOTOR.h"
#include "RM_MOTOR.h"
#include "PID.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim1;
//电机型号motor_type_e
//pid	motor_pid_t


typedef enum {
    PID_speed_mode,
    PID_angle_mode,
	  PID_sleep_mode,
}pid_mode_e;

typedef enum {
	DM_MOTOR,
	KT_MOTOR,
	SERVO,
	GM6020,
	RM3508,
	HT8108,
}test_motor_type_e;

typedef struct {
	motor_pid_t *single_pid;
	motor_pid_t *double_pid;
}pid_struct_t;

// 每种电机的测试配置参数
typedef struct {
	pid_struct_t *pid;
	pid_mode_e  mode;
	float angle_range;  //用于半圈处理，就近归位
	uint32_t start_tick;
	bool test_target_set ; //海泰电机速度设置标志位
	float direction;
	float change_angle;
} MotorTestConfig_t;



typedef struct Motor_struct_t{
    
    test_motor_type_e motor_type;
    
    motor_drive_e drive_type;
    
    uint32_t motor_Id;
	
	 uint8_t rxId;  //rm电机序号

}Motor_base_info_t;


typedef struct {
 Motor_DM_Born_Info_t born_info;
	
Motor_DM_Rx_Info_t rx_info;
	
Motor_DM_Tx_Info_t tx_info;
	
Motor_DM_State_t state;
} DM_Info_t;

typedef struct {
	bool KT_ID_Get;//是否已找到id；
	bool ID_UNKNOW;//是否需要搜寻ID;
	uint32_t Id;
}KT_test_flag_t;

typedef struct {
	bool DM_ID_Get;
	bool ID_UNKNOW;
	uint32_t Rx_Id;
	uint32_t Tx_Id;
}DM_test_flag_t;

typedef enum {
    MOTOR_OP_INIT,   // 初始化操作
    MOTOR_OP_CTRL,   // 控制计算操作
    MOTOR_OP_SEND    // 数据发送操作
} MotorOpType_e;
#endif
