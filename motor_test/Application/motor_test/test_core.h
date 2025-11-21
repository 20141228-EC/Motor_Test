#ifndef __TEST_CORE_H
#define __TEST_CORE_H

#include "test_config.h"

#define TEST_PWM_MAX  1920
#define TEST_PWM_MIN  1080
#define TEST_PWM_MIDDLE 1500

extern MotorTestConfig_t Test_Config;


extern Motor_DM_t TEST_DM_Motor;
extern DM_test_flag_t DM_flag;
void DM_Motor_Init(Motor_base_info_t*info);
void DM_Motor_work(void);
void DM_Motor_send(void);


extern KT_motor_t TEST_KT_motor;
extern KT_test_flag_t KT_flag;
void KT_Motor_Init(Motor_base_info_t*info);
void KT_Motor_work(void);
void KT_Motor_send(void);
void KT_test_id(uint32_t rxId,uint8_t *rxBuf);

extern Motor_RM_t rm3508;
extern Motor_RM_t gm6020;
void m_RM_Motor_Init(Motor_base_info_t*info);
void RM_Motor_work(Motor_base_info_t*info);
void RM_Motor_send(Motor_base_info_t*info);


extern Motor_HT_t ht;
void HT_Motor_Init(Motor_base_info_t*info);
void HT_Motor_work(void);


void Servo_work(Motor_base_info_t*info);
#endif
