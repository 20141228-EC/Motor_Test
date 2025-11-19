#ifndef __MOTOR_TEST_H
#define __MOTOR_TEST_H

#include "test_core.h"

void Pid_Motor_Init(void);
void Pid_Ctrl(void);
void Pid_Send(void);
extern Motor_base_info_t base_info;
#endif
