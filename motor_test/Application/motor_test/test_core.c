#include "test_core.h"

MotorTestConfig_t Test_Config={
	.mode=PID_sleep_mode,
	.start_tick=0,
	.test_target_set=true,  //海泰电机速度设置标志位
	.change_angle=0,
};//传入模式等参数

static void single_pid(MotorTestConfig_t *Test_Config);
static void double_pid(MotorTestConfig_t *Test_Config);
static void KT_Id_test(Motor_base_info_t *base_info);
static void KT_Multi_Mode_Updata(uint8_t id,motor_pid_t*pid);
static void DM_Id_test(Motor_base_info_t *base_info);
/*------------------------------------------------------------*/
/*                           DM                               */
/*------------------------------------------------------------*/

static DM_Info_t DM_Info={0};

DM_test_flag_t DM_flag={
	.DM_ID_Get=true,//是否已找到id；
	.ID_UNKNOW=false,//是否需要搜寻ID;
	.Rx_Id=0x00,
	.Tx_Id=0x00,
};

Motor_DM_t TEST_DM_Motor={
	.born_info=&DM_Info.born_info,
	.rx_info=&DM_Info.rx_info,
	.tx_info=&DM_Info.tx_info,
	.state=&DM_Info.state,
	.single_init=DM_Single_Motor_Init,
};

motor_pid_t DM_Angle_Pid={
	.speed.kp = 0.2,
	.speed.ki = 0,
	.speed.kd = 0,
	.speed.integral_max = 3000,
	.speed.out_max = 28000,
	.angle.kp = 20,
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 500,
};

motor_pid_t DM_Speed_Pid={
	.speed.target=5,
	.speed.kp = 0.8,
	.speed.ki = 0,
	.speed.kd = 0,
	.speed.integral_max = 3000,
	.speed.out_max = 28000,
};

pid_struct_t DM_pid={
	.single_pid=&DM_Speed_Pid,
	.double_pid=&DM_Angle_Pid,
};
/*---------------------------------typedef_end------------*/

//主工作函数    Init初始化（更新绑定结构体） work运行计算pid   send发送

void DM_Motor_Init(Motor_base_info_t*info){
	Motor_DM_Born_Info_t  *born_info=TEST_DM_Motor.born_info;
	
	if(info->drive_type==M_CAN1){
		born_info->hcan=&hcan1;
	}
	else if(info->drive_type==M_CAN2){
		born_info->hcan=&hcan2;
	}
	born_info->stdId=(uint8_t)info->motor_Id;
	DM_Id_test(info);
	Test_Config.pid=&DM_pid;
	TEST_DM_Motor.single_init(&TEST_DM_Motor);
	
	Test_Config.angle_range=2*PI;//电机位置默认最大范围
}



void DM_Motor_work(){
	motor_pid_t *speed_pid=Test_Config.pid->single_pid;
	motor_pid_t *angle_pid=Test_Config.pid->double_pid;
	Motor_DM_Rx_Info_t *info=TEST_DM_Motor.rx_info;
	
	if(DM_flag.DM_ID_Get==false){
		TEST_DM_Motor.born_info->stdId++;
		DM_flag.Tx_Id=TEST_DM_Motor.born_info->stdId;
		if(TEST_DM_Motor.born_info->stdId>=0xFF){
			TEST_DM_Motor.born_info->stdId=0x00;
		}
	}
	
	switch(Test_Config.mode){
		case PID_speed_mode:{
			DM_Speed_Pid.speed.target=5*Test_Config.direction;
			speed_pid->speed.measure=info->speed;
			single_pid(&Test_Config);
			TEST_DM_Motor.tx_info->torque=speed_pid->speed.out;
			break;
		}
		case PID_angle_mode:{
			angle_pid->angle.target=-PI+(Test_Config .angle_range*Test_Config.change_angle/4);
			angle_pid->angle.measure=info->motor_angle;
			angle_pid->speed.measure=info->speed;
			double_pid(&Test_Config);
			TEST_DM_Motor.tx_info->torque=angle_pid->speed.out;
		}
		default:
			break;
	}
	if(TEST_DM_Motor.state->status==DEV_OFFLINE||Test_Config.mode==PID_sleep_mode){
		TEST_DM_Motor.single_sleep(&TEST_DM_Motor);
	}
}



void DM_Motor_send(){
	TEST_DM_Motor.single_set_torque(&TEST_DM_Motor);
}

void DM_test_id(uint32_t rxId,uint8_t *rxBuf){
	if(DM_flag.ID_UNKNOW==true){
		DM_flag.Rx_Id=rxId;
		DM_flag.DM_ID_Get=true;
		TEST_DM_Motor.rx(&TEST_DM_Motor,rxBuf);
	}
 }

/*------------------------------------------------------------*/
/*                           KT                               */
/*------------------------------------------------------------*/
KT_motor_t TEST_KT_motor={
	.init=KT_motor_class_init,
};

motor_pid_t KT_Angle_Pid={
	.speed.kp = 0.5,
	.speed.ki = 0,
	.speed.kd = 0,
	.speed.integral_max = 3000,
	.speed.out_max = 1000,
	.angle.kp = 0.8,
	.angle.ki = 0,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 500,
};

motor_pid_t KT_Speed_Pid={
	.speed.target=200,
	.speed.kp = 0.5,
	.speed.ki = 0,
	.speed.kd = 0,
	.speed.integral_max = 3000,
	.speed.out_max = 1000,
};

KT_test_flag_t KT_flag={
	.KT_ID_Get=true,//是否已找到id；
	.ID_UNKNOW=false,//是否需要搜寻ID;
	.board_mode=false,
	.Id=0x140,
	.iqControl={0},
};

pid_struct_t KT_pid={
	.single_pid=&KT_Speed_Pid,
	.double_pid=&KT_Angle_Pid,
};

void KT_Motor_Init(Motor_base_info_t*info){
	TEST_KT_motor.init(&TEST_KT_motor);
	KT_motor_id_info_t *KT_info=&TEST_KT_motor.KT_motor_info.id;
	
	KT_info->tx_id=info->motor_Id;
	KT_Id_test(info);
	KT_info->drive_type=info->drive_type;
	Test_Config.pid=&KT_pid;
	TEST_KT_motor.tx_W_cmd(&TEST_KT_motor,MOTOR_RUN_ID);
	Test_Config.angle_range=65535;//KT角度范围
}


void KT_Motor_work(){
	static uint8_t id;
	motor_pid_t *speed_pid=Test_Config.pid->single_pid;
	motor_pid_t *angle_pid=Test_Config.pid->double_pid;
	KT_motor_rx_info_t *info=&TEST_KT_motor.KT_motor_info.rx_info;
	
	if(KT_flag.KT_ID_Get==false){
		TEST_KT_motor.KT_motor_info.id.tx_id++;
		if(TEST_KT_motor.KT_motor_info.id.tx_id>=0x160){
			TEST_KT_motor.KT_motor_info.id.tx_id=0x140;
		}
	}
	//多电机模式收到id后处理id序号
	if(KT_flag.KT_ID_Get&&KT_flag.board_mode){
		id=KT_flag.Id&0x0f-1;
	}
	
	
			switch(Test_Config.mode){
		case PID_speed_mode:{
			speed_pid->speed.target=500*Test_Config.direction;
			speed_pid->speed.measure=info->speed;
			single_pid(&Test_Config);
			static int16_t iqContro;
			iqContro=(int16_t)speed_pid->speed.out;
			TEST_KT_motor.W_iqControl(&TEST_KT_motor,iqContro);
			
			KT_Multi_Mode_Updata(id,speed_pid);//多电机
			break;
		}
		case PID_angle_mode:{
			angle_pid->angle.target=Test_Config .angle_range*Test_Config.change_angle/4;
			angle_pid->angle.measure=info->encoder;
			angle_pid->speed.measure=info->speed;
			double_pid(&Test_Config);
			static int16_t iqContro;
			iqContro=(int16_t)angle_pid->speed.out;
			TEST_KT_motor.W_iqControl(&TEST_KT_motor,iqContro);	
			
			KT_Multi_Mode_Updata(id,angle_pid);//多电机
		}
		default:
			break;
	}
}

void KT_Motor_send(){
		KT_motor_state_info_t *state=&TEST_KT_motor.KT_motor_info.state_info;//实际上并没有用上心跳函数
	if(KT_flag.board_mode){
		if(!KT_flag.KT_ID_Get ||Test_Config.mode==PID_sleep_mode){
			static int16_t zero[4]={0};
			kt_motor_multi_control(zero,4,TEST_KT_motor.KT_motor_info.id.drive_type);
		}
			else{
			kt_motor_multi_control(KT_flag.iqControl,4,TEST_KT_motor.KT_motor_info.id.drive_type);
		}
	}
	else{
			if(state->work_state==M_OFFLINE ||Test_Config.mode==PID_sleep_mode){
				TEST_KT_motor.tx_W_cmd(&TEST_KT_motor,MOTOR_STOP_ID);
		}
			else{
				TEST_KT_motor.tx_W_cmd(&TEST_KT_motor,TORQUE_CLOSE_LOOP_ID);
		}
	}
}

void KT_test_id(uint32_t rxId,uint8_t *rxBuf){
	if(KT_flag.ID_UNKNOW==true){
		if(KT_flag.board_mode){
			KT_flag.KT_ID_Get=true;
			KT_flag.Id=rxId;
			TEST_KT_motor.get_info(&TEST_KT_motor,rxBuf);
		}
		else{
			if(rxId==TEST_KT_motor.KT_motor_info.id.tx_id){
				KT_flag.KT_ID_Get=true;
				KT_flag.Id=rxId;
				TEST_KT_motor.get_info(&TEST_KT_motor,rxBuf);
			}
		}
 }
}
/*-----------------------------------------------------------------------*/
/*                               Servo                                   */
/*-----------------------------------------------------------------------*/
//GM6020舵机模式
void Servo_work(Motor_base_info_t*info){
	switch (Test_Config.mode){
		case PID_sleep_mode:{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,TEST_PWM_MIN);
		}
		case PID_angle_mode:
		case PID_speed_mode:{
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,TEST_PWM_MIDDLE);
		}
		default:
			break;
	}
}



/*------------------------------------------------------------*/
/*                          RM                              */
/*------------------------------------------------------------*/
//3508
Motor_RM_Born_Info_t Born3508;

Motor_RM_Tx_Info_t Tx;

Motor_RM_State_t State;

Motor_RM_Rx_Info_t Rx;

pid_ctrl_t Speed_Ctrl = 
{
	.target=300,    //set target
	
	.kp = 5.f,
	.ki = 0.1f,
	.kd = 0.09f,
	.integral_max = 6000.f,
	.out_max = 8000.f
};
pid_ctrl_t Angle_Ctrl_out_3508 = 
{
	.kp = 0.9f, 
	.ki = 0.001f,
	.kd = 0.f,
	.integral_max = 6000.f,
	.out_max = 8000.f,
};

pid_ctrl_t Angle_Ctrl_in_3508 = 
{
	.target=100,
	
	.kp = 1.3f, 
	.ki = 0.001f,
	.kd = 0.0005f,
	.integral_max = 6000.f,
	.out_max = 8000.f,
};

Motor_RM_Ctrl_Info_t Ctrl = 
{
	.speed_ctrl = &Speed_Ctrl,
	.angle_ctrl_outer=&Angle_Ctrl_out_3508,
	.angle_ctrl_inner=&Angle_Ctrl_in_3508,
	.Nearest_Return = true
};

Motor_RM_t rm3508 = 
{
	.born_info = &Born3508,
	
	.rx_info = &Rx,
	
	.tx_info = &Tx,

  .state = &State,
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &Ctrl,
};

//6020
Motor_RM_Born_Info_t Born_6020;

Motor_RM_Tx_Info_t Tx_6020;

Motor_RM_State_t State_6020;

Motor_RM_Rx_Info_t Rx_6020;

pid_ctrl_t Speed_Ctrl6020 = 
{
	.target=500,
	
	.kp = 8.0f, 
	.ki = 0.005f,
	.kd = 0.001f,
	.integral_max = 6000.f,
	.out_max = 8000.f,
};

pid_ctrl_t Angle_Ctrl_out_6020 = 
{
	.target=100,
	
	.kp = 1.2f, 
	.ki = 0.005f,
	.kd = 0.001f,
	.integral_max = 6000.f,
	.out_max = 8000.f,
};

pid_ctrl_t Angle_Ctrl_in_6020 = 
{
	
	.kp = 12.0f, 
	.ki = 0.001f,
	.kd = 0.0f,
	.integral_max = 6000.f,
	.out_max = 8000.f,
};
Motor_RM_Ctrl_Info_t Ctrl6020 = 
{
	.speed_ctrl = &Speed_Ctrl6020,
	.angle_ctrl_outer=&Angle_Ctrl_out_6020,
	.angle_ctrl_inner=&Angle_Ctrl_in_6020,
	.Nearest_Return = true
};

Motor_RM_t gm6020 = 
{
	.born_info = &Born_6020,
	
	.rx_info = &Rx_6020,
	
	.tx_info = &Tx_6020,

  .state = &State_6020,
	
	.single_init = RM_Motor_Init,
	
	.ctrl = &Ctrl6020,
};




void m_RM_Motor_Init(Motor_base_info_t*info){
	if(info->motor_type==RM3508)
	{
			Motor_RM_Born_Info_t  *born_info=rm3508.born_info;

			if(info->drive_type==M_CAN1){
			born_info->hcan=&hcan1;
			}
			else if(info->drive_type==M_CAN2){
			born_info->hcan=&hcan2;
			}
			born_info->stdId=info->motor_Id;
			born_info->type=_3508_Single;
			born_info->rxId=info->rxId;
			rm3508.single_init(&rm3508);
	
   }
  else if(info->motor_type==GM6020)
	{
			Motor_RM_Born_Info_t  *born_info=gm6020.born_info;
			
			if(info->drive_type==M_CAN1){
				born_info->hcan=&hcan1;
			}
			else if(info->drive_type==M_CAN2){
				born_info->hcan=&hcan2;
			}
			born_info->stdId=info->motor_Id;
			born_info->type=_6020_Single;
			gm6020.single_init(&gm6020);
			born_info->rxId=info->rxId;
   }
	 	Test_Config.angle_range=8191;//电机位置默认最大范围
}



void RM_Motor_work(Motor_base_info_t*info){
	switch(Test_Config.mode){
		case PID_speed_mode:{
			Speed_Ctrl6020.target=500*Test_Config.direction;
			Speed_Ctrl.target=300*Test_Config.direction;
				if(info->motor_type==RM3508)
	         {rm3508.single_set_speed(&rm3508);}
				  else if(info->motor_type==GM6020)
	         {gm6020.single_set_speed(&gm6020);}
			break;
		}
		case PID_angle_mode:{
				if(info->motor_type==RM3508)
       	{Angle_Ctrl_out_3508.target=Test_Config .angle_range*Test_Config.change_angle/4;
		rm3508.single_set_angle(&rm3508);}
        else if(info->motor_type==GM6020)
       	{	Angle_Ctrl_out_6020.target=Test_Config .angle_range*Test_Config.change_angle/4;	
			gm6020.single_set_angle(&gm6020);
				 gm6020.tx_info->torque=gm6020.ctrl->angle_ctrl_inner->out ;
				}
				break;
		}
		case PID_sleep_mode:{
				if(info->motor_type==RM3508)
       	{rm3508.single_sleep(&rm3508);}
        else if(info->motor_type==GM6020)
       	{gm6020.single_sleep(&gm6020);}
		}
			
			break;
		default:
			break;
	}
}



void RM_Motor_send(Motor_base_info_t*info){
	if(info->motor_type==RM3508)
	{	rm3508.single_set_torque(&rm3508);}
	else if(info->motor_type==GM6020)
	{gm6020.single_set_torque(&gm6020);}

}



/*------------------------------------------------------------*/
/*                           HT                               */
/*------------------------------------------------------------*/
Motor_HT_Rx_Info_t Rx_Info_t;
Motor_HT_Tx_Info_t Tx_Info_t;
Motor_HT_State_t State_t;
Motor_HT_Born_Info_t Born_Info;
Motor_HT_t ht = 
{
	.born_info = &Born_Info,
	
	.rx_info = &Rx_Info_t,
	
	.tx_info = &Tx_Info_t,
	
	.state = &State_t,
	
	.single_init = &HT_Single_Motor_Init,
};
motor_pid_t HT_Angle_Pid={
	.speed.kp = 0.5,
	.speed.ki = 0.0005,
	.speed.kd = 0,
	.speed.integral_max = 300,
	.speed.out_max = 2800,
	.angle.kp = 2.5,
	.angle.ki = 0.0004,
	.angle.kd = 0,
	.angle.integral_max = 0,
	.angle.out_max = 400,
};

motor_pid_t HT_Speed_Pid={
	.speed.target=3,
	.speed.kp = 0.7,
	.speed.ki = 0.006,
	.speed.kd = 0,
	.speed.integral_max = 300,
	.speed.out_max = 2800,
};

pid_struct_t HT_pid={
	.single_pid=&HT_Speed_Pid,
	.double_pid=&HT_Angle_Pid,
};
/*---------------------------------typedef_end------------*/

//主工作函数    Init初始化（更新绑定结构体） work运行计算pid,发送控制命令

void HT_Motor_Init(Motor_base_info_t*info){
	Motor_HT_Born_Info_t  *born_info=ht.born_info;
	
	if(info->drive_type==M_CAN1){
		born_info->hcan=&hcan1;
	}
	else if(info->drive_type==M_CAN2){
		born_info->hcan=&hcan2;
	}
	born_info->stdId=(uint8_t)info->motor_Id;
	born_info->order_correction=1;
	Test_Config.pid=&HT_pid;
	ht.single_init(&ht);
	Test_Config.angle_range=191.0f;//电机位置默认最大范围
}



void HT_Motor_work(){
	motor_pid_t *speed_pid=Test_Config.pid->single_pid;
	motor_pid_t *angle_pid=Test_Config.pid->double_pid;
	Motor_HT_Rx_Info_t *info=ht.rx_info;
	
	switch(Test_Config.mode){
		case PID_speed_mode:{
			speed_pid->speed.target=3*Test_Config.direction;
			speed_pid->speed.measure=info->speed;
			single_pid(&Test_Config);
			ht.tx_info->torque=speed_pid->speed.out;
			ht.single_set_torque(&ht);
			break;
		}
		case PID_angle_mode:{
			angle_pid->angle.measure=info->encoder;
			if(Test_Config.test_target_set)
			{
			angle_pid->angle.target=(info->encoder)+PI/2;   //测试的时候在原有基础上转1/4圈
			Test_Config.test_target_set=false;	
			}
			angle_pid->speed.measure=info->speed;
			double_pid(&Test_Config);
			ht.tx_info->torque=angle_pid->speed.out;
			ht.single_set_torque(&ht);
		}
		case PID_sleep_mode:
		{
			ht.single_sleep(&ht);
		}
		default:
			break;
	}


	
}




/*-------------------------函数工具---------------------*/
//单环pid计算
static void single_pid(MotorTestConfig_t *Test_Config){
	pid_ctrl_t *pid=&Test_Config->pid->single_pid->speed;
	
	pid->err=pid->target-pid->measure;
	single_pid_ctrl(pid);
}

//双环pid
static void double_pid(MotorTestConfig_t *Test_Config){
	pid_ctrl_t *out_pid=&Test_Config->pid ->double_pid->angle;
	pid_ctrl_t *inn_pid=&Test_Config->pid ->double_pid->speed;
	
	out_pid->err=out_pid->target-out_pid->measure;
	out_pid->err=motor_half_cycle(out_pid->err,Test_Config->angle_range);
	single_pid_ctrl(out_pid);
	inn_pid->target=out_pid->out;
	inn_pid->err=inn_pid->target-inn_pid->measure;
	single_pid_ctrl(inn_pid);
	
}

static void KT_Id_test(Motor_base_info_t *base_info){//放在init，判断开始需不需要试id
	if(base_info->KT_ID_Unkonw){
		KT_flag.KT_ID_Get=false;
		KT_flag.ID_UNKNOW=true;
		if(base_info->KT_Board_Mode){
			KT_flag.board_mode=true;
		}
	}
}


static void DM_Id_test(Motor_base_info_t *base_info){//放在init，判断开始需不需要试id
	if(base_info->DM_ID_Unknow){
		DM_flag.DM_ID_Get=false;
		DM_flag.ID_UNKNOW=true;
	}
}

static void KT_Multi_Mode_Updata(uint8_t id,motor_pid_t*pid){
	KT_flag.iqControl[id]=(int16_t)(pid->speed.out);
}
