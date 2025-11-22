# Motor_Test
## User Guide
- 用途：测试电机

- 目前可用且已测型号：达妙M-J4310，瓴控MG8016E-i6，RM3508，GM6020,海泰8108J6

- 具体食用方法：

1.根据需求修改```motor_test\application\motor_test\motor_test.c```中的```Motor_base_info_t base_info```结构体
  
2.修改```motor_test\Application\Protocol```中对应电机的接收Id（电机id已知情况下）

3.修改```motor_test.c```内宏定义可选择使用角度环还是速度环进行电机测试

4.电机id未知可更改```Motor_base_info_t base_info```结构体中bool变量```KT_ID_Unkonw/DM_ID_Unkonw```决定是否开启id搜寻功能

5.id搜寻功能：
  - 可自动搜寻瓴控，达妙电机的id
  - 瓴控id储存在```motor_core.c```的```KT_test_flag_t KT_flag```结构体中
  - 达妙id储存在```motor_core.c```的```DM_test_flag_t DM_flag```结构体中


## 注意事项
1.rm电机额外需更改电机序号0~3

2.GM6020为电压控制，即motorId为0x1FF

3.测试时请控制好电机，以免出现意外情况O w O
