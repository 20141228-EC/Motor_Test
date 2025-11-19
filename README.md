# Motor_Test
## User Guide
- 用途：测试电机

- 目前可用且已测型号：达妙M-J4310，瓴控MG8016E-i6，RM3508，GM6020,海泰8108J6

- 具体食用方法：
1.根据更改base_info结构体的can和电机发送id
2.在can接收函数中更改对应接收id

## 注意事项
1.rm电机额外需更改电机序号0~3
2.GM6020为电压控制，即motorId为0x1FF

## 拓展功能
瓴控Id未知时motorId设为0x140可搜寻id并进行测试