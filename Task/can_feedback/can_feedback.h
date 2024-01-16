#ifndef CAN_FEEDBACK_H
#define CAN_FEEDBACK_H



#define get_motor_measure(ptr, rx_message)                                               \
{                                                                                        \
    if((ptr)->ecd - (ptr)->last_ecd > 4096) (ptr)->count-- ;                             \
		else if((ptr)->ecd - (ptr)->last_ecd < -4096 ) (ptr)->count ++ ;										 \
    (ptr)->last_ecd = (ptr)->ecd;                                                        \
    (ptr)->ecd = (uint16_t)((rx_message).Data[0] << 8 | (rx_message).Data[1]);           \
    (ptr)->speed_rpm = (uint16_t)((rx_message).Data[2] << 8 |(rx_message).Data[3]);      \
    (ptr)->given_current = (uint16_t)((rx_message).Data[4] << 8 | (rx_message).Data[5]); \
    (ptr)->temperate = (rx_message).Data[6];                                             \
    (ptr)->all_ecd=(ptr)->count*8191+(ptr)->ecd;                                         \
}




#include "stdint.h"


#define ID_3508 0x201
#define ID_2006 0x202
#define ID_hos1 0x212


#define motor_stdid         0x200
#define master_stdid        0x107

#include "pid.h"


typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;  //当前 -- current
    int32_t  all_ecd;
    int32_t  count;
    uint8_t temperate;

    int16_t last_ecd;
	
} motor_measure_t;



typedef struct
{
	int16_t ch[4];
	uint8_t s[2];
}RC_contr;

union rc_data
{
	uint16_t roctrl_data;
	float value;
};

typedef struct
{
	float target_rpm;      //目标
	float target_angle;   //目标角度
	
	PidType pid_param_speed;
	PidType pid_param_angle;
	
	motor_measure_t feedback; //反馈
}MOTOR;


typedef struct
{
	float angle;
	
	MOTOR motor_3508;
	MOTOR motor_2006;
	MOTOR mt6816;
	RC_contr rc_contr;
}CHASSIS;


typedef struct
{  
  uint16_t Data[8];
} CanRxMsg;

void can_motor_send(uint16_t stid, int i1,int i2,int i3,int i4);
void get_host_measure_1(RC_contr *ptr,CanRxMsg rx_message);
void get_host_measure_2(RC_contr *ptr,CanRxMsg rx_message);
#endif
