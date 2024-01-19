#include "motor_task.h"
#include "cmsis_os.h"
#include "math.h"
#include "spi.h"
#include "gpio.h"

extern int my_start_all_ecd;
#define speed_enlarge 8

extern CHASSIS chassis_;
extern osThreadId MT6816TaskHandle;
extern osThreadId defaultTaskHandle;

void StartDefaultTask(void const * argument);

void CHASSIS_TASK(void)
{
	
	
	
	
}







	uint8_t data_t[3]={0x83, 0x84, 0x85}; 
	uint8_t data_r[4]={0};  //SPI接收数据，即角度数据
	uint16_t i;
	int angle;
	int angle____set = 274;
	int angle_slove;
	int my_start_all_ecd;
	int flag_2006_all_ecd = 0;
	int all_ecd_2006_first;
void MT6816_Task(void const * argument)
{
//	portENTER_CRITICAL();

	while(1){
		
	if(angle != angle____set){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				
			HAL_SPI_TransmitReceive(&hspi1, data_t,data_r, 4, 1000);

			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

			i=	(data_r[1]<<6)|(data_r[2]>>2);
			angle = i*1.0/16384.0*360.0;   //先确定angle是否正确
			
			all_ecd_2006_first = chassis_.motor_2006.feedback.all_ecd;
			
			angle_slove = -(angle____set - angle) * (180/3.14159) * (572839.0/180) + all_ecd_2006_first; 
		
			
			my_start_all_ecd = chassis_.motor_2006.feedback.all_ecd ;
		//	flag_2006_all_ecd = 1;

    	PID_calc(&chassis_.mt6816.pid_param_angle , chassis_.motor_2006.feedback.all_ecd   ,  angle_slove);	
	    PID_calc(&chassis_.mt6816.pid_param_speed , chassis_.motor_2006.feedback.speed_rpm ,  chassis_.mt6816.pid_param_angle.out);

		can_motor_send(motor_stdid,0,chassis_.mt6816.pid_param_speed.out,0,0);
	}
	else{
			osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
		  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

		 vTaskDelete(MT6816TaskHandle);
	 
	}
		
 

		vTaskDelay (1);

	}
	
}




	int x=0,y=0,z=0,c=500;

int Delay_Task = 1;
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {	
		chassis_mode(&chassis_);
		chassis_rc_slove(&chassis_);
		chassis_behavier_get(&chassis_);
		if(chassis_.rc_contr.s[0] != 2 || chassis_.rc_contr.s[1] != 2)
			can_motor_send(motor_stdid,chassis_.motor_3508.pid_param_speed.out,chassis_.motor_2006.pid_param_speed.out,0,0);
    vTaskDelay(Delay_Task);
  }
  /* USER CODE END StartDefaultTask */
}

void system_init(void)
{
	PID_init(&chassis_.motor_3508.pid_param_speed,PID_POSITION,10.0f,1.0f,0.0f,15000.0f,800.0f,500.0f);
	
	PID_init(&chassis_.motor_2006.pid_param_speed,PID_POSITION,9.0f,0.0f,5.0f,9000.0f,800.0f,5.0f);
	PID_init(&chassis_.motor_2006.pid_param_angle,PID_POSITION,0.4f,0.0f,1.5f,8192.0f*36.0f*4.0f,800.0f,5.0f);
	
	PID_init(&chassis_.mt6816.pid_param_speed,PID_POSITION,9.0f,0.0f,5.0f,9000.0f,800.0f,5.0f);
	PID_init(&chassis_.mt6816.pid_param_angle,PID_POSITION,0.0042f,0.0f,0.003f,8000.0f*36.0f*4.0f,800.0f,5.0f);	
	
}

void chassis_mode(CHASSIS *_chassis)
{
	if(_chassis->rc_contr.s[0] == 2 && _chassis->rc_contr.s[1] == 2)
		can_motor_send(motor_stdid,0,0,0,0);	
}

int speed_set = 800;
int angle_set = 2500;
double cos45 = 0.707106;
void chassis_rc_slove(CHASSIS *_chassis)
{
//	_chassis->motor_3508.target_rpm = speed_set;
//	_chassis->motor_2006.target_angle = angle_set;
	
	float vx,vy,vw;
	
	vx = _chassis->rc_contr.ch[2];
	vy = _chassis->rc_contr.ch[3];
	vw = _chassis->rc_contr.ch[0];

	//向后走
	if(vy < 0 && (vx <= 2 && vx >= -2) && vw == 0)
	{
		_chassis->motor_3508.target_rpm = - sqrtf(powf(vx+cos45*vw,2) + powf(vy-cos45*vw,2)) * speed_enlarge;	

		_chassis->motor_2006.target_angle = my_start_all_ecd;
	
	}//后右
	else if(atan2(vx+cos45*vw,vy-cos45*vw)*(180/3.14159) > 92 )//要改数值可能
	{
		_chassis->motor_3508.target_rpm = - sqrtf(powf(vx+cos45*vw,2) + powf(vy-cos45*vw,2)) * speed_enlarge;	


		_chassis->motor_2006.target_angle = - (atan2f(vx+cos45*vw,vy-cos45*vw)*(180/3.14159) - 180)*(572839.0/180) + my_start_all_ecd;



	}//后左
	else if( -atan2(vx+cos45*vw,vy-cos45*vw)*(180/3.14159) > 92	)
	{

		_chassis->motor_3508.target_rpm = - sqrtf(powf(vx+cos45*vw,2) + powf(vy-cos45*vw,2)) * speed_enlarge;	


		_chassis->motor_2006.target_angle = - (atan2f(vx+cos45*vw,vy-cos45*vw)*(180/3.14159) + 180)*(572839.0/180) + my_start_all_ecd;

	}
	else if(vw > 0 && vx == 0 && vy == 0)//右旋
	{

		_chassis->motor_3508.target_rpm = - sqrtf(powf(vx+cos45*vw,2) + powf(vy-cos45*vw,2)) * speed_enlarge;	


		_chassis->motor_2006.target_angle = - (atan2f(vx+cos45*vw,vy-cos45*vw)*(180/3.14159) - 180)*(572839.0/180) + my_start_all_ecd;

	}
	else if(vw < 0 && vx == 0 && vy == 0)//左旋
	{

		_chassis->motor_3508.target_rpm =   sqrtf(powf(vx+cos45*vw,2) + powf(vy-cos45*vw,2)) * speed_enlarge;	


		_chassis->motor_2006.target_angle = - atan2f(vx+cos45*vw,vy-cos45*vw)*(572839.0/180)*(180/3.14159) + my_start_all_ecd;

	}
	else{
                                                                                                                       
	  _chassis->motor_3508.target_rpm =   sqrtf(powf(vx+cos45*vw,2) + powf(vy-cos45*vw,2)) * speed_enlarge;	


		_chassis->motor_2006.target_angle = - atan2f(vx+cos45*vw,vy-cos45*vw)*(572839.0/180)*(180/3.14159) + my_start_all_ecd;

	}
	
}


void chassis_behavier_get(CHASSIS *_chassis)
{
		                                                                                                                                                                                                                                
	PID_calc(&_chassis->motor_2006.pid_param_angle , _chassis->motor_2006.feedback.all_ecd   ,  _chassis->motor_2006.target_angle);	
	PID_calc(&_chassis->motor_2006.pid_param_speed , _chassis->motor_2006.feedback.speed_rpm ,  _chassis->motor_2006.pid_param_angle.out);

	PID_calc(&_chassis->motor_3508.pid_param_speed , _chassis->motor_3508.feedback.speed_rpm ,  _chassis->motor_3508.target_rpm);	

}