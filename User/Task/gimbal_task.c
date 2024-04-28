/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia  
 * @Teammate�?
 * @Version: V3.0
 * @Date:2020.4.13
 * @Description: 关于云台的控�?
 * @Note:       
 * @Others: 
**/
#include "gimbal_task.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_app.h"
#include "can1_app.h"
#include "pid.h"
#include "INS_task.h"
#include "connect_task.h"
#include "GUI_task.h"
#include "monitor_task.h"
#include "tim.h"
#include "gpio.h"
#include "oled.h"
#include "start_task.h"
#include "gimbal_task.h"
#include "can2_app.h"
#include "flash.h"
#include "usart.h"
#include "shoot_task.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include "stm32f4xx_hal_cortex.h"

#define Logistic( X, K, X0, L)	((L)/(1+pow(2.718282f,-(K)*((X)-(X0))))+0.0002)

uint8_t view_control_flag=0,last_work_mode;
float gimbal_yaw_set1,gimbal_yaw_fdb1,yaw_delta,out,yaw_sensit;
extern uint8_t gimbal_init_ok_flag;
extern float value;
extern uint8 getflag;
extern _tx2_control_data control_data;
extern TaskHandle_t INS_Task_Handler;
extern monitor_t monitor;

gimbal_pid_t gimbal_pid;
gimbal_control_data_t gimbal_control_data;
robot_work_mode_e robot_work_mode;
robot_control_mode_e robot_control_mode;
gimbal_work_mode_e gimbal_work_mode;
extern shoot_control_data_t shoot_control_data;

extern ext_Judge_data_t Judge_data;
extern shoot_control_data_t shoot_control_data;
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
float gimbal_abs(float value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}			



//机器人工作模�?
void set_robot_control_mode(robot_control_mode_e mode)
{
	robot_control_mode = mode;
}
uint8_t get_robot_control_mode(void)
{
	return robot_control_mode;
}
void robot_control_mode_update(RC_ctrl_t *rc_s)	
{
	//control mode
	switch(rc_s->rc.s2) 
	{
		case RC_SW_UP:
			set_robot_control_mode(KEY_MOUSE_MODE);break;
		case RC_SW_MID:
			set_robot_control_mode(REMOTE_MODE);break;
		case RC_SW_DOWN:
			set_robot_control_mode(GUI_CALI_MODE);break;
	}
//	 if(monitor.exist_error_flag == 1)//发生严重错�??时，不受遥控指挥强�?�转为调试模�?
//	 {
//	 	set_robot_control_mode(GUI_CALI_MODE);
//	 }
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */

uint32_t time_tick_ms = 0;
uint8_t gimbal_position_init_finish_flag = 0;

//robot_work_mode
void set_robot_work_mode(robot_work_mode_e mode)
{
	robot_work_mode = mode;
}
uint8_t get_robot_work_mode(void)
{
	return robot_work_mode;
}
void robot_work_mode_update(RC_ctrl_t *rc_s)	//***
{
//  if(Judge_data.game_start_flag)			//比赛开始就开�?云台�?描，底盘小陀�?
//		set_robot_work_mode(ROBOT_ROTATE_STOP_MODE);
	 if(gimbal_position_init_finish_flag == 0 && get_robot_control_mode() != GUI_CALI_MODE)
	{
		set_robot_work_mode(ROBOT_INIT_MODE);	//初�?�化	
		gimbal_position_init_finish_flag = 1;
	}
	else if(get_robot_control_mode() == REMOTE_MODE)	//遥控器控制工作模�?
	{
		switch(rc_s->rc.s1)
		{
			case RC_SW_UP:
				set_robot_work_mode(ROBOT_ROTATE_STOP_MODE);break;
			case RC_SW_MID:
				set_robot_work_mode(ROBOT_ROTATE_MOTION_MODE);break;
			case RC_SW_DOWN:
				set_robot_work_mode(ROBOT_COMMON_MODE);break;
		}
	}
	else if(get_robot_control_mode() == KEY_MOUSE_MODE)	//�?鼠控制工作模式EQV
	{
		if(rc_s->key.v & ROBOT_ROTATE_STOP_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_ROTATE_STOP_MODE);
		}
		else if(rc_s->key.v & ROBOT_ROTATE_MOTION_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_ROTATE_MOTION_MODE);
		}
		else if(rc_s->key.v & ROBOT_COMMON_MODE_KEY)
		{
			set_robot_work_mode(ROBOT_COMMON_MODE);
			shoot_control_data.magazine_control_flag = 0; 
		}
	}
	else if(get_robot_control_mode() == GUI_CALI_MODE)
	{
		set_robot_work_mode(ROBOT_CALI_MODE);	//调试模式
		gimbal_position_init_finish_flag = 0;
	}	
	
	if(get_robot_work_mode() == ROBOT_INIT_MODE)
	{
		GREEDLED_ON();
	}
	else 
	{
		GREEDLED_OFF();
	}
}	
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
uint8_t stop_rotate_flag=0,last_rotate_flag,work_mode_change,rotate_flag=1;

//云台工作模式
void set_gimbal_work_mode(gimbal_work_mode_e mode)
{
	gimbal_work_mode = mode;
}
uint8_t get_gimbal_work_mode(void)
{
	return gimbal_work_mode;
}

uint8_t work_mode;
void gimbal_work_mode_update(RC_ctrl_t *rc_s, gimbal_control_data_t *gimbal_control_data)
{
	
	robot_control_mode_update(rc_s);
	robot_work_mode_update(rc_s);
	
	work_mode = get_robot_work_mode();	
	if((last_rotate_flag==1)&&((work_mode!=ROBOT_ROTATE_MOTION_MODE)||(work_mode!=ROBOT_ROTATE_STOP_MODE)))
	{
		stop_rotate_flag = 1;
		last_rotate_flag = 0;
	}
	
	if(get_robot_work_mode() == ROBOT_CALI_MODE)
	{
		set_gimbal_work_mode(GIMBAL_CALI_MODE);
	}
	else if(get_robot_work_mode() == ROBOT_INIT_MODE)
	{
		set_gimbal_work_mode(GIMBAL_RELATIVE_ANGLE_MODE);		
	}
	else if(get_robot_work_mode() == ROBOT_ROTATE_STOP_MODE)
	{
		work_mode_change=1;
		last_rotate_flag = 1;
		set_gimbal_work_mode(GIMBAL_ROTATE_MODE);		
	}
	
	else 
	{
		set_gimbal_work_mode(GIMBAL_ABSOLUTE_ANGLE_MODE);		
	}
	last_work_mode=get_robot_work_mode();
	
	switch(get_gimbal_work_mode())
	{
		case GIMBAL_RELATIVE_ANGLE_MODE:         //编码�?   *hyj
		{
			gimbal_control_data->yaw_motor_fdb_mode = GIMBAL_MOTOR_ENCONDE;
			gimbal_control_data->pitch_motor_fdb_mode = GIMBAL_MOTOR_ENCONDE;
		}break;
		case GIMBAL_ABSOLUTE_ANGLE_MODE:         //陀螺仪   *hyj
		{
			gimbal_control_data->yaw_motor_fdb_mode = GIMBAL_MOTOR_GYRO;
			gimbal_control_data->pitch_motor_fdb_mode = GIMBAL_MOTOR_GYRO;
		}break;
	}
}	
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
#define MOUSE_SINGLE_TIME_ALLOW_VALUE  38   //150
void gimbal_val_limit(int16_t *val, int16_t min, int16_t max)
{
	if(*val < min)
	{
		*val = min;
	}
	else if(*val > max)
	{
		*val = max;
	}
}
void mouse_sensit_cali(RC_ctrl_t *rc_ctrl)
{
	if( rc_ctrl->mouse.x < 20)
	{
		rc_ctrl->yaw_sensit = GAMBAL_MOUSE_YAW_MIN_ANGLE_SET_FACT;
	}
	else
	{
		rc_ctrl->yaw_sensit = GAMBAL_MOUSE_YAW_MAX_ANGLE_SET_FACT;
	}
	
	if ( rc_ctrl->mouse.y < 20)
	{
		rc_ctrl->pitch_sensit = GAMBAL_MOUSE_PITCH_MIN_ANGLE_SET_FACT;
	}
	else
	{
		rc_ctrl->pitch_sensit = GAMBAL_MOUSE_PITCH_MAX_ANGLE_SET_FACT;
	}
}


float abs_fun(float a)
{
	if(a < 0) a = -a;
	return a;
}

float vision_K = 1;
int ROBOT_ROTATE_MOTION_MODE_flag;

pid_t delta_yaw= 
{
	.kp = 10,//0.05
	.ki = 0,//0.008,
	.kd = 0,
	.ioutMax = 1000,
	.outputMax = 5000,
	.mode = PID_DELTA,
	.Calc = &PID_Calc,
	.Reset =  &PID_Reset,
};
pid_t delta_pitch= 
{
	.kp = 8,
	.ki = 0,
	.kd = 0,
	.ioutMax = 30,
	.outputMax = 5000,
	.mode = PID_DELTA,
	.Calc = &PID_Calc,
	.Reset =  &PID_Reset,
};



uint16_t WINDOW_SIZE=2000;
float slidingWindowFilter(int input) {
		int window[50] = {0};
    int sum = 0;
    int output;
    sum -= window[0];
    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
        window[i] = window[i + 1];
    }
    window[WINDOW_SIZE - 1] = input;
    sum += input;

    output = sum / WINDOW_SIZE;
		return output;
}

//全局变量�?多了，最好写进结构体�?
float tt,PITCH,delta_yaw_dev,delta_pitch_dev=-0.5,delta_pitch_dev_y1=-1.0,delta_pitch_dev_y2=3.0,gyroy_aver,yaw_forecast_factor=0.01;
extern float yaw_cnt;
uint8_t vision_flag=0,rotate_break_flag,gimbal_deduce,get_target,vision_control,rotate_deduce,left_num,right_num,enemy_rotate;
int break_time,break_num=1500;

void gimbal_set_and_fdb_update(gimbal_control_data_t *gimbal_control_data,
							   robot_control_mode_e robot_control_mode,							   
							   _tx2_control_data control_data  )
{
/*根据yaw轴的运动规律判断对方�?否开�?小陀螺，写这�?�?为了做自瞄补偿用，当时时间紧没用�?
	基本逻辑:在有�?瞄数�?的前提下，云台有规律地左右摇摆，摇摆时大于某�?速度，就认为对面开
	�?了小陀�?
	有了这个标志位做反小陀螺的办法：给delta_yaw_dev赋值，让枪管和装甲板在yaw轴差一�?恒定的�?�度�?
	然后让云台回正时开�?*/
	switch(rotate_deduce)
	{
		case 0:
			if(gimbal_control_data->gimbal_INS->gyro_y>5)
			{
			rotate_deduce=1;
			left_num+=1;
			}break;
		case 1:
			if(gimbal_control_data->gimbal_INS->gyro_y<-5)
			{
			rotate_deduce=0;
			right_num+=1;
			}
			break;
		default:break;
	}
		if(gyroy_aver<20)
	{
	enemy_rotate=0;
	}
	else if((right_num>3&&left_num>3)&&(gyroy_aver>20))
	{
	enemy_rotate=1;
	right_num--;
	left_num--;
	}
	
		/*下面这一部分的代码是尝试进�?�做视�?��?�测的部分，基本逻辑：由于自瞄时，云台的运动基本�?和敌方�?�甲板的运动绑定�?
	所以�?�云台的yaw轴�?�速度进�?�长时间的取平均，得到比较平滑的角速度，那么�?��?�速度就与敌方的�?�甲板移动速度正相关，
	用�?��?�速度给yaw轴做补偿（�?�甲板水平运动时），该部分目前未实践，只�?写了�?接口,能不能用，稳不稳定还不好�?*/
	gyroy_aver=slidingWindowFilter(abs_fun(gimbal_control_data->gimbal_INS->gyro_y));
	//delta_yaw_dev=gyroy_aver*yaw_forecast_factor;//用的时候再开

	
	

	    if(gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_GYRO)//mpu
	{
				if(stop_rotate_flag)
			 {
			 yaw_cnt=0;
			 stop_rotate_flag=0;
			 }
			if(robot_control_mode == GUI_CALI_MODE)
		{
			delta_yaw.iout=0;
			delta_pitch.iout=0;	
		}

		else if(robot_control_mode == REMOTE_MODE||robot_control_mode == KEY_MOUSE_MODE) 
			/*s2的二三档，二档手动射弹，三档加上识别到�?�甲板自动射击（已有），导航使用该挡位，当�?��?�没有数�?时可以�?�航�?
			有数�?将中�?导航，�?��?�标志位�?0才继�?导航，两者都不满足开�?云台�?描模式（构想�?*/
		
		{
			/*由于给每算一次自瞄给view_control_flag�?0，所以这�?标志位不能作为进入自瞄模式的标志（会跟踪得很不稳定）�?
			使用该状态机对�?�标志位进�?�改进为get_target标志�?*/
			switch(gimbal_deduce)
			{
			case 0:
				if(control_data.view_control_flag==1 )
				{
					gimbal_deduce=1;
					get_target=1;
				}
				break;
			case 1:
				if(break_time>break_num)
				{
					get_target=0;
					gimbal_deduce=0;
					break_time=0;
				}
				else if(control_data.view_control_flag==0 )
					break_time++;
				break;
			default: break;
			}
				

			if(control_data.recog_flag)	//�?瞄�?�理
			{

				//delta_pitch_dev=-(control_data.Target_distance-5000.0)/4000.0*(delta_pitch_dev_y1-delta_pitch_dev_y2)+delta_pitch_dev_y2;
				/*pitch轴差值补偿，用两点式拟合距�?�与补偿量的关系 y=(x-x2)/(x1-x2)*(y1-y2)+y2  (1000,-1) (5000,3)*/
				delta_yaw.set=0;
				if(control_data.yaw_sign)
					//delta_yaw.fdb=control_data.yaw_dev-2+delta_yaw_dev;
					delta_yaw.fdb=control_data.yaw_dev;
				else
					//delta_yaw.fdb=-control_data.yaw_dev-2-delta_yaw_dev;
					delta_yaw.fdb=-control_data.yaw_dev;
				delta_yaw.Calc(&delta_yaw);
				
				delta_pitch.set=0;
				if(control_data.pitch_sign)
					delta_pitch.fdb=control_data.pitch_dev-delta_pitch_dev;
				else
					delta_pitch.fdb=-control_data.pitch_dev-delta_pitch_dev;
				delta_pitch.Calc(&delta_pitch);
				control_data.recog_flag=0;
				vision_control=1;	//进入�?瞄�?�算标志�?
			}
			

			
			else
			{
				//hzp
				yaw_sensit = Logistic(gimbal_abs(gimbal_control_data->rc_ctrl->rc.ch3*0.0182),1,6,0.00015);/*回归函数，旋�?灵敏度随着前进速度
																																										增大而�?�大，这�?函数应�?�加到�?�兵的鼠标灵敏度里边*/
				
				gimbal_control_data->gimbal_yaw_set += (-gimbal_control_data->rc_ctrl->rc.ch0) *    \
													 yaw_sensit;				
				//gimbal_control_data->gimbal_yaw_set += 0.005*value;
				gimbal_control_data->gimbal_pitch_set += (-gimbal_control_data->rc_ctrl->rc.ch1) *  \
													 GAMBAL_PITCH_MAX_ANGLE_SET_FACT;//ch1
			}
		}	
		if(!control_data.recog_flag)
				{
				 rotate_break_flag=0;
				}
		gimbal_control_data->gimbal_yaw_fdb = gimbal_control_data->gimbal_INS->yaw_angle;
		yaw_delta = gimbal_control_data->gimbal_yaw_fdb - gimbal_yaw_fdb1;
		gimbal_yaw_fdb1 = gimbal_control_data->gimbal_yaw_fdb;
		gimbal_control_data->gimbal_pitch_fdb = gimbal_control_data->gimbal_INS->pitch_angle;	
	if(gimbal_control_data->gimbal_pitch_set>25)
	{
		gimbal_control_data->gimbal_pitch_set=25;
	}
	else if(gimbal_control_data->gimbal_pitch_set<-25)
	{
		gimbal_control_data->gimbal_pitch_set=-25;
	}
		else
		gimbal_control_data->gimbal_pitch_set=gimbal_control_data->gimbal_pitch_set;
	}
	else if(gimbal_control_data->pitch_motor_fdb_mode == GIMBAL_MOTOR_ENCONDE)//初�?�化时的编码模式 set �?�?�?
	{
		//yaw
		gimbal_control_data->gimbal_yaw_set =  gimbal_control_data->gimbal_INS->yaw_angle\
		+((float)(GAMBAL_YAW_INIT_ENCODE_VALUE-gimbal_control_data->gimbal_yaw_motor_msg->encoder.raw_value))/GAMBAL_YAW_angle_VALUE;
		gimbal_control_data->gimbal_yaw_fdb = gimbal_control_data->gimbal_INS->yaw_angle;	
		//pitch
		gimbal_control_data->gimbal_pitch_set = gimbal_control_data->gimbal_INS->pitch_angle\
		-((float)(GAMBAL_PITCH_INIT_ENCODE_VALUE-gimbal_control_data->gimbal_pitch_motor_msg->encoder.raw_value))/GAMBAL_PITCH_angle_VALUE;	
		//gimbal_control_data->gimbal_pitch_set =  0;
		gimbal_control_data->gimbal_pitch_fdb = gimbal_control_data->gimbal_INS->pitch_angle;
	if(gimbal_control_data->gimbal_pitch_set>25)
	{
		gimbal_control_data->gimbal_pitch_set=25;
	}
	else if(gimbal_control_data->gimbal_pitch_set<-25)
	{
		gimbal_control_data->gimbal_pitch_set=-25;
	}
		else
		gimbal_control_data->gimbal_pitch_set=gimbal_control_data->gimbal_pitch_set;
	}

}


/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
float rotate_speed_angle;
uint8_t first_rotate=1;

void gimbal_cascade_pid_calculate(gimbal_pid_t *gimbal_pid,       \
					              gimbal_control_data_t *gimbal_control_data)
{
	
	//yaw轴�?�度pid计算
		if((gimbal_work_mode == GIMBAL_ROTATE_MODE)&&(!get_target))	//s1的三档，云台�?描挡
		//if(((gimbal_work_mode == GIMBAL_ROTATE_MODE)&&(lose_flag))||first_rotate)	
	{
	gimbal_control_data->gimbal_pitch_set= -12.5 * sin( tt * PI/180);//�?描时pitch的�?�度变化
	//gimbal_control_data->gimbal_pitch_set=0;
  tt+=0.07;
  if(tt==180)
  tt=0;
  gimbal_control_data->gimbal_yaw_fdb = gimbal_control_data->gimbal_INS->yaw_angle;
  gimbal_control_data->gimbal_pitch_fdb = gimbal_control_data->gimbal_INS->pitch_angle;	

	gimbal_pid->yaw_pid.speed_pid.set = -150;	//云台�?描时的旋�?速度
	gimbal_pid->yaw_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_y;
	}
	else
	{
	//gimbal_pid->yaw_pid.position_pid.set = gimbal_control_data->gimbal_yaw_set;
	gimbal_pid->yaw_pid.position_pid.set = gimbal_control_data->gimbal_yaw_set;
	gimbal_pid->yaw_pid.position_pid.fdb = gimbal_control_data->gimbal_yaw_fdb;
	gimbal_pid->yaw_pid.position_pid.Calc(&gimbal_pid->yaw_pid.position_pid);
	
	//yaw轴速度pid计算
	//gimbal_pid->yaw_pid.speed_pid.set = 0 ;
	if(vision_control)
	{
	gimbal_pid->yaw_pid.speed_pid.set   = -delta_yaw.output;
	gimbal_pid->pitch_pid.speed_pid.set = -delta_pitch.output;
	vision_control=0;
	}
	else
	{	
	gimbal_pid->yaw_pid.speed_pid.set   = -gimbal_pid->yaw_pid.position_pid.output;
	gimbal_pid->pitch_pid.speed_pid.set = -gimbal_pid->pitch_pid.position_pid.output;
	}
	}
	gimbal_pid->yaw_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_y;
	gimbal_pid->yaw_pid.speed_pid.Calc(&gimbal_pid->yaw_pid.speed_pid);
	
	//pitch轴�?�度pid计算
	gimbal_pid->pitch_pid.position_pid.set = -gimbal_control_data->gimbal_pitch_set;
	gimbal_pid->pitch_pid.position_pid.fdb = -gimbal_control_data->gimbal_pitch_fdb; 
	gimbal_pid->pitch_pid.position_pid.Calc(&gimbal_pid->pitch_pid.position_pid);
	//pitch轴速度pid计算
	gimbal_pid->pitch_pid.speed_pid.fdb = -gimbal_control_data->gimbal_INS->gyro_z;
	gimbal_pid->pitch_pid.speed_pid.Calc(&gimbal_pid->pitch_pid.speed_pid);	
	
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_control_loop(gimbal_pid_t *gimbal_pid,       \
					     gimbal_control_data_t *gimbal_control_data)
{
	//这里改为了�?��?

	gimbal_control_data->given_current.yaw_motor = -gimbal_pid->yaw_pid.speed_pid.output;//g6020反向安�?�需要加负号
	gimbal_control_data->given_current.pitch_motor = gimbal_pid->pitch_pid.speed_pid.output;
	//gimbal_control_data->given_current.yaw_motor = 0;
	//gimbal_control_data->given_current.pitch_motor = 0;

	if(rc_ctrl_data.rc.s1==2&&rc_ctrl_data.rc.s2==2)
	{
		set_gimbal_stop();
	}
	else 
	{
		set_gimbal_behaviour(gimbal_control_data->given_current.yaw_motor,   \
							gimbal_control_data->given_current.pitch_motor); 
	}
	
}

/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_pid_init(pid_t *pid, cali_pid_t *cali_pid)
{
	pid->kp = cali_pid->kp;
	pid->ki = cali_pid->ki;
	pid->kd = cali_pid->kd;
	pid->a = cali_pid->a;
	
	pid->ioutMax = cali_pid->ioutput_max;
	pid->outputMax = cali_pid->output_max;
	
	pid->mode = cali_pid->mode;
	
	pid->Calc = &PID_Calc;
	pid->Reset =  &PID_Reset;
}
/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
void gimbal_init(gimbal_pid_t *gimbal_pid,   \
				 cali_gimbal_t *cali_pid,    \
				 gimbal_control_data_t *gimbal_control_data)
{
	gimbal_control_data->rc_ctrl = get_rc_data_point();
	gimbal_control_data->gimbal_INS = get_INS_point();
	gimbal_control_data->gimbal_yaw_motor_msg = get_yaw_motor_msg_point();
	gimbal_control_data->gimbal_pitch_motor_msg = get_pitch_motor_msg_point();
	//yaw cascade pid
	gimbal_pid_init(&gimbal_pid->yaw_pid.position_pid, &cali_pid->yaw_pid.position);
	gimbal_pid_init(&gimbal_pid->yaw_pid.speed_pid, &cali_pid->yaw_pid.speed);
	//pitch cascade pid
	gimbal_pid_init(&gimbal_pid->pitch_pid.position_pid, &cali_pid->pitch_pid.position);
	gimbal_pid_init(&gimbal_pid->pitch_pid.speed_pid, &cali_pid->pitch_pid.speed);
	//GIMBAL_InitArgument();

	set_robot_control_mode(GUI_CALI_MODE);
	set_robot_work_mode(ROBOT_CALI_MODE);
	set_gimbal_work_mode(GIMBAL_CALI_MODE);

}
/**
  * @brief        运�?�时挂起GUI任务
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */


/**
  * @brief        
  * @author         
  * @param[in]      
  * @retval			
  * @note  		1.gimbal_task 跑不了oled程序 md不知道什么问�?  
				2.while�?�?的前�?两个函数消�?4%(其中�?一�?0.3%) 整个while消�?15%
  */
void gimbal_task(void *argument)
{
	TickType_t current_time = 0;
	
	vTaskDelay(GIMBAL_TASK_INIT_TIME);
	gimbal_init(&gimbal_pid, &cali_gimbal_pid, &gimbal_control_data);
	while(1)
	{		
		current_time = xTaskGetTickCount();                         //当前系统时间       *hyj
		send_gyro_data_to_chassis();
		gimbal_work_mode_update(&rc_ctrl_data, &gimbal_control_data);//更新云台状�?
		gimbal_set_and_fdb_update(&gimbal_control_data, robot_control_mode, control_data );//set fdb数据更新
		
		gimbal_cascade_pid_calculate(&gimbal_pid, &gimbal_control_data);//串级pid计算
		gimbal_control_loop(&gimbal_pid, &gimbal_control_data);//控制�?�?
		vTaskDelayUntil(&current_time, GIMBAL_TASK_TIME_1MS);       //1ms一�?         *hyj     
	}	
}
