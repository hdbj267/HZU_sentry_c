/**
 * @Copyright(C),
 * @FileName:.c
 * @Author: HongYuJia
 * @Teammate
 * @Version: V1.0
 * @Date: 2021.4.13
 * @Description:     根据当届规则进行应用修改           
 * @Note:           
 * @Others: 
**/
#include "rule.h"
#include "can2_app.h"
#include "connect_task.h"
#include "shoot_task.h"
extern ext_Judge_data_t Judge_data;
shoot_real_mag shoot_mag;
void shoot_limit(void)                      
{
	if (Judge_data.hurt_type == 0x3)                                      //读取超热量标志，辅助
	{
		shoot_mag.Excess_Heat_flag =  1;
	}
	else
	{
		shoot_mag.Excess_Heat_flag =  0;
	}
	if (Judge_data.hurt_type == 0x2)                                      //读取超射速标志，辅助
	{
		shoot_mag.Excess_Speed_shoot_flag =  1;
	}
	else
	{
		shoot_mag.Excess_Speed_shoot_flag =  0;
	}
    shoot_heat_limit();
	shoot_speed_limit();
}
/**
  * @brief         发射机构热量限制
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
//void shoot_heat_limit(void)                   
//{	
//    if (Judge_data.shooter_id1_17mm_cooling_limit > Judge_data.shooter_id1_17mm_cooling_heat )
//	{

//		shoot_mag.residual_heat = Judge_data.shooter_id1_17mm_cooling_limit - Judge_data.shooter_id1_17mm_cooling_heat;
//	}
//	else
//	{
//		shoot_mag.residual_heat = 0;

//	}
//	shoot_mag.allow_shoot = (uint16_t)(shoot_mag.residual_heat/10);            //+10 
//	if (shoot_mag.allow_shoot > 1  )
//	{
//		shoot_mag.allow_shoot_flag = 1;
//	}
//	else
//	{
//		shoot_mag.allow_shoot_flag = 0;
//	}
extern ext_Judge_data_t Judge_data;
extern uint8_t trigger_stop_flag;
uint8_t _one_flag = 1;
uint8_t _two_flag = 0;
uint8_t change_shoot_time = 0;
extern shoot_control_data_t shoot_control_data;

//超热量换枪管的舵机代码，调用该函数便能换枪管
void shoot_heat_limit(void)
{
	if((change_shoot_time > 116)&&(_one_flag == 1))
	{
		shoot_mag.allow_shoot_flag = 0;
		while(trigger_stop_flag == 0){};
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2200);//4150//2200
			shoot_mag.allow_shoot_flag = 1;
			_one_flag = 0;
			_two_flag = 1;
			change_shoot_time = 0;
	}
	else if((change_shoot_time > 116)&&(_two_flag == 1))
	{
		shoot_mag.allow_shoot_flag = 0;
		while(trigger_stop_flag == 0){};
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,4150);//4150//2200
			shoot_mag.allow_shoot_flag = 1;
			_two_flag = 0;
			_one_flag = 1;
			change_shoot_time = 0;			
	}
	else if(shoot_control_data.mouse_right_long_press_flag==1)
	{
		change_shoot_time++;
	}
	else 
	{
		shoot_mag.allow_shoot_flag = 1;
	}
}	


extern _tx2_control_data control_data;
/**
  * @brief      射速限制
  * @author         
  * @param[in]      
  * @retval			
  * @note           
  */
//int shoot_B;
void shoot_speed_limit(void)     //18的要小心     *hyj
{

		shoot_mag.allow_shoot_speed = (27-30)/27.0*340+1000;
	//加最值限制
	if (shoot_mag.allow_shoot_speed > 1000)
	{
		shoot_mag.allow_shoot_speed = 1000;
	}
	shoot_mag.allow_fri_speed = shoot_mag.allow_shoot_speed;     
	
}

