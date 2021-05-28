#include "moto.h"
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "main.h"

int times=0;                                             //ADC平稳时检测次数
int ADC_value = 0;																			 //ADC平稳时检测值
int temp_ADC = 0;                                        //单次检测电流
int Bias_target0 = 0.2*390;                              //目标误差，题5
int Bias_target1_ang = 2;																 //目标距离误差，题6
int Bias_target1_sp = 0.2*390;													 //目标速度误差，题6
float k1 = 0.6;
float k2 = 0.4;


int calculate_speed(int pwm_all){                        //获得4倍频后的原pwm频率
	int pwm_sum = 0, pwm_avg = 0, pwm_value_final = 0;
	pwm_sum += pwm_all * 100; 			                       //pwm_sum累加
	pwm_sum -= pwm_avg;							                       //pwm_sum减去上次的平均值
	pwm_avg = pwm_sum * 1.0 / 5; 		                       //更新pwm的平均值
	pwm_value_final = pwm_avg; 			                       //pwm_value_final的值即为当前pwm的频率
	return pwm_value_final;                              
}

int* pwm_output0 (int pwm_last,int Target)
{ 	
	static float Bias,Pwm,Last_bias;
	Bias=pwm_last-Target;                                  //计算偏差
	Pwm+=Kp0*(Bias-Last_bias)+Ki0*Bias;                    //增量式PI控制器
	Last_bias=Bias;	                                       //保存上一次偏差 
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Pwm);       //设置本次的占空比
	int res[] = {Bias,Last_bias};
	return res;                                            //增量输出
}

int* pwm_output1 (int angel_last,int Target_ang,int pwm_last,int Target_sp)
{ 	
	static float Bias_ang,Pwm_ang,Integral_bias_ang,Last_Bias_ang,Bias_sp,Last_bias_sp,Pwm_sp,Pwm;
	Bias_sp = pwm_last-Target_sp;																			 //计算偏差
	Bias_ang = angel_last-Target_ang;                                  //计算偏差
	Integral_bias_ang += Bias_ang;	                                   //求出偏差的积分
	Pwm_ang = Kp10*Bias_ang+Ki10*Integral_bias_ang+Kd10*(Bias_ang-Last_Bias_ang);     //位置式PID控制器
	Pwm_sp += Kp11*(Bias_sp-Last_bias_sp)+Ki11*Bias_sp;                 //位置式PID控制器
	Pwm = k1*Pwm_ang+k2*Pwm_sp;
	Last_Bias_ang = Bias_ang;                                          //保存上一次偏差 
	Last_bias_sp = Bias_sp;																						 //保存上一次偏差 
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Pwm);                   //设置本次的占空比
	int res[] = {Bias_ang,Bias_sp};
	return res;																						       //增量输出
}

void mode_0(int target,int speed){             //模式0--第五题
	
	int *Bias = pwm_output0(speed,target);        //得到本次的误差

	if(*Bias<Bias_target0 && *(Bias+1)<Bias_target0){                               //检测是否稳定，当本次偏差小于Bias_target为稳定
		HAL_ADC_Start(&hadc1);                             //启动ADC
		HAL_ADC_PollForConversion(&hadc1, 5);              //读取ADC数值，时间5ms  实际时间12Mhz/14
		if(times<30){                                      //读取5次求平均值，总时间为0.3s
			ADC_value = HAL_ADC_GetValue(&hadc1);
			times++;
		}else{
			temp_ADC = HAL_ADC_GetValue(&hadc1);             //记录后续检测电流
		}
		if(temp_ADC >= ADC_value*1.5/times || temp_ADC <= ADC_value*0.5)  //检测电流是否超过稳定时的值
		{                     
			HAL_GPIO_WritePin(bb_GPIO_Port,bb_Pin,GPIO_PIN_SET);            //启动有源蜂鸣器
			
		}
		else{
			HAL_GPIO_WritePin(bb_GPIO_Port,bb_Pin,GPIO_PIN_RESET);          //关闭有源蜂鸣器
		}
	}
}

void mode_1(int target1_ang,int target2_ang,int angel,int target_sp,int speed){        //模式1--第六题
	int *Bias0 = pwm_output1(angel,target1_ang,speed,target_sp);             //得到本次的误差，转0.3m
	if(*Bias0<Bias_target1_ang && *(Bias0+1)<Bias_target1_sp){
		int *Bias1 = pwm_output1(angel,target2_ang,speed,target_sp);           //达到要求则转1m
		if(*Bias1<Bias_target1_ang && *(Bias1)<Bias_target1_sp){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);   //达到要求，关闭电机
		}
	}
}
