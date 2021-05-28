#include "moto.h"
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "main.h"

int times=0;                                             //ADCƽ��ʱ������
int ADC_value = 0;																			 //ADCƽ��ʱ���ֵ
int temp_ADC = 0;                                        //���μ�����
int Bias_target0 = 0.2*390;                              //Ŀ������5
int Bias_target1_ang = 2;																 //Ŀ���������6
int Bias_target1_sp = 0.2*390;													 //Ŀ���ٶ�����6
float k1 = 0.6;
float k2 = 0.4;


int calculate_speed(int pwm_all){                        //���4��Ƶ���ԭpwmƵ��
	int pwm_sum = 0, pwm_avg = 0, pwm_value_final = 0;
	pwm_sum += pwm_all * 100; 			                       //pwm_sum�ۼ�
	pwm_sum -= pwm_avg;							                       //pwm_sum��ȥ�ϴε�ƽ��ֵ
	pwm_avg = pwm_sum * 1.0 / 5; 		                       //����pwm��ƽ��ֵ
	pwm_value_final = pwm_avg; 			                       //pwm_value_final��ֵ��Ϊ��ǰpwm��Ƶ��
	return pwm_value_final;                              
}

int* pwm_output0 (int pwm_last,int Target)
{ 	
	static float Bias,Pwm,Last_bias;
	Bias=pwm_last-Target;                                  //����ƫ��
	Pwm+=Kp0*(Bias-Last_bias)+Ki0*Bias;                    //����ʽPI������
	Last_bias=Bias;	                                       //������һ��ƫ�� 
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Pwm);       //���ñ��ε�ռ�ձ�
	int res[] = {Bias,Last_bias};
	return res;                                            //�������
}

int* pwm_output1 (int angel_last,int Target_ang,int pwm_last,int Target_sp)
{ 	
	static float Bias_ang,Pwm_ang,Integral_bias_ang,Last_Bias_ang,Bias_sp,Last_bias_sp,Pwm_sp,Pwm;
	Bias_sp = pwm_last-Target_sp;																			 //����ƫ��
	Bias_ang = angel_last-Target_ang;                                  //����ƫ��
	Integral_bias_ang += Bias_ang;	                                   //���ƫ��Ļ���
	Pwm_ang = Kp10*Bias_ang+Ki10*Integral_bias_ang+Kd10*(Bias_ang-Last_Bias_ang);     //λ��ʽPID������
	Pwm_sp += Kp11*(Bias_sp-Last_bias_sp)+Ki11*Bias_sp;                 //λ��ʽPID������
	Pwm = k1*Pwm_ang+k2*Pwm_sp;
	Last_Bias_ang = Bias_ang;                                          //������һ��ƫ�� 
	Last_bias_sp = Bias_sp;																						 //������һ��ƫ�� 
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Pwm);                   //���ñ��ε�ռ�ձ�
	int res[] = {Bias_ang,Bias_sp};
	return res;																						       //�������
}

void mode_0(int target,int speed){             //ģʽ0--������
	
	int *Bias = pwm_output0(speed,target);        //�õ����ε����

	if(*Bias<Bias_target0 && *(Bias+1)<Bias_target0){                               //����Ƿ��ȶ���������ƫ��С��Bias_targetΪ�ȶ�
		HAL_ADC_Start(&hadc1);                             //����ADC
		HAL_ADC_PollForConversion(&hadc1, 5);              //��ȡADC��ֵ��ʱ��5ms  ʵ��ʱ��12Mhz/14
		if(times<30){                                      //��ȡ5����ƽ��ֵ����ʱ��Ϊ0.3s
			ADC_value = HAL_ADC_GetValue(&hadc1);
			times++;
		}else{
			temp_ADC = HAL_ADC_GetValue(&hadc1);             //��¼����������
		}
		if(temp_ADC >= ADC_value*1.5/times || temp_ADC <= ADC_value*0.5)  //�������Ƿ񳬹��ȶ�ʱ��ֵ
		{                     
			HAL_GPIO_WritePin(bb_GPIO_Port,bb_Pin,GPIO_PIN_SET);            //������Դ������
			
		}
		else{
			HAL_GPIO_WritePin(bb_GPIO_Port,bb_Pin,GPIO_PIN_RESET);          //�ر���Դ������
		}
	}
}

void mode_1(int target1_ang,int target2_ang,int angel,int target_sp,int speed){        //ģʽ1--������
	int *Bias0 = pwm_output1(angel,target1_ang,speed,target_sp);             //�õ����ε���ת0.3m
	if(*Bias0<Bias_target1_ang && *(Bias0+1)<Bias_target1_sp){
		int *Bias1 = pwm_output1(angel,target2_ang,speed,target_sp);           //�ﵽҪ����ת1m
		if(*Bias1<Bias_target1_ang && *(Bias1)<Bias_target1_sp){
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);   //�ﵽҪ�󣬹رյ��
		}
	}
}
