#include "pid task.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"
#include "pid.h"
#include "usart.h"
#include "ble_remote.h"
#include "math.h"

float pid_give[4];
float fdb[4];
uint8_t bt_data[11];
CHASIS_Engine_t  engine_struct[4];
int32_t set[4];
float ware_send[8];
int8_t i;
uint32_t wait_time;

/*ware picture*/
void sendware(void *wareaddr, uint32_t waresize)
{
	#define CMD_WARE     3
	uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    
	uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};   
	HAL_UART_Transmit(&huart1, (uint8_t *)cmdf, sizeof(cmdf), 5000);
	HAL_UART_Transmit(&huart1, (uint8_t *)wareaddr, waresize ,5000);
	HAL_UART_Transmit(&huart1,(uint8_t *)cmdr, sizeof(cmdr), 5000);
}

/*blue tooth*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4)
	{
		uart_to_remote (bt_data);
		HAL_UART_Receive_IT (&huart4, bt_data, 11);
	}
}
/*任务函数*/
void CHASSIS_TASK(void const * argument)
{
	/*enable*/
	HAL_TIM_PWM_Start (&htim15, TIM_CHANNEL_2 );//m2+
  HAL_TIM_PWM_Start (&htim8, TIM_CHANNEL_4);//m2-
	HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_4);//m1+
	HAL_TIM_PWM_Start (&htim3, TIM_CHANNEL_2 );//m1-
	HAL_TIM_PWM_Start (&htim12, TIM_CHANNEL_2 );//m3-
	HAL_TIM_PWM_Start (&htim12, TIM_CHANNEL_1);//m3+
	HAL_TIM_PWM_Start (&htim15, TIM_CHANNEL_1);//m4+
	HAL_TIM_PWM_Start (&htim17, TIM_CHANNEL_1);//m4-
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);//m1
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);//m2
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);//m3
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);//m4
	/*pid init*/
	PID_Init(&engine_struct[0].engine_pid, CHASIS_PID_MODE, CHASIS_PID_MAX_OUT, CHASIS_PID_MAX_IOUT, CHASIS_PID_MAX_KP0, CHASIS_PID_MAX_KI0, CHASIS_PID_MAX_KD0);	
	PID_Init(&engine_struct[1].engine_pid, CHASIS_PID_MODE, CHASIS_PID_MAX_OUT, CHASIS_PID_MAX_IOUT, CHASIS_PID_MAX_KP1, CHASIS_PID_MAX_KI1, CHASIS_PID_MAX_KD1);	
	PID_Init(&engine_struct[2].engine_pid, CHASIS_PID_MODE, CHASIS_PID_MAX_OUT, CHASIS_PID_MAX_IOUT, CHASIS_PID_MAX_KP2, CHASIS_PID_MAX_KI2, CHASIS_PID_MAX_KD2);	
	PID_Init(&engine_struct[3].engine_pid, CHASIS_PID_MODE, CHASIS_PID_MAX_OUT, CHASIS_PID_MAX_IOUT, CHASIS_PID_MAX_KP3, CHASIS_PID_MAX_KI3, CHASIS_PID_MAX_KD3);	
	wait_time = xTaskGetTickCount ();
	/*work*/
	for(;;)
  {
		/*数据接收*/
		HAL_UART_Receive_IT (&huart4, bt_data, 11);
		engine_struct[0].engine_blt = get_remote_control_point ();
//		if (fabsf(engine_struct[0].engine_blt->rocker[0].x_position) > engine_struct[0].engine_blt->rocker[0].y_position)
//		{
//			engine_struct[0].engine_blt->rocker[0].y_position = 0;
//		}
//		else
//		{
//			engine_struct[0].engine_blt->rocker[0].x_position = 0;
//		}
		/*麦轮解算*/
		set[0] = engine_struct[0].engine_blt->rocker[0].x_position + engine_struct[0].engine_blt->rocker[0].y_position + engine_struct[0].engine_blt->rocker[1].x_position;
		set[1] = -engine_struct[0].engine_blt->rocker[0].x_position + engine_struct[0].engine_blt->rocker[0].y_position + engine_struct[0].engine_blt->rocker[1].x_position;
		set[2] = -engine_struct[0].engine_blt->rocker[0].x_position - engine_struct[0].engine_blt->rocker[0].y_position + engine_struct[0].engine_blt->rocker[1].x_position;
		set[3] = engine_struct[0].engine_blt->rocker[0].x_position - engine_struct[0].engine_blt->rocker[0].y_position + engine_struct[0].engine_blt->rocker[1].x_position;
		/*m1*/
		fdb[0] = (short)(__HAL_TIM_GET_COUNTER (&htim1))*1.28;
		pid_give[0] = PID_Calculate(&engine_struct[0].engine_pid, fdb[0], set[0]);
		/*m2*/
		fdb[1] = (short)(__HAL_TIM_GET_COUNTER (&htim2))*1.28;
		pid_give[1] = PID_Calculate(&engine_struct[1].engine_pid, fdb[1], set[1]);
		/*m3*/
		fdb[2] = (short)(__HAL_TIM_GET_COUNTER (&htim4))*1.28;
		pid_give[2] = PID_Calculate(&engine_struct[2].engine_pid, fdb[2], set[2]);
		/*m4*/
		fdb[3] = (short)(__HAL_TIM_GET_COUNTER (&htim5))*1.28;
		pid_give[3] = PID_Calculate(&engine_struct[3].engine_pid, fdb[3], set[3]);
		/*ware show*/
		for(i = 0; i < 4; i++)
		{
			ware_send [i] = fdb [i];
			ware_send [i+4] = set[i];
		}
		sendware (ware_send, sizeof(ware_send));
		/*正负判断*/
		/*m1*/
		if (pid_give[0] > 0)
		{
			__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_2, 0);  //m1-
			__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_4, pid_give[0]); //m1+	
		}
		else
		{
			__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_2, -(pid_give[0]));  //m1-
			__HAL_TIM_SetCompare (&htim3, TIM_CHANNEL_4, 0);	//m1+
		}
		__HAL_TIM_SET_COUNTER (&htim1, 0);
		/*m2*/
		if (pid_give[1] > 0)
		{
			__HAL_TIM_SetCompare (&htim8, TIM_CHANNEL_4, 0);  //m2-
			__HAL_TIM_SetCompare (&htim15, TIM_CHANNEL_2, pid_give[1]); //m2+	
		}
		else
		{
			__HAL_TIM_SetCompare (&htim8, TIM_CHANNEL_4, -(pid_give[1]));  //m2-
			__HAL_TIM_SetCompare (&htim15, TIM_CHANNEL_2, 0);	//m2+
		}
		__HAL_TIM_SET_COUNTER (&htim2, 0);
		/*m3*/
		if (pid_give[2] > 0)
		{
			__HAL_TIM_SetCompare (&htim12, TIM_CHANNEL_2, 0);  //m3-
			__HAL_TIM_SetCompare (&htim12, TIM_CHANNEL_1, pid_give[2]); //m3+	
		}
		else
		{
			__HAL_TIM_SetCompare (&htim12, TIM_CHANNEL_2, -(pid_give[2]));  //m3-
			__HAL_TIM_SetCompare (&htim12, TIM_CHANNEL_1, 0);	//m3+
		}
		__HAL_TIM_SET_COUNTER (&htim4, 0);
		/*m4*/
		if (pid_give[3] > 0)
		{
			__HAL_TIM_SetCompare (&htim17, TIM_CHANNEL_1, 0);  //m4-
			__HAL_TIM_SetCompare (&htim15, TIM_CHANNEL_1, pid_give[3]); //m4+	
		}
		else
		{
			__HAL_TIM_SetCompare (&htim17, TIM_CHANNEL_1, -(pid_give[3]));  //m4-
			__HAL_TIM_SetCompare (&htim15, TIM_CHANNEL_1, 0);	//m4+
		}
		__HAL_TIM_SET_COUNTER (&htim5, 0);
		osDelayUntil (&wait_time, 5);
  }
}