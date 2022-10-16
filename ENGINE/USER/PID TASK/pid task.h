#ifndef __PID_TASK_H_
#define __PID_TASK_H_

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ble_remote.h"

#include "pid.h"

#define CHASIS_PID_MODE PID_POSITION
#define CHASIS_PID_MAX_OUT  10000.0f
#define CHASIS_PID_MAX_IOUT 100.0f

#define CHASIS_PID_MAX_KP0  180.0f
#define CHASIS_PID_MAX_KI0 0.0f
#define CHASIS_PID_MAX_KD0 0.0f

#define CHASIS_PID_MAX_KP1 180.0f
#define CHASIS_PID_MAX_KI1 0.0f
#define CHASIS_PID_MAX_KD1 0.0f

#define CHASIS_PID_MAX_KP2 180.0f
#define CHASIS_PID_MAX_KI2  0.0f
#define CHASIS_PID_MAX_KD2 0.0f

#define CHASIS_PID_MAX_KP3  180.0f
#define CHASIS_PID_MAX_KI3 0.0f
#define CHASIS_PID_MAX_KD3 0.0f

typedef struct
{
	PID_Regulator_t engine_pid;
	const remote_t *engine_blt;
}CHASIS_Engine_t;

extern void CHASSIS_TASK(void const * argument);

#endif