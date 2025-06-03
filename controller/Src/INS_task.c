#include "INS_task.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

INS_para_t INS_para;

#define INS_TASK_TIME_OUT		1
#define INS_CALI_TIME_OUT		500
uint32_t INS_hight_water;

#define INS_CALI_YAW_THRESHOLD	0.00001f

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
uint8_t ins_receive_buffer;

//static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

float fAcc[3], fGyro[3], fAngle[3];

void INS_init(void)
{
	HAL_UART_Receive_IT(&huart3, &ins_receive_buffer, 1);
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
	WitDelayMsRegister(Delayms);
	AutoScanSensor();
	CmdProcess();
}


void yaw_add_count(float yaw_p, float yaw_last_p, float *yaw_add_p)
{
	if(yaw_p - yaw_last_p > PI)
	{
		*yaw_add_p -= 2.0f * PI;
	}
	else if(yaw_p - yaw_last_p < -PI)
	{
		*yaw_add_p += 2.0f * PI;
	}
}

void INS_task(void const * argument)
{
	vTaskDelay(500);
	INS_init();
	vTaskDelay(1000);
	INS_init();
	while(1)
	{
		if(s_cDataUpdate)
		{
			for(int i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * PI;
			}
			INS_para.INS_attitude.roll = fAngle[0];
			INS_para.INS_attitude.pitch = fAngle[1];
			if(fAngle[2] - INS_para.yaw_last > PI)
			{
				INS_para.yaw_add -= 2.0f * PI;
			}
			else if(fAngle[2] - INS_para.yaw_last < -PI)
			{
				INS_para.yaw_add += 2.0f * PI;
			}
			INS_para.yaw_last = fAngle[2];
			INS_para.INS_attitude.yaw = fAngle[2] + INS_para.yaw_add;
		}
		vTaskDelay(INS_TASK_TIME_OUT);
#if INCLUDE_uxTaskGetStackHighWaterMark
		INS_hight_water = uxTaskGetStackHighWaterMark(NULL);
#endif
	}
}



void CmdProcess(void)
{
	while(fAngle[2]==0)
	{
		if(s_cDataUpdate)
		{
			for(int i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * PI;
			}
		}
		vTaskDelay(INS_TASK_TIME_OUT);
	}
	INS_para.yaw_start = fAngle[2];
	fAngle[2] = 0;
//	WitStartAccCali();
//	vTaskDelay(3000);
//	WitStopAccCali();
//	WitZeroZ();
//	vTaskDelay(100);
//	WitSave();
	
//	yaw = sReg[Yaw] / 32768.0f * PI;
//	INS_para.yaw_start = yaw;
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	HAL_UART_Transmit(&huart3, p_data, uiSize, 1);
}

static void Delayms(uint16_t ucMs)
{
	vTaskDelay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
				break;
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
				break;
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
				break;
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
				break;
            default:
				s_cDataUpdate |= READ_UPDATE;
				break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 1; i < 10; i++)
	{
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			vTaskDelay(100);
			if(s_cDataUpdate != 0)
			{
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
}


void INS_Communication_Callback(void)
{
	WitSerialDataIn(ins_receive_buffer);
	HAL_UART_Receive_IT(&huart3, &ins_receive_buffer, 1);
}

const INS_para_t *get_INS_point(void)
{
	return &INS_para;
}
