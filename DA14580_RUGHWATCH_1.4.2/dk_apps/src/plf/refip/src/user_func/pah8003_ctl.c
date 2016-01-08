#include "stdint.h"
#include "stdbool.h"
#include "pxialg.h"
#include "pah8003_ctl.h"
#include "pah8003_led.h"
#include "pxialg.h"
#include "soft_delay.h"
#include "i2c_eeprom.h"
#include "mpu6050.h"
#include "app_rughwatch_proj.h"
extern uint32_t GetHWCounter(void);
extern uint32_t ToMSec(uint32_t);
extern void app_sample128_write_char2_value(uint8_t val);
extern void ReStartHWCounter(void);
//extern float ReadGSensorX(void);
//extern float ReadGSensorY(void);
//extern float ReadGSensorZ(void);
extern int printf ( const char * format, ... );

#define FIFO_SIZE 24
#define FIFO_SIZE_M1 (FIFO_SIZE-1)

typedef struct {
	uint8_t HRD_Data[13] ;
	float MEMS_Data[3] ;
}ppg_mems_data_t; 


static ppg_mems_data_t _ppg_mems_data[FIFO_SIZE] ;
static int _read_index = 0 ;
static int _write_index = 0 ;
static uint8_t _frame_Count=0;

static uint32_t _ppg_raw=0;
static uint32_t _ppg_raw_on=0;
static uint32_t _ppg_raw_off=0;
void PAH8003_Init()
{
	i2c_eeprom_init(0x33,I2C_FAST, I2C_7BIT_ADDR, I2C_1BYTE_ADDR);
}

bool isFIFOEmpty(void) {
    return (_write_index == _read_index) ;
}

bool Push(ppg_mems_data_t *data) { //Push return data must be true. If return false, please increase FIFO size.
    int tmp = _write_index ;
    
    tmp++;
    if(tmp >= FIFO_SIZE)
        tmp = 0 ;
    if(tmp == _read_index)
        return false;
    _ppg_mems_data[tmp] = *data ;
    _write_index = tmp ;
    return true;
}

bool Pop(ppg_mems_data_t *data) {
    int tmp ;
    
    if(isFIFOEmpty())
        return false;
    *data = _ppg_mems_data[_read_index] ;
    tmp = _read_index + 1;
    if(tmp >= FIFO_SIZE)
        tmp = 0 ;
    _read_index = tmp ;
    return true;
}

bool Pixart_HRD(void) 
{
	if(_state == STATE_NO_TOUCH)
	{
		PAH8003_Init();
  		i2c_eeprom_write_byte(0x7F,0x00); //bank0
		_touch_flag = i2c_eeprom_read_byte(0x59) & 0x80;
		if(_touch_flag == 0x80)
		{
			_state = STATE_NORMAL;
		}		
	}
	else
	{
		
	  switch(_step)
	  {
	    case 0:
			i2c_eeprom_write_byte(0x7F,0x00); //bank0
			i2c_eeprom_write_byte(0x06,0x82);  //sw reset
			delay_ms(1);

			i2c_eeprom_write_byte(0x7F,0x00); //bank0
			if(_led_status == _led_off)
			{        
				InitialSettingLEDOff(); //LED off
			}
			else
			{
				InitialSettingLEDOn(); //LED on	        
			}
			_step++;
			break;
		case 1:
case 2:
	      	_step++;
			break;
case 3:
			GetRawData ();
			
			if(_led_status == _led_off)
			{        
				_led_status = _led_on;
			}
			else
			{
				_led_status = _led_off;
			}
			_step ++ ;
			if(_touch_flag != 0x80)
			{
				_state = STATE_NO_TOUCH;
			}
			break;
		case 4:
			_step = 0 ;
	  	default:
	    		break;
	  }
	}

   return true;
}

void GetRawData(void) 
{	
	uint32_t HWCounter = 0 ;
	ppg_mems_data_t ppg_mems_data;
	short gx = 0,gy = 0,gz = 0;
	int i  = 0;
	PAH8003_Init();
	
	i2c_eeprom_write_byte(0x7F,0x01);  //bank1
	
	do
	{
		//check status: 0 is not ready, 1 is ready, 2 is loss one data?
		ppg_mems_data.HRD_Data[0]=i2c_eeprom_read_byte(0x68)&0x0f;	
	}while(ppg_mems_data.HRD_Data[0] == 0);

	//Only support burst read (0x64~0x67), when using I2C interface
	ppg_mems_data.HRD_Data[1]=i2c_eeprom_read_byte(0x64)&0xff;
	ppg_mems_data.HRD_Data[2]=i2c_eeprom_read_byte(0x65)&0xff;
	ppg_mems_data.HRD_Data[3]=i2c_eeprom_read_byte(0x66)&0xff;
	ppg_mems_data.HRD_Data[4]=i2c_eeprom_read_byte(0x67)&0xff;
	
	if(_led_status == _led_off)	//led off
	{
		_ppg_raw_off = ppg_mems_data.HRD_Data[4] ;
		_ppg_raw_off = _ppg_raw_off << 8;
		_ppg_raw_off |= ppg_mems_data.HRD_Data[3] ;
		_ppg_raw_off = _ppg_raw_off << 8;
		_ppg_raw_off |= ppg_mems_data.HRD_Data[2] ;
		_ppg_raw_off = _ppg_raw_off << 8;
		_ppg_raw_off |= ppg_mems_data.HRD_Data[1] ;
    //_ppg_raw_off=(ppg_mems_data.HRD_Data[3]<<24)| (ppg_mems_data.HRD_Data[2] <<16) | (ppg_mems_data.HRD_Data[1]<<8)| ppg_mems_data.HRD_Data[0];
		PowerDownSensor();

		return;
	}
	else
	{	
		_ppg_raw_on = ppg_mems_data.HRD_Data[4] ;
		_ppg_raw_on =_ppg_raw_on << 8;
		_ppg_raw_on |= ppg_mems_data.HRD_Data[3] ;
		_ppg_raw_on = _ppg_raw_on << 8;
		_ppg_raw_on |= ppg_mems_data.HRD_Data[2] ;
		_ppg_raw_on = _ppg_raw_on << 8;
		_ppg_raw_on |= ppg_mems_data.HRD_Data[1] ;		
    //_ppg_raw_on=(ppg_mems_data.HRD_Data[3]<<24)| (ppg_mems_data.HRD_Data[2] <<16) | (ppg_mems_data.HRD_Data[1]<<8)| ppg_mems_data.HRD_Data[0];		
    _ppg_raw = _ppg_raw_on - _ppg_raw_off;
     
    ppg_mems_data.HRD_Data[1] = _ppg_raw & 0xff;
    ppg_mems_data.HRD_Data[2] =(_ppg_raw >>8) & 0xff;
    ppg_mems_data.HRD_Data[3] =(_ppg_raw >>16)& 0xff;
    ppg_mems_data.HRD_Data[4] =(_ppg_raw >>24)& 0xff;
		
		//Only support burst read (0x1A~0x1C), when using I2C interface
		ppg_mems_data.HRD_Data[5]=i2c_eeprom_read_byte(0x1A)&0xff;
		ppg_mems_data.HRD_Data[6]=i2c_eeprom_read_byte(0x1B)&0xff;
		ppg_mems_data.HRD_Data[7]=i2c_eeprom_read_byte(0x1C)&0xff;
		ppg_mems_data.HRD_Data[8]=_frame_Count++;
	//	HWCounter = GetHWCounter();
		ppg_mems_data.HRD_Data[9]=/*ToMSec(HWCounter)*/ 0; //translate HW counter to msec
	//	ReStartHWCounter();
/***********************HW counter explanation start***********************************/
//HW counter is MCU background counter. 
//These functions read heart rate data ready interval time then provide to lib. (Unit is ms)
//ppg_mems_data.HRD_Data[9] Please set 50ms.
// If you want better Performance,please provide real HW timer counter.
/*********************** HW counter explanation end***********************************/
		ppg_mems_data.HRD_Data[10]=0;
		i2c_eeprom_write_byte(0x7F,0x00); //bank0
		//bit7 is Touch Flag (bit7=1 is meant Touch, and bit7=0 is meant No Touch)
		ppg_mems_data.HRD_Data[11]=(i2c_eeprom_read_byte(0x59)&0x80); //Check Touch Flag
		ppg_mems_data.HRD_Data[12]= ppg_mems_data.HRD_Data[6];
		_touch_flag = ppg_mems_data.HRD_Data[11]; 
		led_ctrl(_touch_flag);

		PowerDownSensor();
/***********************G sensor explanation start***********************************/
		//If no G sensor, please set ppg_mems_data.MEMS_Data[3] = {0};
		//G sensor default range = +/-8G, for more detail setting please refer to MEMS scale setting 		 at page 25.
		//If G sensor output data format is not 16bit, please change to 16bit format.
		//For example: G sensor is 12bit
		//ppg_mems_data.MEMS_Data[0] = ReadGSensorX() <<4;
		//ppg_mems_data.MEMS_Data[1] = ReadGSensorY() <<4;
		//ppg_mems_data.MEMS_Data[2] = ReadGSensorZ() <<4;
// In Earphone application, G sensor must be placed above the neck and near ear . 
//The closer to ear, the better. The distance between G-Sensor and 8003 must less than 5cm.
/***********************G sensor explanation end***********************************/
	//	MPU_Init();
	//	MPU_Get_Accelerometer(&gx,&gy,&gz);
		ppg_mems_data.MEMS_Data[0] = gx/*ReadGSensorX()*/;
		ppg_mems_data.MEMS_Data[1] = gy/*ReadGSensorY()*/;
		ppg_mems_data.MEMS_Data[2] = gz/*ReadGSensorZ()*/;   
    for(i = 0; i < 13; i++){
			app_sample128_write_char2_value(ppg_mems_data.HRD_Data[i]);
			uart_send_byte(ppg_mems_data.HRD_Data[i]);
		}
		for(i = 0; i < 3; i++){
			//app_sample128_write_char2_value(ppg_mems_data.MEMS_Data[i]);
			uart_send_byte(ppg_mems_data.MEMS_Data[i]);
		}
    if(Push(&ppg_mems_data)==false) //Save data into FIFO    
    {
			printf("FIFO overflow\n");
    }
	}
}

void TimerISR()	//interrupt per 5ms
{
	 Pixart_HRD();
} 

//void PAH8003_Begin()
//{
//	ppg_mems_data_t ppg_mems_data;
//	unsigned char ready_flag = 0;
//	unsigned char motion_flag = 0;
//	float myHR ;
//	
//	//while(1)
//	//{
//		if(!isFIFOEmpty())
//		{
//			if(Pop(&ppg_mems_data))	//Get data from FIFO
//			{
//				printf("PPG_GSENSOR_RAW_DATA, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", 
//				ppg_mems_data.HRD_Data[0], ppg_mems_data.HRD_Data[1], 
//				ppg_mems_data.HRD_Data[2], ppg_mems_data.HRD_Data[3], 
//				ppg_mems_data.HRD_Data[4], ppg_mems_data.HRD_Data[5], 
//				ppg_mems_data.HRD_Data[6], ppg_mems_data.HRD_Data[7], 
//				ppg_mems_data.HRD_Data[8], ppg_mems_data.HRD_Data[9], 
//				ppg_mems_data.HRD_Data[10], ppg_mems_data.HRD_Data[11], 
//				ppg_mems_data.HRD_Data[12],
//				(int)ppg_mems_data.MEMS_Data[0], 
//				(int)ppg_mems_data.MEMS_Data[1], 
//				(int)ppg_mems_data.MEMS_Data[2]) ;	//Log for analysis

//				if( PxiAlg_Process(ppg_mems_data.HRD_Data, ppg_mems_data.MEMS_Data) == FLAG_DATA_READY) {
//					PxiAlg_HrGet(&myHR);
//				}
//				else
//				{
//					//Check Flag
//				}
//				ready_flag = PxiAlg_GetReadyFlag();
//				motion_flag = PxiAlg_GetMotionFlag() ;
//			}
//		}
//	//}
//}
