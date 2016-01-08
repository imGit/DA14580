#ifndef __PAH8003_LED_H
#define __PAH8003_LED_H

enum
{
	_led_on = 0 ,
	_led_off,
};

enum
{
  STATE_NO_TOUCH=0,
  STATE_NORMAL,
};

typedef unsigned char uint8_t;
extern uint8_t _led_status;
extern uint8_t _touch_flag;
//extern uint8_t _led_step; 
extern uint8_t _step;
extern uint8_t _state;
extern void GetRawData(void) ;
extern void InitialSettingLEDOn(void);
extern void InitialSettingLEDOff(void);

#endif //__PAH8003_LED_H
