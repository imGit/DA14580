#include "pah8003_led_ctl.h"
#include "i2c_eeprom.h"
#include "pah8003_led.h"
#define LED_INC_DEC_STEP 2
#define LED_CTRL_EXPO_TIME_HI_BOUND 496
#define LED_CTRL_EXPO_TIME_LOW_BOUND 32
#define LED_CTRL_EXPO_TIME_HI 420
#define LED_CTRL_EXPO_TIME_LOW 64
#define LED_CURRENT_HI 31
#define LED_CURRENT_LOW 1
#define STATE_COUNT_TH 3
#define DEFAULT_LED_STEP 10
static uint8_t _led_step = DEFAULT_LED_STEP;
static uint8_t _state = 0, _state_count = 0;
static uint8_t _led_current_change_flag = 0;
static uint8_t _sleepflag = 1 ;

void led_ctrl(uint8_t touch)
{
if(touch == 0x80)
{
uint8_t data;
uint16_t Frame_Average, EP_L, EP_H, Exposure_Line;
i2c_eeprom_write_byte(0x7f,0x00);
i2c_eeprom_write_byte(0x05, 0x98);
data = i2c_eeprom_read_byte(0x33);
EP_H=data&0x03;
data = i2c_eeprom_read_byte(0x32);
EP_L=data;
Exposure_Line=(EP_H<<8)+EP_L;
i2c_eeprom_write_byte(0x7f,0x01);
if(_sleepflag==1)
{
i2c_eeprom_write_byte(0x38, (0xE0|DEFAULT_LED_STEP));
_sleepflag = 0 ;
}
if (_state_count <= STATE_COUNT_TH) {
_state_count++;
_led_current_change_flag = 0;
}
else {
_state_count = 0;
if(_state == 0)
{
if( (Exposure_Line>=LED_CTRL_EXPO_TIME_HI_BOUND) ||
(Exposure_Line<=LED_CTRL_EXPO_TIME_LOW_BOUND )
)
{
data = i2c_eeprom_read_byte(0x38);
_led_step=data&0x1f;
if( (Exposure_Line>=LED_CTRL_EXPO_TIME_HI_BOUND)
&& (_led_step < LED_CURRENT_HI))
{
_state = 1 ;
_led_step=_led_step+LED_INC_DEC_STEP;
if(_led_step>LED_CURRENT_HI)
_led_step=LED_CURRENT_HI;
i2c_eeprom_write_byte(0x38, (_led_step|0xE0));
_led_current_change_flag = 1;
}
else if((Exposure_Line<=LED_CTRL_EXPO_TIME_LOW_BOUND)
&& (_led_step > LED_CURRENT_LOW))
{
_state = 2 ;
if(_led_step<=(LED_CURRENT_LOW+LED_INC_DEC_STEP))
_led_step=LED_CURRENT_LOW;
else
_led_step=_led_step-LED_INC_DEC_STEP;
i2c_eeprom_write_byte(0x38, (_led_step|0xE0));

_led_current_change_flag = 1;
}else
{
_state = 0 ;
_led_current_change_flag = 0;
}
}
else {
_led_current_change_flag = 0;
}
}
else if(_state == 1)
{
if(Exposure_Line > LED_CTRL_EXPO_TIME_HI)
{
_state = 1 ;
_led_step=_led_step+LED_INC_DEC_STEP;
if(_led_step>=LED_CURRENT_HI)
{
_state = 0 ;
_led_step=LED_CURRENT_HI;
}
i2c_eeprom_write_byte(0x38, (_led_step|0xE0));
_led_current_change_flag = 1;
}
else
{
_state = 0 ;
_led_current_change_flag = 0;
}
}
else
{
if(Exposure_Line < LED_CTRL_EXPO_TIME_LOW)
{
_state = 2 ;
if(_led_step<=(LED_CURRENT_LOW+LED_INC_DEC_STEP))
{
_state = 0 ;
_led_step=LED_CURRENT_LOW;
}
else
_led_step=_led_step-LED_INC_DEC_STEP;
i2c_eeprom_write_byte(0x38, (_led_step|0xE0));
_led_current_change_flag = 1;
}
else
{
_state = 0;
_led_current_change_flag = 0;
}
}
}
}

else
{
i2c_eeprom_write_byte(0x7f,0x00);
i2c_eeprom_write_byte(0x05, 0xB8);
i2c_eeprom_write_byte(0x7F, 0x01); 
_led_step = DEFAULT_LED_STEP;
//i2c_eeprom_write_byte(0x38, (0xE0 | DEFAULT_LED_STEP)); //for Asian person only
i2c_eeprom_write_byte(0x38, 0xFF);
_sleepflag = 1;
_led_current_change_flag = 0;
}
}
/***********************LED Control End ***********************************/
/*********************** PowerDownSensor start ***********************************/
void PowerDownSensor(void)
{
if(_led_status == _led_off)
{
uint8_t b0_0x32_off;
uint8_t b0_0x33_off;
uint8_t b0_0x35_off;
i2c_eeprom_write_byte(0x7F,0x00); //bank0
b0_0x32_off = i2c_eeprom_read_byte(0x32);
b0_0x33_off = i2c_eeprom_read_byte(0x33);
b0_0x35_off = i2c_eeprom_read_byte(0x35);
_b1_0x37_off = b0_0x32_off ;
_b1_0x36_off = 0x62 & (( b0_0x33_off & 0x3 ) << 5) ;
_b0_0x4d_off = 0x9A & (( b0_0x33_off & 0x4 ) << 5) ;
_b0_0x48_off = 0x0F & ( b0_0x35_off & 0xF ) ;
i2c_eeprom_write_byte(0x06,0x0A);
}
else
{
uint8_t b0_0x32_on;
uint8_t b0_0x33_on;
uint8_t b0_0x35_on;
i2c_eeprom_write_byte(0x7F,0x00); //bank0
b0_0x32_on = i2c_eeprom_read_byte(0x32);
b0_0x33_on = i2c_eeprom_read_byte(0x33);
b0_0x35_on = i2c_eeprom_read_byte(0x35);
_b1_0x37_on = b0_0x32_on ;
_b1_0x36_on = 0x62 & (( b0_0x33_on & 0x3 ) << 5) ;
_b0_0x4d_on = 0x9A & (( b0_0x33_on & 0x4 ) << 5) ;
_b0_0x48_on = 0x0F & ( b0_0x35_on & 0xF ) ;
if(_touch_flag == 0x80) //if no touch, do not go to power down.
i2c_eeprom_write_byte(0x06,0x0A);

else
{
i2c_eeprom_write_byte(0x7f,0x00); //for bank0
i2c_eeprom_write_byte(0x5, 0xB8);
i2c_eeprom_write_byte(0x7F, 0x01);
i2c_eeprom_write_byte(0x38, 0xFF);
}
}
}