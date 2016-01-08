/**
 ****************************************************************************************
 *
 * @file periph_setup.c
 *
 * @brief Peripherals setup and initialization. 
 *
 * Copyright (C) 2014. DGC Technology Co., Ltd. All Rights Reserved.
 *
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"             // SW configuration
#include "periph_setup.h"            // periphera configuration
#include "global_io.h"
#include "gpio.h"
#include "uart.h"                    // UART initialization
#include "pwm.h"
#include "app_rughwatch_proj.h"
#include "mpu6050.h"
#include "app_hrps.h"
#include "app_accel_task.h"

#ifndef _PERIPH_SETUP_H_
#define _PERIPH_SETUP_H_
#ifdef CLOVE_V1_0
#include "battery.h"
#endif

extern int app_sample128_write_char2_value(uint8_t val);
enum
{
	NONE_STATE,
	LED_STATE,
	MOTOR_CHECK_STATE,
	BUTTON_SW1_STATE,
	BUTTON_SW2_STATE,
	BUTTON_SW3_STATE,
	BUTTON_SW4_STATE,
	BUTTON_SOS_STATE,
	LED1_WHITE_STATE,
	LED2_WHITE_STATE,
	LED3_WHITE_STATE,
	LED4_WHITE_STATE,
	MOTOR_STATE,
}jplus_timer_state;



/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 *
 * @return void
 ****************************************************************************************
 */

#if DEVELOPMENT__NO_OTP

void GPIO_reservations(void)
{

/*
* Application specific GPIOs reservation
*/    
#if (BLE_APP_PRESENT)
#if BLE_PROX_REPORTER	
	  RESERVE_GPIO( PUSH_BUTTON, GPIO_BUTTON_PORT,  GPIO_BUTTON_PIN, PID_GPIO);  //
//	  RESERVE_GPIO( BUZZER, GPIO_ALERT_BUZZER_PORT, GPIO_ALERT_BUZZER_PIN, PID_PWM3);
#endif	

#if BLE_BATT_SERVER
	//Setup LED GPIO for battery alert
//	RESERVE_GPIO( RED_LED, GPIO_BAT_LED_PORT, GPIO_BAT_LED_PIN, PID_GPIO);
	RESERVE_GPIO( BAT_DET, GPIO_BAT_DET_PORT, GPIO_BAT_DET_PIN, PID_ADC);//gsx,ADC-DEC
//	RESERVE_GPIO( RED_LED, GPIO_ALERT_CHRG_PORT, GPIO_ALERT_CHRG_PIN, PID_GPIO);//gsx,CHRG,充电指示
#endif
#endif   

	RESERVE_GPIO( led, GPIO_ALERT_PWR_PORT,GPIO_ALERT_PWR_PIN,PID_GPIO);
	RESERVE_GPIO( led, LED_B_PORT, LED_B_PIN, PID_GPIO);
	RESERVE_GPIO( led, LED_O_PORT, LED_O_PIN, PID_GPIO);

	RESERVE_GPIO(UART_TX, UART1_PORT, UART1_TX_PIN, PID_UART1_TX);
	RESERVE_GPIO(UART_RX, UART1_PORT, UART1_RX_PIN, PID_UART1_RX);
	
	RESERVE_GPIO(I2C_SCL, I2C_PORT, I2C_PIN_SCL, PID_I2C_SCL);
	RESERVE_GPIO(I2C_SDA, I2C_PORT, I2C_PIN_SDA, PID_I2C_SDA);
}
#endif

/**
 ****************************************************************************************
 * @brief Map port pins
 *
 * The Uart and SPI port pins and GPIO ports(for debugging) are mapped
 ****************************************************************************************
 */
void set_pad_functions(void)        // set gpio port function mode
{
    
#if BLE_PROX_REPORTER
	
   GPIO_ConfigurePin( GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, INPUT_PULLUP, PID_GPIO, false ); // Push Button 
#endif
	
#if USE_MOTOR
//	GPIO_ConfigurePin( GPIO_ALERT_MOTOR_PORT, GPIO_ALERT_MOTOR_PIN, OUTPUT, PID_GPIO, false ); //MOTOR
#endif


#if BLE_BATT_SERVER    
//    GPIO_ConfigurePin( GPIO_BAT_LED_PORT, GPIO_BAT_LED_PIN, OUTPUT, PID_GPIO, false ); //Battery alert LED
		GPIO_ConfigurePin( GPIO_BAT_DET_PORT, GPIO_BAT_DET_PIN, INPUT, PID_ADC, false ); //gsx,ADC-DEC
//		GPIO_ConfigurePin( GPIO_ALERT_CHRG_PORT, GPIO_ALERT_CHRG_PIN, INPUT_PULLUP, PID_GPIO, false ); //gsx,CHRG,充电指示.//V.III版本CHRG与PWR互换对应引脚，但名称未变
#endif

	GPIO_ConfigurePin( GPIO_ALERT_PWR_PORT, GPIO_ALERT_PWR_PIN, OUTPUT, PID_GPIO, false ); //Alert LED
	GPIO_ConfigurePin( LED_B_PORT, LED_B_PIN, OUTPUT, PID_GPIO, false ); //Alert LED
	GPIO_ConfigurePin( LED_O_PORT, LED_O_PIN, OUTPUT, PID_GPIO, false ); //Alert LED
	
	GPIO_ConfigurePin(UART1_PORT, UART1_TX_PIN, OUTPUT, PID_UART1_TX, false);
	GPIO_ConfigurePin(UART1_PORT, UART1_RX_PIN, INPUT, PID_UART1_RX, false);
	
	GPIO_ConfigurePin(I2C_PORT, I2C_PIN_SCL, OUTPUT, PID_I2C_SCL, true);
	GPIO_ConfigurePin(I2C_PORT, I2C_PIN_SDA, OUTPUT, PID_I2C_SDA, true);
}
#if 0
void pwm_init(void)            
{
	set_tmr_enable(CLK_PER_REG_TMR_ENABLED);    //enable timer0 timer2
  set_tmr_div(CLK_PER_REG_TMR_DIV_8);         //   8分频
	//timer2_enable(TRIPLE_PWM_ENABLED);   
  timer2_set_hw_pause(HW_CAN_NOT_PAUSE_PWM_2_3_4);
  timer2_set_sw_pause(PWM_2_3_4_SW_PAUSE_DISABLED );
  timer2_set_pwm_frequency(500);                 //pwm  时钟频率(16M/8)/500=4KHZ
	timer2_set_pwm3_duty_cycle(400);                //设置占空比 400/500  80%
}
#endif
/**
 ****************************************************************************************
 * @brief Enable pad's and peripheral clocks assuming that peripherals' power domain is down. The Uart and SPi clocks are set.
 *
 * @return void
 ****************************************************************************************
 */
void periph_init(void)  // set i2c, spi, uart, uart2 serial clks
{
	// Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP)) ; 
    
    SetBits16(CLK_16M_REG,XTAL16_BIAS_SH_DISABLE, 1);
	
	//rom patch
	patch_func();
	
	//Init pads
	set_pad_functions();
   //init pwm    
	//  pwm_init();       

#if (BLE_APP_PRESENT)
    
#if BLE_PROX_REPORTER
    app_proxr_port_reinit(GPIO_ALERT_LED1_PORT, GPIO_ALERT_LED1_PIN);
//		app_proxr_init(GPIO_ALERT_LED1_PORT, GPIO_ALERT_LED1_PIN);
    app_button_enable();
#elif BLE_FINDME_LOCATOR
    app_button_enable();
#endif //BLE_PROX_REPORTER
#if BLE_BATTERY_SERVER
    app_batt_port_reinit();
#endif //BLE_BATTERY_SERVER
#endif //BLE_APP_PRESENT
    // Enable the pads
	SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
	
	Button_Setup();
	
	Timer0_Setup();
	
	Uart_init();
	
//	MPU_Init();//function of init MPU6560		
}

#endif //_PERIPH_SETUP_H_


void Button_Setup(void)
{
	//SW-SOS Config(P2_3)
	NVIC_DisableIRQ(GPIO3_IRQn);//SOS
	SetWord16(P23_MODE_REG,INPUT_PULLUP);								// set P2_3
	SetBits16(GPIO_INT_LEVEL_CTRL_REG, EDGE_LEVELn3, 1); //interrupt can be initiated immediately
	SetBits16(GPIO_INT_LEVEL_CTRL_REG, INPUT_LEVEL3, 1); //input will generate GPIO IRQ0 if P2.3 input is low
	SetWord16(GPIO_IRQ3_IN_SEL_REG, 18); 	//P2.3 is selected, GPIO Input push buttton
	
	NVIC_SetPriority(GPIO3_IRQn,2);
	NVIC_EnableIRQ(GPIO3_IRQn);
#if 0	
	//SW1 Config(P2_8)
	NVIC_DisableIRQ(GPIO0_IRQn);//SW1
	SetWord16(P28_MODE_REG,INPUT_PULLUP);								// set P2_8
	SetBits16(GPIO_INT_LEVEL_CTRL_REG, EDGE_LEVELn0, 1); //interrupt can be initiated immediately
	SetBits16(GPIO_INT_LEVEL_CTRL_REG, INPUT_LEVEL0, 1); //input will generate GPIO IRQ0 if P2.8 input is low
	SetWord16(GPIO_IRQ0_IN_SEL_REG, 23); 	//P2.8 is selected, GPIO Input push buttton
	
	NVIC_SetPriority(GPIO0_IRQn,2);
	NVIC_EnableIRQ(GPIO0_IRQn);
	
	//SW2 Config(P2_4)
	NVIC_DisableIRQ(GPIO4_IRQn);//SW2
	SetWord16(P24_MODE_REG,INPUT_PULLUP);								// set P2_4
//	GPIO_SetPinFunction(GPIO_BUTTON_SW_PORT,GPIO_BUTTON_SW2_PIN,INPUT_PULLDOWN,PID_GPIO);
	SetBits16(GPIO_INT_LEVEL_CTRL_REG, EDGE_LEVELn4, 1); //interrupt can be initiated immediately
	SetBits16(GPIO_INT_LEVEL_CTRL_REG, INPUT_LEVEL4, 1); //input will generate GPIO IRQ0 if P2.4 input is low
	SetWord16(GPIO_IRQ4_IN_SEL_REG, 19); 	//P2.4 is selected, GPIO Input push buttton
	
	NVIC_SetPriority(GPIO4_IRQn,2);
	NVIC_EnableIRQ(GPIO4_IRQn);

	//SW3 Config(P2_5)
	NVIC_DisableIRQ(GPIO1_IRQn);//SW1
	SetWord16(P25_MODE_REG,INPUT_PULLUP);								// set P2_5
	SetBits16(GPIO_INT_LEVEL_CTRL_REG, EDGE_LEVELn1, 1); //interrupt can be initiated immediately
	SetBits16(GPIO_INT_LEVEL_CTRL_REG, INPUT_LEVEL1, 1); //input will generate GPIO IRQ0 if P2.5 input is low
	SetWord16(GPIO_IRQ1_IN_SEL_REG, 20); 	//P2.5 is selected, GPIO Input push buttton
	
	NVIC_SetPriority(GPIO1_IRQn,2);
	NVIC_EnableIRQ(GPIO1_IRQn);

	//SW4 Config(P2_9)
	NVIC_DisableIRQ(GPIO2_IRQn);//SW4
	SetWord16(P29_MODE_REG,INPUT_PULLUP);								// set P2_9
	SetBits16(GPIO_INT_LEVEL_CTRL_REG, EDGE_LEVELn2, 1); //interrupt can be initiated immediately
	SetBits16(GPIO_INT_LEVEL_CTRL_REG, INPUT_LEVEL2, 1); //input will generate GPIO IRQ0 if P2.9 input is low
	SetWord16(GPIO_IRQ2_IN_SEL_REG, 24); 	//P2.9 is selected, GPIO Input push buttton
	
	NVIC_SetPriority(GPIO2_IRQn,2);
	NVIC_EnableIRQ(GPIO2_IRQn);	
#endif
}


//SW-SOS Handler(LED1~LED4)
void GPIO3_Handler(void)
{
		static uint8_t test_num = 0;
	
		NVIC_DisableIRQ(GPIO3_IRQn);
		SetWord16(GPIO_RESET_IRQ_REG,1<<3); // reset GPIO3 IRQ
		GPIO_ConfigurePin(  LED_B_PORT,  LED_B_PIN, OUTPUT, PID_GPIO, true);
		app_sample128_write_char2_value(test_num++);//上报数据
	#if (BLE_HR_SENSOR)
		app_heart_rate_set_value(test_num);
	#endif
	
	#if (BLE_ACCEL)
		//updateData();
	#endif
		NVIC_EnableIRQ(GPIO3_IRQn);
}

//SW1 Handler(LED1,LED2)
void GPIO0_Handler(void)
{
		NVIC_DisableIRQ(GPIO0_IRQn);
		SetWord16(GPIO_RESET_IRQ_REG,1); // reset GPIO0 IRQ
		
		NVIC_EnableIRQ(GPIO0_IRQn);
}

//SW2 Handler(LED2,LED3)
void GPIO4_Handler(void)
{
		NVIC_DisableIRQ(GPIO4_IRQn);
		SetWord16(GPIO_RESET_IRQ_REG,1<<4); // reset GPIO4 IRQ 

		NVIC_EnableIRQ(GPIO4_IRQn);
}
//SW3 Handler(LED3,LED4)
void GPIO1_Handler(void)
{		
		NVIC_DisableIRQ(GPIO1_IRQn);
		SetWord16(GPIO_RESET_IRQ_REG,1<<1); // reset GPIO1 IRQ

		NVIC_EnableIRQ(GPIO1_IRQn);
}
//SW4 Handler(LED4,LED1)
void GPIO2_Handler(void)
{
		NVIC_DisableIRQ(GPIO2_IRQn);
		SetWord16(GPIO_RESET_IRQ_REG,1<<2); // reset GPIO2 IRQ

		NVIC_EnableIRQ(GPIO2_IRQn);
}



void Timer0_Setup(void)
{
	set_tmr_enable(CLK_PER_REG_TMR_ENABLED);

	set_tmr_div(CLK_PER_REG_TMR_DIV_8);

	timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_DIV_BY_10);

	timer0_set(512, 0, 0);
	
	timer0_enable_irq();

	timer0_start();
}


/*
void Timer0_Setup(void)
{
	SetBits16(CLK_PER_REG, TMR_ENABLE ,1);    // enable  clock for  TMR DIV=0 
	SetWord16(RESET_FREEZE_REG,FRZ_SWTIM);   
  SetWord16(TIMER0_CTRL_REG,0x0);           // stop timer
  SetWord16(TIMER0_RELOAD_M_REG,0x010A);  // set to 0x7D00/16MHz time out    2ms	//16000=0x3E80(3s)---measured by gsx
  SetWord16(TIMER0_RELOAD_N_REG,0x010A);   // set to 0x7D00/16MHz time out    2ms  //266=0x010A(50ms),0x14D5(1s)
  SetWord16(TIMER0_CTRL_REG,0x07);           // set timer with 16MHz source clock no div 10
	NVIC_SetPriority(SWTIM_IRQn,1);
  NVIC_EnableIRQ(SWTIM_IRQn);           // enable software timer interrupt
//	NVIC_DisableIRQ(SWTIM_IRQn);
	jplus_timer_state=NONE_STATE;
}
*/

extern void app_batt_lvl(void);

extern void app_batt_set_level(uint8_t batt_lvl);//电池电压值
extern void sample128_send_val(uint8_t val1,uint8_t val2);

extern uint8_t ledb_sw;
/**定时器频率为50ms**/
void SWTIM_Handler(void) 
{
	SetWord16(TIMER0_CTRL_REG,TIM0_CTRL);//Timer0 is off and in reset state
	static uint8_t state_led=0;
	static uint8_t count_led=0;
	unsigned char data_who = 0;
//	count_led++;

	if(1)//(count_led==10)
	{
		if(state_led==0)
		{
			GPIO_ConfigurePin(  LED_O_PORT,  LED_O_PIN, OUTPUT, PID_GPIO, true);
			if(ledb_sw==1)
				GPIO_ConfigurePin(  LED_B_PORT,  LED_B_PIN, OUTPUT, PID_GPIO, false);
			state_led=1;
		}
		else
		{
			GPIO_ConfigurePin(  LED_O_PORT,  LED_O_PIN, OUTPUT, PID_GPIO, false);
			if(ledb_sw==1)
				GPIO_ConfigurePin(  LED_B_PORT,  LED_B_PIN, OUTPUT, PID_GPIO, true);
			state_led=0;
		}
		count_led=0;
	}
	
//	get_mpu_id(&data_who);
//	uart_send_byte(data_who);
	
}

