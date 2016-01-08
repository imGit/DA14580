/**
 ****************************************************************************************
 *
 * @file periph_setup.h
 *
 * @brief Peripherals setup header file. 
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 
#include "global_io.h"
#include "arch.h"


/*
 * DEFINES
 ****************************************************************************************
 */

#define JPLUS_V3

#define EXT_SPI_FLASH		1	//开启外部存储功能

/* Enable WKUPCT. Required by wkupct_quadec driver. */
#define WKUP_ENABLED

//#define GPIO_ALERT_LED3_PORT     GPIO_PORT_1        
//#define GPIO_ALERT_LED3_PIN      GPIO_PIN_0          

//#define GPIO_BAT_LED_PORT       GPIO_PORT_1        
//#define GPIO_BAT_LED_PIN        GPIO_PIN_2 

#define GPIO_BAT_DET_PORT			GPIO_PORT_0	//gsx
#define GPIO_BAT_DET_PIN			GPIO_PIN_1	//gsx

#ifdef JPLUS_V3

#define GPIO_BUTTON_PORT        GPIO_PORT_2         
#define GPIO_BUTTON_PIN         GPIO_PIN_3 	//唤醒
	
	#define GPIO_ALERT_PWR_PORT     	GPIO_PORT_0	//rughwatch
	#define GPIO_ALERT_PWR_PIN      	GPIO_PIN_2  //rughwatch

	#define GPIO_ALERT_LED1_PORT			GPIO_PORT_2  //gsx
	#define GPIO_ALERT_LED1_PIN      	GPIO_PIN_2   //gsx

	#define GPIO_ALERT_LED2_PORT     	GPIO_PORT_1	//rughwatch
	#define GPIO_ALERT_LED2_PIN      	GPIO_PIN_0  //rughwatch

	#define GPIO_ALERT_LED3_PORT     GPIO_PORT_2        
	#define GPIO_ALERT_LED3_PIN      GPIO_PIN_6  
	
	#define GPIO_ALERT_LED4_PORT			GPIO_PORT_0  //gsx
	#define GPIO_ALERT_LED4_PIN      	GPIO_PIN_3   //gsx

	#define USE_MOTOR											1
	#define GPIO_ALERT_MOTOR_PORT       GPIO_PORT_1 	//dangshaojun GPIO_PORT_1
	#define GPIO_ALERT_MOTOR_PIN        GPIO_PIN_1		//dangshaojun GPIO_PIN_0

	#define GPIO_BUTTON_SW_PORT				GPIO_PORT_2		//gsx
	#define GPIO_BUTTON_SW1_PIN				GPIO_PIN_8		//gsx
	#define GPIO_BUTTON_SW2_PIN				GPIO_PIN_4		//gsx
	#define GPIO_BUTTON_SW3_PIN				GPIO_PIN_5		//gsx
	#define GPIO_BUTTON_SW4_PIN				GPIO_PIN_9		//gsx
	#define GPIO_BUTTON_SOS_PIN				GPIO_PIN_3		//gsx
#endif

#define I2C_PORT    GPIO_PORT_1
#define I2C_PIN_SCL     GPIO_PIN_2
#define I2C_PIN_SDA     GPIO_PIN_3

//#define BAT_ADC_PORT 		GPIO_PORT_0
//#define BAT_ADC_PIN 		GPIO_PIN_1

#define GPIO_PROXR_LED_PORT	GPIO_ALERT_LED2_PORT
#define GPIO_PROXR_LED_PIN GPIO_ALERT_LED2_PIN

#define UART1_PORT GPIO_PORT_0
#define UART1_TX_PIN	GPIO_PIN_4
#define UART1_RX_PIN	GPIO_PIN_5


//spi flash gpio define
#if EXT_SPI_FLASH
#define SPI_CLOCK_PORT 	GPIO_PORT_0
#define SPI_CLOCK_PIN 	GPIO_PIN_0
#define SPI_EN_PORT 		GPIO_PORT_0
#define SPI_EN_PIN 			GPIO_PIN_3
#define SPI_MISO_PORT 	GPIO_PORT_0
#define SPI_MISO_PIN 		GPIO_PIN_5
#define SPI_MOSI_PORT 	GPIO_PORT_0
#define SPI_MOSI_PIN 		GPIO_PIN_6
#endif

#define LED_B_PORT 			GPIO_PORT_2
#define LED_B_PIN 			GPIO_PIN_0
#define LED_O_PORT 			GPIO_PORT_2
#define LED_O_PIN 			GPIO_PIN_2

#define SOS_KEY_PORT 		GPIO_PORT_2
#define SOS_KEY_PIN 		GPIO_PIN_3

#define MPU6050_INT_PORT 	GPIO_PORT_2
#define MPU6050_INT_PIN 	GPIO_PIN_4 
#define PAH8003_SCL_PORT 	GPIO_PORT_2
#define PAH8003_SCL_PIN 	GPIO_PIN_5 
#define PAH8003_PD_PORT 	GPIO_PORT_2
#define PAH8003_PD_PIN 		GPIO_PIN_6 





/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

void periph_init(void);

void GPIO_reservations(void);

void Button_Setup(void);

void Timer0_Setup(void);

void motor_vibrate(uint8_t motor_sw);

