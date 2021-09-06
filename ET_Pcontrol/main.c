#define F_CPU 24000000

#include <util/delay.h>
#include <avr/io.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/xmega.h>
#include "LIBRARIES/UART_1_AVR128DA64.h"
#include "LIBRARIES/ADC_AVR128DA64.h"
#include "LIBRARIES/I2C_0_AVR128DA64.h"
#include "LIBRARIES/MPRSS_READ.h"
#include "LIBRARIES/RTC_AVR128DA64.h"


/************DIFFERENT SOLENOID CONFIGURATIONS**************/
#define both_sleev_extubate 0
#define sleev1_intubate  1
#define sleev1_extubate  2
#define sleev2_intubate  3
#define sleev2_extubate  4

#define clock_values 24000
#define sol1_port	PORTE
#define sol1_pin	(1<<0)
#define sol2_port	PORTE
#define sol2_pin	(1<<1)
#define max_pressure_limit	45
#define motor_port	PORTA
#define motor_pin	(1<<0)


int working_config_mode = 0;
unsigned long mode_change_timmer = 0;
bool mode_change_flag = false;

int Kp = 1;
int set_pressure = 30, dutyCycle = 0;
float current_pressure = 0, Kp_error = 0;

void P_control(void);
void TCA0_PWM_init(void);
void PWM_Condition(void);
void continous_intubation(void);
void gpio_config(void);
void TCB1_init(void);
void sleev1_intubate_config(void);
void sleev2_intubate_config(void);
void sleev1_extubate_config(void);
void sleev2_extubate_config(void);


void TCA0_PWM_init(void)
{
	//sei();
	/*		First PWM on PA1 PIN		*/
	/* Direction set and set output to high */
	PORTA.DIR |= (1 << 0);
	PORTA.PIN0CTRL |= (1 << 3);
	//PORTA.OUTSET |= (1 << 0);
	TCA0.SINGLE.PER = 3999;						 // calculation for 1ms
	
	TCA0.SINGLE.CNT = 0;

	TCA0.SINGLE.CTRLA |= (0x0 << 1);			//Ftca = fclk_per
	TCA0.SINGLE.CTRLB |= (1 << 4);				// compare 0 enable
	TCA0.SINGLE.CTRLB |= (0x3 << 0);			// Single slope PWM
	TCA0.SINGLE.CTRLA |= (1 << 0);				// Enable PWM
	
	/*		Second PWM on PA1 PIN		*/
	
	// 	PORTA.DIR |= (1 << 1);						// Direction set and set output to high
	// 	PORTA.PIN1CTRL |= (1 << 3);
	// 	TCA0.SINGLE.CTRLB |= (1 << 5);				// Enable Compare Pa1
	// 	TCA0.SINGLE.CTRLA |= (1 << 1);				// Enable PWM PA1
	// 	TCA0.SINGLE.CMP1 = 2500;					// Duty cycle
	
}


void P_control(void)
{
	Kp_error = (set_pressure ) - current_pressure;
	dutyCycle = dutyCycle + (Kp*Kp_error);			//OUTPUT ON PA0 PIN
//	USART1_sendInt(dutyCycle);
	PWM_Condition();
	
}

void PWM_Condition(void)
{
	if (dutyCycle > 4096)
	{
		dutyCycle  = 4096;
	}
	else if (dutyCycle < 0)
	{
		dutyCycle = 0;
	}
}


void continous_intubation(void)
{
	
	current_pressure = Pressure_read();
	P_control();
	if ((current_pressure < (set_pressure + 1)) || (current_pressure > (set_pressure - 1)))
	{
		Kp = 1.2;
		current_pressure =  Pressure_read();
		_delay_ms(50);
		P_control();
		TCA0.SINGLE.CTRLA |= (1 << 0);
		TCA0.SINGLE.CMP0 = dutyCycle;			//MOTOR PWM

	}
	else if (current_pressure < (set_pressure-5))
	{
		Kp= 1.5;
		current_pressure =  Pressure_read();
		P_control();
		TCA0.SINGLE.CTRLA |= (1 << 0);
		TCA0.SINGLE.CMP0 = dutyCycle;			//MOTOR PWM
		// 		PORTE_OUT &= ~(1 << 0);					// SOL 1 OFF
		// 		PORTE_OUT |= (1 << 1);					// SOL 2 ON
	}
// 	if (current_pressure < (set_pressure-10))
// 	{
// 		Kp= 1.4;
// 		//		current_pressure =  Pressure_read();
// 		P_control();
// 		TCA0.SINGLE.CTRLA |= (1 << 0);
// 		TCA0.SINGLE.CMP0 = dutyCycle;			//MOTOR PWM
// 		// 		PORTE_OUT &= ~(1 << 0);					// SOL 1 OFF
// 		// 		PORTE_OUT |= (1 << 1);					// SOL 2 ON
// 
// 	}
// 	else ()
// 	{
// 		Kp = 1;
// 		_delay_ms(50);
// 		P_control();
// 		TCA0.SINGLE.CTRLA |= (1 << 0);
// 		TCA0.SINGLE.CMP0 = dutyCycle;			//MOTOR PWM
// 
// 	}

// 	else if (current_pressure < (set_pressure- 10))
// 	{
// 		Kp= 1.7;
// 		//		current_pressure =  Pressure_read();
// 		P_control();
// 		TCA0.SINGLE.CTRLA |= (1 << 0);
// 		TCA0.SINGLE.CMP0 = dutyCycle;			//MOTOR PWM
// 		// 		PORTE_OUT &= ~(1 << 0);					// SOL 1 OFF
// 		// 		PORTE_OUT |= (1 << 1);					// SOL 2 ON
// 	}
}

// int main()
// {
// 	//_PROTECTED_WRITE (CLKCTRL.OSCHFCTRLA, ((CLKCTRL_FREQSEL_24M_gc)|(CLKCTRL_AUTOTUNE_bm)));
// 	sei();
// 	USART1_init(9600);
// 	gpio_config();
// 	TCB1_init();
// 	TCA0_PWM_init();
// 	Pressure_init();
// 	//USART1_sendString("hello");
// 	//_delay_ms(5);
// 	while (1)
// 	{
// 		continous_intubation();
// 	}
// 	
// }
















void gpio_config(void)
{
	sol1_port.DIRSET |= sol1_pin;
	sol2_port.DIRSET |= sol2_pin;
//	motor_port.DIRSET |= motor_pin;
}

int main(void)
{
	sei();
	_PROTECTED_WRITE (CLKCTRL.OSCHFCTRLA, ((CLKCTRL_FREQSEL_24M_gc)|(CLKCTRL_AUTOTUNE_bm)));
	USART1_init(9600);
	gpio_config();
	TCB1_init();
	TCA0_PWM_init();
	Pressure_init();
	while (1)
	{
		current_pressure = Pressure_read();
		
		if (mode_change_flag == true)
		{
			switch(working_config_mode)
			{
				case sleev1_intubate:	sleev1_intubate_config();
										break;
				case sleev1_extubate:   sleev1_extubate_config();
										break;
				case sleev2_intubate:   sleev2_intubate_config();
										break;
				case sleev2_extubate:	sleev2_extubate_config();
										break;
				default:				break;
			}

// 			if (working_config_mode == sleev1_intubate)
// 			{
// 				sleev1_intubate_config();
// 			}
// 			else if(working_config_mode == sleev1_extubate)
// 			{
// 				sleev1_extubate_config();
// 			}
// 			else if (working_config_mode == sleev2_intubate)
// 			{
// 				sleev2_intubate_config();
// 			}
// 			else if (working_config_mode == sleev2_extubate)
// 			{
// 				sleev2_extubate_config();
// 			}


			mode_change_flag = false;
		}
		
		if ((working_config_mode == sleev1_intubate)||(working_config_mode == sleev2_intubate))
		{
			continous_intubation();
		}
		else
		{
			dutyCycle=0;
			TCA0.SINGLE.CMP0 = 0;
		}
	}
}

void TCB1_init(void)
{
	TCB1_CCMP = clock_values;						// Write a TOP value to the Compare/Capture (TCBn.CCMP) register

	TCB1_CTRLB |= (0x0 << 0);
	TCB1_INTCTRL |= (1<<0);
	
	TCB1_CTRLA |= (1<<0)|(0x0 <<1);		// ENABLE bit in the Control A (TCBn.CTRLA) register,
}


/*******************************************DIFFERENT SOLENOID CONFIGURATIONS*******************************************************/
void sleev1_intubate_config(void)
{
	
	sol1_port.OUT |= sol1_pin;
	sol2_port.OUT |= sol2_pin;
	//  working_config_mode = sleev1_intubate;
}

void sleev2_intubate_config(void)
{
	sol1_port.OUT |= sol1_pin;
	//	_delay_ms(5);
	sol2_port.OUT &= ~(sol2_pin);
	//	working_config_mode = sleev2_intubate;
}


void sleev1_extubate_config(void)
{
	
	sol1_port.OUT &= ~(sol1_pin);
	sol2_port.OUT |= sol2_pin;
	//	working_config_mode = sleev1_extubate;
}

void sleev2_extubate_config(void)
{
	sol1_port.OUT &= ~(sol1_pin);
	sol2_port.OUT &= ~(sol2_pin);
	//	working_config_mode = sleev2_extubate;
}

// void extubate_config(void)
// {
// 	switch(working_config_mode)
// 	{
// 		case sleev1_intubate:	sleev1_extubate_config();
// 								break;
//
// 		case sleev2_intubate:	sleev2_extubate_config();
// 								break;
//
// 		default:	break;
// 	}
// }


ISR (TCB1_INT_vect)
{
	mode_change_timmer++;
	if (mode_change_timmer > 15000)
	{
		working_config_mode++;
		if ((working_config_mode > 4) || (working_config_mode == 0))
		{
			working_config_mode = 1;
		}
		mode_change_timmer = 0;
		mode_change_flag = true;
	}
	if (current_pressure > max_pressure_limit)
	{
		sol1_port.OUT |= sol1_pin;
		motor_port.OUTCLR |= motor_pin;
	}
	TCB1_INTFLAGS |= (1<<0);
}
