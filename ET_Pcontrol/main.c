#define F_CPU 4000000

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

int Kp = 1;
int set_pressure = 50, dutyCycle = 0;
float current_pressure = 0, Kp_error = 0;

void P_control(void);
void TCA0_PWM_init(void);
void PWM_Condition(void);
void continous_intubation(void);

void TCA0_PWM_init(void)
{
	//sei();
	/*		First PWM on PA1 PIN		*/
	/* Direction set and set output to high */
	PORTA.DIR |= (1 << 0);
	PORTA.PIN0CTRL |= (1 << 3);
	//PORTA.OUTSET |= (1 << 0);
	TCA0.SINGLE.PER = 4000;						 // calculation for 1ms
	
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
		_delay_ms(50);
		P_control();
		TCA0.SINGLE.CTRLA |= (1 << 0);
		TCA0.SINGLE.CMP0 = dutyCycle;			//MOTOR PWM

	}
	else if (current_pressure < (set_pressure-5))
	{
		Kp= 1.5;
		//		current_pressure =  Pressure_read();
		P_control();
		TCA0.SINGLE.CTRLA |= (1 << 0);
		TCA0.SINGLE.CMP0 = dutyCycle;			//MOTOR PWM
		// 		PORTE_OUT &= ~(1 << 0);					// SOL 1 OFF
		// 		PORTE_OUT |= (1 << 1);					// SOL 2 ON
	}
}

int main()
{
	//_PROTECTED_WRITE (CLKCTRL.OSCHFCTRLA, ((CLKCTRL_FREQSEL_24M_gc)|(CLKCTRL_AUTOTUNE_bm)));
	sei();
	USART1_init(9600);
	PORTE.DIR |= (1<<0);
	PORTE.DIR |= (1<<1);
	PORTE.OUT |= (1<<0);
	//PORTE.OUT |= (1<<1);
	TCA0_PWM_init();
	Pressure_init();
	//USART1_sendString("hello");
	//_delay_ms(5);
	while (1)
	{
		continous_intubation();
	}
	
}
