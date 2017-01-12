/*
 * main.c
 *
 *  Created on: Feb 10, 2016
 *      Author: kosmaz
 */
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "lcd.h"

void ADC_init()
{
	DDRA &= 0b11111110;	//set PA0 as input

    // AREF = AVcc
    ADMUX = (1<<REFS0);

    // ADC Enable and prescaler of 128
    // 12000000/128 = 93750Hz
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

    // ADC Enable and prescaler of 64
    // 12000000/64 = 187500Hz
    //ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);
    return;
}


// read ADC value
uint16_t ADC_read(uint8_t ch)
{
    // select the corresponding channel 0~7
    // ANDing with '7' will always keep the value
    // of 'ch' between 0 and 7
    ch &= 0b00000111;  // AND operation with 7
    //ADMUX = (ADMUX & 0xF8)|ch;     // clears the bottom 3 bits before ORing
    ADMUX |= ch;

    // start single conversion
    // write '1' to ADSC
    ADCSRA |= (1<<ADSC);

    // wait for conversion to complete
    // ADSC becomes '0' again
    // till then, run loop continuously
    while(ADCSRA & (1<<ADSC));

    return (ADC);
}


void PWM_init()
{
	/*Timer Clock = CPU Clock (No Prescalling)
	Mode        = Fast PWM
	PWM Output  = Non Inverted
	*/

	TCCR0|=(1<<WGM00)|(1<<WGM01)|(1<<COM01)|(1<<CS00);

	//Set OC0 PIN as output. It is  PB3 on ATmega16 ATmega32

	DDRB |= (1<<PB3) | (1<<PB0) | (1<<PB1);

	return;
}


void set_PWM(uint8_t value)
{
	OCR0 = value;
	return;
}



int main()
{
	//initialize ADC, PWM and LCD
	ADC_init();
	PWM_init();
	LCDInit(LS_BLINK | LS_ULINE);

	uint16_t  temperature = 0x00,speed = 0x00;
	char* temp_display="Temp ('C) = ";
	char* speed_display="Speed (%) = ";

	LCDGotoXY(0,0);
	LCDWriteString(temp_display);

	LCDGotoXY(0,1);
	LCDWriteString(speed_display);

	//enable clockwise rotation of DC motor by activating IN1
	PORTB |= 1<<PB1;

	while(1)
	{
		temperature=ADC_read(0)/2;

		LCDGotoXY(strlen(temp_display),0);
		LCDWriteInt(temperature,3);

		if(temperature < 25)
			set_PWM(speed = 0);
		else if(temperature > 25 && temperature <= 35)
			set_PWM(speed = 50);
		else if(temperature > 35 && temperature <= 45)
			set_PWM(speed = 100);
		else if(temperature > 45 && temperature <= 55)
			set_PWM(speed = 150);
		else if(temperature > 55 && temperature <= 65)
			set_PWM(speed = 200);
		else if(temperature >= 65)
			set_PWM(speed = 245);

		LCDGotoXY(strlen(speed_display),1);
		LCDWriteInt(speed,3);

		_delay_ms(10);
	}

	return 0;
}
