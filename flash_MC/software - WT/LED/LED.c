/*
 * LED.c
 *
 * Created: 23/07/2018 15:19:40
 *  Author: Marcus
 */ 
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include<stdio.h>
#include<stdlib.h>
#include "LED.h"
#include "../i2c/i2c_machine.h"
#include "../i2c/i2c_slave_defs.h"
#include "../ws2812/light_ws2812.h"

volatile uint8_t led_data[N_LEDS*3];

//Calculates all values for the led's and send values
void update_led_values(void)
{
	uint8_t i=0;
	for (i=0;i<N_LEDS*3;i++)
	{
		//duckie: addressable LED channel are as follows: R, G, B duckie
		//WT: addressable LED channel are as follows: G, R, B duckie
		
		led_data[i]=i2c_reg[I2C_SLAVE_LED][i];
	}
	//send led values
	ws2812_sendarray((uint8_t *)led_data, N_LEDS * 3);
}
void init_led_data()
{
	uint8_t i=0;
	for (i=0;i<N_LEDS*3;i++)
	{
		led_data[i]=100;
	}
	//send led values
	ws2812_sendarray((uint8_t *)led_data, N_LEDS * 3);
	
}