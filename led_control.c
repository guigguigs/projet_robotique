/*
 * led_control.c
 *
 *  Created on: 7 mai 2022
 *      Author: gvull
 */


#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <leds.h>
#include <choc.h>
#include <pi_regulator.h>

extern bool get_arret(void);
extern char get_cote(void);
extern bool get_speed(void);


static THD_WORKING_AREA(waLED, 256);
static THD_FUNCTION(LED, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    char cote = 'n';

    while(1){
    	cote = get_cote();
		if(get_arret()){
			switch (cote){
			case 'n' : 	clear_leds();
						break;
			case 'f' : 	clear_leds();
						set_led(LED1,1);
						break;
			case 'b' : 	clear_leds();
						set_led(LED5,1);
						break;
			}
		}else{
			clear_leds();
		}
		if(get_speed()){
			set_body_led(1);
		}else{
			set_body_led(0);
		}
    }
}

void led_test_start(void){
	chThdCreateStatic(waLED, sizeof(waLED), NORMALPRIO, LED, NULL);
}
