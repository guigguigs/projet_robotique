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
#include <obstacle.h>
#include <pid_regulator.h>
#include <audio\play_melody.h>

extern bool get_arret(void);
extern char get_cote(void);		//'n' for 'no info' ||| 	'f' for 'front' |||	'b' for 'back'
extern bool get_speed(void);

void check_music(uint8_t counter_music){
	while(counter_music < 10){
		chThdSleepMilliseconds(200);
		stopCurrentMelody();
		++counter_music;
	}
	    playMelody(MARIO_DEATH,0,NULL);
	    counter_music = 0;
}

static THD_WORKING_AREA(waLED, 512);
static THD_FUNCTION(LED, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    char cote = 'n';
    static uint8_t counter = 0;
   // static uint8_t counter_music = 0;
    static bool already_played = false;

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
			if(counter < 10){
				set_body_led(0);
				chThdSleepMilliseconds(200);
				++counter;
			}else{
				set_body_led(1);
				if(!already_played){
					playMelody(MARIO_FLAG,0,NULL);
					waitMelodyHasFinished();
					already_played = true;
				}else{
					stopCurrentMelody();
				}

			}
		}else{
			set_body_led(0);
			stopCurrentMelody();
			already_played = false;
			counter = 0;
		}
    }
}

void led_test_start(void){
	chThdCreateStatic(waLED, sizeof(waLED), NORMALPRIO, LED, NULL);
}
