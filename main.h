/*
 * main.h
 *
 *  Created on: NA
 *      Author: Tiré du TP4 de robotique 
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//------------constants used in the project-----------
#define KP						3
#define KI 						0.3//must not be zero
#define KD						0.1
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define GOAL_ACC				0.
#define ERROR_THRESHOLD			35.
#define MOTOR_MIN_SPEED 		100
#define MIN_DISTANCE_VALUE		150 //must be ajusted by the user with modification of the code
									//because it depends of the environment


void motors_same_speed(int16_t speed);


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
