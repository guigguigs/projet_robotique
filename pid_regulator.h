/*
 * pid_regulator.h
 *
 *  Created on: 2 mai 2022
 *      Author: Guillaume VULLIOUD & Gael ESCHBACH inspiré du TP4 de robotique 
 */

#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

//start the PID regulator thread

void piD_regulator_start(void);
void set_speed(bool a);
bool get_speed(void);

#endif /* PID_REGULATOR_H */
