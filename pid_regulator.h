#ifndef PID_REGULATOR_H
#define PID_REGULATOR_H

//start the PID regulator thread

void piD_regulator_start(void);
void set_speed(bool a);
bool get_speed(void);

#endif /* PID_REGULATOR_H */
