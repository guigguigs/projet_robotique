#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread

void pi_regulator_start(void);
void set_speed(bool a);
bool get_speed(void);

#endif /* PI_REGULATOR_H */
