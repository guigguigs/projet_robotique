#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"


#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <process_image.h>

#include <sensors/imu.h>
#include <i2c_bus.h>
#include <msgbus/messagebus.h>


#define NB_SAMPLES_OFFSET     200
#define MIN_VALUE 	0.2

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


void show_gravity(imu_msg_t *imu_values){

    //variable to measure the time some functions take
    //volatile to not be optimized out by the compiler if not used
    volatile uint16_t time = 0;

    /*
    *   Use this to reset the timer counter and prevent the system
    *   to switch to another thread.
    *   Place it at the beginning of the code you want to measure
    */
    chSysLock();
    //reset the timer counter
    GPTD11.tim->CNT = 0;

    /*
    *   Use this to capture the counter and stop to prevent
    *   the system to switch to another thread.
    *   Place it at the end of the code you want to measure
    */
    time = GPTD11.tim->CNT;
    chSysUnlock();

    float acc_x = imu_values->acceleration [X_AXIS];
    float acc_y = imu_values->acceleration [Y_AXIS];
    palSetPad(GPIOD, GPIOD_LED1);
	palSetPad(GPIOD, GPIOD_LED3);
	palSetPad(GPIOD, GPIOD_LED5);
	palSetPad(GPIOD, GPIOD_LED7);
    if(fabs(acc_x) > fabs(acc_y)){
    	if(acc_x > 0){
    		//palClearPad(GPIOD, GPIOD_LED7);
    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
    	}else{
    		//palClearPad(GPIOD, GPIOD_LED3);
    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
    	}
    }else{
    	if(acc_y < MIN_VALUE && acc_y > -MIN_VALUE){
    		right_motor_set_speed(0);
    		left_motor_set_speed(0);
    		return;
    	}
    	if(acc_y < 0){
    		palClearPad(GPIOD, GPIOD_LED5);
    		right_motor_set_speed(-250);
			left_motor_set_speed(-250);
    	}else{
    		right_motor_set_speed(250);
    		left_motor_set_speed(250);
    		palClearPad(GPIOD, GPIOD_LED1);
    	}
    }
}

int main(void)
{
    /* System init */
    halInit();
    chSysInit();
    serial_start();
    i2c_start();
    imu_start();
    motors_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);


    //to change the priority of the thread invoking the function. The main function in this case
    //chThdSetPriority(NORMALPRIO+2);

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
    //imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);
    calibrate_acc();
    calibrate_gyro();

    while(1){
        //wait for new measures to be published
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
        //prints raw values
//        chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
//                imu_values.acc_raw[X_AXIS], imu_values.acc_raw[Y_AXIS], imu_values.acc_raw[Z_AXIS],
//                imu_values.gyro_raw[X_AXIS], imu_values.gyro_raw[Y_AXIS], imu_values.gyro_raw[Z_AXIS]);

        //prints raw values with offset correction
//        chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n",
//                imu_values.acc_raw[X_AXIS]-imu_values.acc_offset[X_AXIS],
//                imu_values.acc_raw[Y_AXIS]-imu_values.acc_offset[Y_AXIS],
//                imu_values.acc_raw[Z_AXIS]-imu_values.acc_offset[Z_AXIS],
//                imu_values.gyro_raw[X_AXIS]-imu_values.gyro_offset[X_AXIS],
//                imu_values.gyro_raw[Y_AXIS]-imu_values.gyro_offset[Y_AXIS],
//                imu_values.gyro_raw[Z_AXIS]-imu_values.gyro_offset[Z_AXIS]);
//
        //prints values in readable units

        chprintf((BaseSequentialStream *)&SD3, "%Ax=%.2f Ay=%.2f Az=%.2f Gx=%.2f Gy=%.2f Gz=%.2f (%x)\r\n\n",
                imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS],
                imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS], imu_values.gyro_rate[Z_AXIS],
                imu_values.status);

        show_gravity(&imu_values);
        chThdSleepMilliseconds(100);
    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
