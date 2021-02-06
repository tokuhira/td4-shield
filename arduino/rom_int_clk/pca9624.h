/*
 * pca9624.h
 *
 *  Created on: Feb 5, 2021
 *      Origin: https://os.mbed.com/teams/CQ_I2C_book/code/PCA9624/
 */

#ifndef __pca9624_h
#define __pca9624_h

#define PCA9624_ADDR 0x40

#define MODE1            0x00
#define MODE2            0x01
#define PWM0             0x02
#define PWM1             0x03
#define PWM2             0x04
#define PWM3             0x05
#define PWM4             0x06
#define PWM5             0x07
#define PWM6             0x08
#define PWM7             0x09
#define GRPPWM           0x0a
#define GRPFREQ          0x0b
#define LEDOUT0          0x0c
#define LEDOUT1          0x0d
#define SUBADR1          0x0e
#define SUBADR2          0x0f
#define SUBADR3          0x10
#define ALLCALLADR       0x11

#endif /* __pca9624_h */
