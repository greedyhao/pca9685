/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-15     greedyhao    first version
 */

#ifndef __PCA9685_H
#define __PCA9685_H

#include <rtthread.h>
#include <rtdevice.h>

#define PCA9685_ADDR_DEFAULT    0x40

#define PCA9685_SUBADR1         0x2 /**< i2c bus address 1 */
#define PCA9685_SUBADR2         0x3 /**< i2c bus address 2 */
#define PCA9685_SUBADR3         0x4 /**< i2c bus address 3 */

#define PCA9685_MODE1           0x0 /**< Mode Register 1 */
#define PCA9685_PRESCALE        0xFE /**< Prescaler for PWM output frequency */

#define LED0_ON_L               0x6 /**< LED0 output and brightness control byte 0 */
#define LED0_ON_H               0x7 /**< LED0 output and brightness control byte 1 */
#define LED0_OFF_L              0x8 /**< LED0 output and brightness control byte 2 */
#define LED0_OFF_H              0x9 /**< LED0 output and brightness control byte 3 */

#define ALLLED_ON_L             0xFA /**< load all the LEDn_ON registers, byte 0 */
#define ALLLED_ON_H             0xFB /**< load all the LEDn_ON registers, byte 1 */
#define ALLLED_OFF_L            0xFC /**< load all the LEDn_OFF registers, byte 0 */
#define ALLLED_OFF_H            0xFD /**< load all the LEDn_OFF registers, byte 1 */

/* pca9685 device structure */
struct pca9685_device
{
    rt_device_t bus;
    rt_uint8_t i2c_addr;
};
typedef struct pca9685_device *pca9685_device_t;

/**
 * @brief 
 * 
 * @param dev 
 * @param freq 
 */
rt_err_t pca9685_set_pwm_freq(pca9685_device_t dev, float freq);

void pca9685_set_pwm(pca9685_device_t dev, rt_uint8_t num, rt_uint16_t on, rt_uint16_t off);

void pca9685_restart(pca9685_device_t dev);

/**
 * This function initialize the pca9685 device.
 *
 * @param dev_name the name of i2c bus device
 * @param i2c_addr the i2c device address for i2c communication,RT_NULL use default address
 *
 * @return the pointer of device structure, RT_NULL reprensents  initialization failed.
 */
pca9685_device_t pca9685_init(const char *dev_name, rt_uint8_t i2c_addr);

/**
 * This function releases memory
 *
 * @param dev the pointer of device structure
 */
void pca9685_deinit(struct pca9685_device *dev);

// /**
//  * This function read the data port of pca9685.
//  *
//  * @param dev the pointer of device structure
//  *
//  * @return the state of data port. 0xFF meas all pin is high.
//  */
// uint8_t pca9685_port_read(pca9685_device_t dev);

// /**
//  * This function sets the status of the data port.
//  *
//  * @param dev the pointer of device structure
//  * @param port_val the port value you want to set, 0xFF meas all pin output high.
//  */
// void pca9685_port_write(pca9685_device_t dev, uint8_t port_val);

// /**
//  * This function read the specified port pin of the pca9685.
//  *
//  * @param dev the pointer of device structure
//  * @param pin the specified pin of the data port
//  *
//  * @return the status of the specified data port pin, 0 is low, 1 is high.
//  */
// uint8_t pca9685_pin_read(pca9685_device_t dev, uint8_t pin);

// /**
//  * This function sets the status of the specified port pin.
//  *
//  * @param dev the pointer of device structure
//  * @param pin_val the specified pin value you want to set, 0 is low, 1 is high.
//  */
// void pca9685_pin_write(pca9685_device_t dev, uint8_t pin, uint8_t pin_val);



#endif

