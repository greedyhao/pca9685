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

#include "pca9685.h"

// #define DBG_ENABLE
#define DBG_SECTION_NAME     "pca9685"
#define DBG_LEVEL            DBG_INFO
#define DBG_COLOR
#include <rtdbg.h>

/**
 * This function writes the value of the register for pca9685
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for pca9685
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing the value of the register successfully.
 */
static rt_err_t write_reg(pca9685_device_t dev, rt_uint8_t reg, unsigned short length, const unsigned char *data)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[length+1];
    buf[0] = reg;
    int i;
    for ( i = 1; i <= length; i++)
    {
        buf[i] = data[i-1];
    }

#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs.addr  = dev->i2c_addr;    /* slave address */
        msgs.flags = RT_I2C_WR;        /* write flag */
        msgs.buf   = buf;              /* Send data pointer */
        msgs.len   = length+1;

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        res = rt_spi_send_then_send((struct rt_spi_device *)dev->bus, &reg, 1, &data, 1);
#endif
    }
    return res;
}

/**
 * This function reads the value of registers for pca9685
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for pca9685
 * @param len number of register
 * @param buf read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the value of registers successfully.
 */
static rt_err_t read_regs(pca9685_device_t dev, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
#endif
#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &reg;             /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = buf;              /* Read data pointer */
        msgs[1].len   = len;              /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf, len);
#endif
    }
    return res;
}

/**
 * @brief 
 * 
 * @param dev 
 * @param freq 
 */
rt_err_t pca9685_set_pwm_freq(pca9685_device_t dev, float freq)
{
    rt_err_t ret = RT_EOK;
    rt_uint8_t reg[2] = {PCA9685_MODE1};

    freq *= 0.982; // Correct for overshoot in the frequency setting (see issue #11).
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    rt_uint8_t prescale = (rt_uint8_t)(prescaleval + 0.5);
    LOG_I("%d",prescale);

    // rt_uint8_t prescale = 3;

    rt_uint8_t oldmode = 0;
    read_regs(dev, PCA9685_MODE1, 1, &oldmode);
    rt_uint8_t newmode = ((oldmode & 0x7F) | 0x10);

    LOG_D("sleep..");
    ret = write_reg(dev, PCA9685_MODE1, 1, &newmode);            // go to sleep
    if (ret != RT_EOK)
    {
        LOG_E("set sleep fail!");
    }
    read_regs(dev,PCA9685_MODE1, 1, &reg[1]);
    LOG_I("after sleep reg:%p", reg[1]);

    LOG_D("prescaler..");
    ret = write_reg(dev, PCA9685_PRESCALE, 1, &prescale);        // set the prescaler
    if (ret != RT_EOK)
    {
        LOG_E("set prescaler fail!");
    }
    read_regs(dev,PCA9685_MODE1, 1, &reg[1]);
    LOG_I("after prescaler reg:%p", reg[1]);

    LOG_D("old mode..");
    ret = write_reg(dev, PCA9685_MODE1, 1, &oldmode);
    if (ret != RT_EOK)
    {
        LOG_E("set oldmode fail!");
    }
    read_regs(dev,PCA9685_MODE1, 1, &reg[1]);
    LOG_I("after oldmode reg:%p", reg[1]);

    rt_thread_mdelay(5);
    oldmode |= 0xA0;
    reg[1] = oldmode;
    LOG_D("auto run..");
    ret = write_reg(dev, PCA9685_MODE1, 1, &oldmode); //  This sets the MODE1 register to turn on auto increment.

    read_regs(dev,PCA9685_MODE1, 1, &reg[1]);
    LOG_I("after auto reg:%p", reg[1]);

    if (ret != RT_EOK)
    {
        LOG_E("set pwm freq fail!");
    }
    
    return ret;
}

void pca9685_set_pwm(pca9685_device_t dev, rt_uint8_t num, rt_uint16_t on, rt_uint16_t off)
{
    rt_uint8_t outputBuffer[4] = {on, (on >> 8), off, (off >> 8)};
    LOG_D("set pwm..");
    write_reg(dev, LED0_ON_L + 4*num, 4, outputBuffer);
}

void pca9685_restart(pca9685_device_t dev)
{
    rt_uint8_t reg;
    read_regs(dev, PCA9685_MODE1, 1, &reg);
    if ((reg & 0x80) != 0)
    {
        LOG_I("restart..");
        reg = reg & 0xBF;
        write_reg(dev, PCA9685_MODE1, 1, &reg);
        rt_thread_mdelay(2);
        reg = reg | 0x80; //reset
        write_reg(dev, PCA9685_MODE1, 1, &reg);
    }
    
}

pca9685_device_t pca9685_init(const char *dev_name, rt_uint8_t i2c_addr)
{
    rt_uint8_t buffer[] = { PCA9685_MODE1, 0x80 };
    pca9685_device_t dev = RT_NULL;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct pca9685_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for pca9685 device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("i2c_bus %s for PCA9685 not found!", dev_name);
        goto __exit;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        if (i2c_addr != RT_NULL)
            dev->i2c_addr = i2c_addr;
        else
            dev->i2c_addr = PCA9685_ADDR_DEFAULT;
    }
    else
    {
        LOG_E("Unsupported device:'%s'!", dev_name);
        goto __exit;
    }

    LOG_D("reset..");
    /* reset before use it */
    if (write_reg(dev, PCA9685_MODE1, 1, &buffer[1]) != RT_EOK)
    {
        LOG_E("i2c_bus %s for PCA9685 opened failed!", dev_name);
        goto __exit;
    }
    read_regs(dev, PCA9685_MODE1, 1, &buffer[1]);
    LOG_I("rest mode:%p", buffer[1]);

    rt_thread_mdelay(10);

    pca9685_set_pwm_freq(dev, 50);
    

    LOG_D("pca9685 init done", dev_name);
    return dev;

__exit:
    if (dev != RT_NULL)
        rt_free(dev);

    return RT_NULL;
}

void pca9685_deinit(pca9685_device_t dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}
