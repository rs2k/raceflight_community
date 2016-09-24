/*
 * This file is part of RaceFlight.
 *
 * RaceFlight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RaceFlight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RaceFlight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "build_config.h"

#include "io.h"
#include "system.h"

#include "bus_i2c.h"
#include "nvic.h"
#include "io_impl.h"
#include "rcc.h"

#ifndef SOFT_I2C

static void i2c_er_handler(I2CDevice device);
static void i2c_ev_handler(I2CDevice device);
static void i2cUnstick(IO_t scl, IO_t sda);

#define GPIO_AF_I2C GPIO_AF_I2C1

#if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)

#if defined(USE_I2C_PULLUP)
#define IOCFG_I2C IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_UP)
#else
#define IOCFG_I2C IOCFG_AF_OD
#endif

#ifndef I2C1_SCL
#define I2C1_SCL PB8
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB9
#endif
#else
#ifndef I2C1_SCL
#define I2C1_SCL PB6
#endif
#ifndef I2C1_SDA
#define I2C1_SDA PB7
#endif
#endif

#ifndef I2C2_SCL
#define I2C2_SCL PB10
#endif
#ifndef I2C2_SDA
#define I2C2_SDA PB11
#endif

#if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
#ifndef I2C3_SCL
#define I2C3_SCL PA8
#endif
#ifndef I2C3_SDA
#define I2C3_SDA PB4
#endif
#endif

static i2cDevice_t i2cHardwareMap[] = {
    { .dev = I2C1, .scl = IO_TAG(I2C1_SCL), .sda = IO_TAG(I2C1_SDA), .rcc = RCC_APB1(I2C1), .overClock = true, .ev_irq = I2C1_EV_IRQn, .er_irq = I2C1_ER_IRQn },
    { .dev = I2C2, .scl = IO_TAG(I2C2_SCL), .sda = IO_TAG(I2C2_SDA), .rcc = RCC_APB1(I2C2), .overClock = true, .ev_irq = I2C2_EV_IRQn, .er_irq = I2C2_ER_IRQn },
#if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
    { .dev = I2C3, .scl = IO_TAG(I2C3_SCL), .sda = IO_TAG(I2C3_SDA), .rcc = RCC_APB1(I2C3), .overClock = false, .ev_irq = I2C3_EV_IRQn, .er_irq = I2C3_ER_IRQn }
#endif
};

#define I2C_DEFAULT_TIMEOUT 30000
static volatile uint16_t i2cErrorCount = 0;

static i2cState_t i2cState[] = {
	{ false, false, 0, 0, 0, 0, 0, 0, 0 },
	{ false, false, 0, 0, 0, 0, 0, 0, 0 },
	{ false, false, 0, 0, 0, 0, 0, 0, 0 }
};

void I2C1_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_1);
}

void I2C1_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_1);
}

void I2C2_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_2);
}

void I2C2_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_2);
}

void I2C3_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_3);
}

void I2C3_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_3);
}

static bool i2cHandleHardwareFailure(I2CDevice device) {
    i2cErrorCount++;
    
    i2cInit(device);
    return false;
}

bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data) {

    if (device == I2CINVALID)
        return false;

    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    I2C_TypeDef *I2Cx;
    I2Cx = i2cHardwareMap[device].dev;

    i2cState_t *state;
    state = &(i2cState[device]);

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 1;
    state->reading = 0;
    state->write_p = data;
    state->read_p = data;
    state->bytes = len_;
    state->busy = 1;
    state->error = false;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    
        if (!(I2Cx->CR1 & I2C_CR1_START)) {                             
            while (I2Cx->CR1 & I2C_CR1_STOP && --timeout > 0) { ; }     
            if (timeout == 0)
                return i2cHandleHardwareFailure(device);
            I2C_GenerateSTART(I2Cx, ENABLE);                            
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (state->busy && --timeout > 0) { ; }
    if (timeout == 0)
        return i2cHandleHardwareFailure(device);

    return !(state->error);
}

bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data) {
    return i2cWriteBuffer(device, addr_, reg_, 1, &data);
}

bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf) {

    if (device == I2CINVALID)
        return false;

    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    I2C_TypeDef *I2Cx;
    I2Cx = i2cHardwareMap[device].dev;

    i2cState_t *state;
    state = &(i2cState[device]);

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 0;
    state->reading = 1;
    state->read_p = buf;
    state->write_p = buf;
    state->bytes = len;
    state->busy = 1;
    state->error = false;

    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                    
        if (!(I2Cx->CR1 & I2C_CR1_START)) {                             
            while (I2Cx->CR1 & I2C_CR1_STOP && --timeout > 0) { ; }     
            if (timeout == 0)
                return i2cHandleHardwareFailure(device);
            I2C_GenerateSTART(I2Cx, ENABLE);                            
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            
    }

    timeout = I2C_DEFAULT_TIMEOUT;
    while (state->busy && --timeout > 0) { ; }
    if (timeout == 0)
        return i2cHandleHardwareFailure(device);

    return !(state->error);
}

static void i2c_er_handler(I2CDevice device) {

    I2C_TypeDef *I2Cx;
    I2Cx = i2cHardwareMap[device].dev;

    i2cState_t *state;
    state = &(i2cState[device]);

    
    volatile uint32_t SR1Register = I2Cx->SR1;

    if (SR1Register & 0x0F00)                                           
        state->error = true;

    
    if (SR1Register & 0x0700) {
        (void)I2Cx->SR2;                                                        
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                                
        if (!(SR1Register & I2C_SR1_ARLO) && !(I2Cx->CR1 & I2C_CR1_STOP)) {     
            if (I2Cx->CR1 & I2C_CR1_START) {                                    
                while (I2Cx->CR1 & I2C_CR1_START) { ; }                         
                I2C_GenerateSTOP(I2Cx, ENABLE);                                 
                while (I2Cx->CR1 & I2C_CR1_STOP) { ; }                          
                i2cInit(device);                                                
            } else {
                I2C_GenerateSTOP(I2Cx, ENABLE);                                 
                I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);           
            }
        }
    }
    I2Cx->SR1 &= ~0x0F00;                                                       
    state->busy = 0;
}

void i2c_ev_handler(I2CDevice device) {

    I2C_TypeDef *I2Cx;
    I2Cx = i2cHardwareMap[device].dev;

    i2cState_t *state;
    state = &(i2cState[device]);

    static uint8_t subaddress_sent, final_stop;                                 
    static int8_t index;                                                        
    uint8_t SReg_1 = I2Cx->SR1;                                                 

    if (SReg_1 & I2C_SR1_SB) {                                                  
        I2Cx->CR1 &= ~I2C_CR1_POS;                                              
        I2C_AcknowledgeConfig(I2Cx, ENABLE);                                    
        index = 0;                                                              
        if (state->reading && (subaddress_sent || 0xFF == state->reg)) {          
            subaddress_sent = 1;                                                
            if (state->bytes == 2)
                I2Cx->CR1 |= I2C_CR1_POS;                                       
            I2C_Send7bitAddress(I2Cx, state->addr, I2C_Direction_Receiver);      
        } else {                                                                
            I2C_Send7bitAddress(I2Cx, state->addr, I2C_Direction_Transmitter);   
            if (state->reg != 0xFF)                                              
                index = -1;                                                     
        }
    } else if (SReg_1 & I2C_SR1_ADDR) {                                         
        
        __DMB();                                                                
        if (state->bytes == 1 && state->reading && subaddress_sent) {             
            I2C_AcknowledgeConfig(I2Cx, DISABLE);                               
            __DMB();
            (void)I2Cx->SR2;                                                    
            I2C_GenerateSTOP(I2Cx, ENABLE);                                     
            final_stop = 1;
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                     
        } else {                                                        
            (void)I2Cx->SR2;                                            
            __DMB();
            if (state->bytes == 2 && state->reading && subaddress_sent) {         
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                           
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        
            } else if (state->bytes == 3 && state->reading && subaddress_sent)    
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        
            else                                                                
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
        }
    } else if (SReg_1 & I2C_SR1_BTF) {                                  
        final_stop = 1;
        if (state->reading && subaddress_sent) {                         
            if (state->bytes > 2) {                                      
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   
                state->read_p[index++] = (uint8_t)I2Cx->DR;              
                I2C_GenerateSTOP(I2Cx, ENABLE);                         
                final_stop = 1;                                         
                state->read_p[index++] = (uint8_t)I2Cx->DR;              
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                 
            } else {                                                    
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    
                state->read_p[index++] = (uint8_t)I2Cx->DR;                    
                state->read_p[index++] = (uint8_t)I2Cx->DR;                    
                index++;                                                
            }
        } else {                                                        
            if (subaddress_sent || (state->writing)) {
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    
                index++;                                                
            } else {                                                    
                I2C_GenerateSTART(I2Cx, ENABLE);                        
                subaddress_sent = 1;                                    
            }
        }
        
        while (I2Cx->CR1 & 0x0100) { ; }
    } else if (SReg_1 & I2C_SR1_RXNE) {                                 
        state->read_p[index++] = (uint8_t)I2Cx->DR;
        if (state->bytes == (index + 3))
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                    
        if (state->bytes == index)                                             
            index++;                                                    
    } else if (SReg_1 & I2C_SR1_TXE) {                                  
        if (index != -1) {                                              
            I2Cx->DR = state->write_p[index++];
            if (state->bytes == index)                                         
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                
        } else {
            index++;
            I2Cx->DR = state->reg;                                             
            if (state->reading || !(state->bytes))                                      
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                
        }
    }
    if (index == state->bytes + 1) {                                           
        subaddress_sent = 0;                                            
        if (final_stop)                                                 
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       
        state->busy = 0;
    }
}

void i2cInit(I2CDevice device)
{
    if (device == I2CINVALID)
        return;

    i2cDevice_t *i2c;
	i2c = &(i2cHardwareMap[device]);

    NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2cInit;

	IO_t scl = IOGetByTag(i2c->scl);
	IO_t sda = IOGetByTag(i2c->sda);

    IOInit(scl, OWNER_SYSTEM, RESOURCE_I2C);
    IOInit(sda, OWNER_SYSTEM, RESOURCE_I2C);

    
	RCC_ClockCmd(i2c->rcc, ENABLE);

	I2C_ITConfig(i2c->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    i2cUnstick(scl, sda);

    
#if defined(STM32F40_41xxx) || defined(STM32F411xE) || defined(STM32F446xx)
    IOConfigGPIOAF(scl, IOCFG_I2C, GPIO_AF_I2C);
    IOConfigGPIOAF(sda, IOCFG_I2C, GPIO_AF_I2C);
#else
    IOConfigGPIO(scl, IOCFG_AF_OD);
    IOConfigGPIO(sda, IOCFG_AF_OD);
#endif

	I2C_DeInit(i2c->dev);
    I2C_StructInit(&i2cInit);

	I2C_ITConfig(i2c->dev, I2C_IT_EVT | I2C_IT_ERR, DISABLE);               
	i2cInit.I2C_Mode = I2C_Mode_I2C;
	i2cInit.I2C_DutyCycle = I2C_DutyCycle_2;
	i2cInit.I2C_OwnAddress1 = 0;
	i2cInit.I2C_Ack = I2C_Ack_Enable;
	i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

	if (i2c->overClock) {
	    i2cInit.I2C_ClockSpeed = 800000; 
    } else {
	    i2cInit.I2C_ClockSpeed = 400000; 
    }

	I2C_Cmd(i2c->dev, ENABLE);
	I2C_Init(i2c->dev, &i2cInit);

    
	nvic.NVIC_IRQChannel = i2c->er_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    
	nvic.NVIC_IRQChannel = i2c->ev_irq;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV);
	NVIC_Init(&nvic);
}

uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

static void i2cUnstick(IO_t scl, IO_t sda)
{
    int i;
    int timeout = 100;

    IOHi(scl);
    IOHi(sda);

    IOConfigGPIO(scl, IOCFG_OUT_OD);
    IOConfigGPIO(sda, IOCFG_OUT_OD);

    for (i = 0; i < 8; i++) {
        
        while (!IORead(scl) && timeout) {
            delayMicroseconds(10);
            timeout--;
        }

        
        IOLo(scl); 
        delayMicroseconds(10);
        IOHi(scl); 
        delayMicroseconds(10);
    }

    
    IOLo(sda); 
    delayMicroseconds(10);
    IOLo(scl); 
    delayMicroseconds(10);
    IOHi(scl); 
    delayMicroseconds(10);
    IOHi(sda); 
}

#endif
