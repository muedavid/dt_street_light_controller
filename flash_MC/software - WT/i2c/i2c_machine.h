/*
 * Copyright Brian Starkey 2014 <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __I2C_MACHINE__
#define __I2C_MACHINE__

#include <stdint.h>
#include "i2c_slave_defs.h"
#define FirstRegisterAddress 0x06 //The LED0_ON_L Register on the PCA9685 is at 0x06
#define BytesPerLedPwmChannel 0x04 //For each LED, 4 Byte of date are used in the PCA9685


/* Initialise the USI and I2C state machine */
void i2c_init(void);

/*
 * Check for and handle a stop condition.
 * Returns non-zero if any registers have been changed
 */
uint8_t i2c_check_stop(int8_t SlaveIndex);

/*
 * Return non-zero if a transaction is ongoing
 * A transaction is considered ongoing if the slave address has
 * been matched, but a stop has not been received yet.
 */
uint8_t i2c_transaction_ongoing(void);

extern uint8_t getI2CPWMValue(uint8_t , uint8_t );

/*
 * These need to be instantiated somewhere in your application.
 * I2C_N_REG should be defined in i2c_slave_defs.h
 */
extern volatile uint8_t i2c_reg[I2C_N_SLAVES][I2C_N_REG];
extern volatile void i2c_reg_write_8bit(uint8_t, uint8_t, uint8_t);
extern volatile void i2c_reg_write_16bit(uint8_t, uint8_t, uint16_t);
extern  uint16_t i2c_reg_read_16bit(uint8_t,uint8_t);
extern  uint8_t i2c_reg_read_8bit(uint8_t,uint8_t);

#if !defined(I2C_GLOBAL_WRITE_MASK)
/* See i2c_slave-defs.h */
extern const uint8_t i2c_w_mask[I2C_N_REG];
#endif

#endif /* __I2C_MACHINE__ */
