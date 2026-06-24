#ifndef I2C_BUS_H_
#define I2C_BUS_H_

#include <stdbool.h>
#include <stdint.h>

void i2c_bus_init(void);
bool i2c_bus_write(uint8_t addr7, const uint8_t *data, uint16_t len);
bool i2c_bus_write_reg(uint8_t addr7, uint8_t reg, uint8_t value);

#endif
