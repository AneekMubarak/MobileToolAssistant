#ifndef DW1000_H
#define DW1000_H

#include <Arduino.h>

void dw1000_init(uint8_t cs_pin, uint8_t rst_pin);
void dw1000_reset(void);

uint32_t dw1000_read_device_id(void);

void dw1000_read(uint8_t reg, uint8_t *data, uint16_t len);
void dw1000_write(uint8_t reg, const uint8_t *data, uint16_t len);

uint32_t dw1000_read32(uint8_t reg);
void dw1000_write32(uint8_t reg, uint32_t value);

#endif
