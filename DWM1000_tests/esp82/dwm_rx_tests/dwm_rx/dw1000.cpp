#include <SPI.h>

#include "dw1000.h"

static uint8_t DW_CS;
static uint8_t DW_RST;

static void dw_select() {
    digitalWrite(DW_CS, LOW);
}

static void dw_deselect() {
    digitalWrite(DW_CS, HIGH);
}

void dw1000_init(uint8_t cs_pin, uint8_t rst_pin) {
    DW_CS = cs_pin;
    DW_RST = rst_pin;

    pinMode(DW_CS, OUTPUT);
    digitalWrite(DW_CS, HIGH);

    SPI.begin();
    SPI.setFrequency(328125);   // match STM32 baud rate
    SPI.setDataMode(SPI_MODE0); // CPOL=0, CPHA=0
    SPI.setBitOrder(MSBFIRST);  // explicit, matches STM32 MSB first


    dw1000_reset();
}

void dw1000_reset(void) {
    pinMode(DW_RST, OUTPUT);
    digitalWrite(DW_RST, LOW);
    delay(1);

    pinMode(DW_RST, INPUT);
    delay(10);
}

// void dw1000_read(uint8_t reg, uint8_t *data, uint16_t len) {
//     dw_select();

//     SPI.transfer(reg & 0x3F);

//     for (uint16_t i = 0; i < len; i++) {
//         data[i] = SPI.transfer(0x00);
//     }

//     dw_deselect();
// }

void dw1000_read(uint8_t reg, uint8_t *data, uint16_t len) {
    // Buffer for the first byte (header) and dummy bytes
    uint8_t tx[1 + len];
    uint8_t rx[1 + len];

    // Prepare header: read, no sub-index
    tx[0] = reg & 0x3F;

    // Fill dummy bytes (for clocking out during read)
    memset(&tx[1], 0x00, len);

    // Pull CS low to select the DW1000
    dw_select();

    // Send header byte
    rx[0] = SPI.transfer(tx[0]);

    // Send dummy bytes and read actual data
    for (uint16_t i = 0; i < len; i++) {
        rx[i + 1] = SPI.transfer(tx[i + 1]);
    }

    // Release CS
    dw_deselect();

    // Copy received data into user buffer, skipping the first dummy/echo byte
    memcpy(data, &rx[1], len);
}


void dw1000_write(uint8_t reg, const uint8_t *data, uint16_t len) {
    dw_select();

    SPI.transfer(0x80 | (reg & 0x3F));

    for (uint16_t i = 0; i < len; i++) {
        SPI.transfer(data[i]);
    }

    dw_deselect();
}

uint32_t dw1000_read32(uint8_t reg) {
    uint8_t buf[4];
    dw1000_read(reg, buf, 4);

    return ((uint32_t)buf[0]) |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

void dw1000_write32(uint8_t reg, uint32_t value) {
    uint8_t buf[4];

    buf[0] = value & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = (value >> 16) & 0xFF;
    buf[3] = (value >> 24) & 0xFF;

    dw1000_write(reg, buf, 4);
}

uint32_t dw1000_read_device_id(void) {
    
    return dw1000_read32(0x00);
}
