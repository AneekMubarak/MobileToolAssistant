// #include <SPI.h>

// #define CS_PIN  15   // D8
// #define RST_PIN 16   // D0

// uint32_t dwm_read_device_id() {
//   uint8_t tx[5] = {0};
//   uint8_t rx[5] = {0};

//   digitalWrite(CS_PIN, LOW);

//   for (int i = 0; i < 5; i++) {
//     rx[i] = SPI.transfer(tx[i]);
//   }

//   digitalWrite(CS_PIN, HIGH);

//   return (rx[4] << 24) |
//          (rx[3] << 16) |
//          (rx[2] << 8)  |
//          rx[1];
// }

// void dwm_reset() {
//   pinMode(RST_PIN, OUTPUT);
//   digitalWrite(RST_PIN, LOW);
//   delay(1);

//   pinMode(RST_PIN, INPUT);  // release to high-Z
//   delay(10);
// }

// void setup() {
//   Serial.begin(115200);

//   pinMode(CS_PIN, OUTPUT);
//   digitalWrite(CS_PIN, HIGH);

//   SPI.begin();
//   SPI.setFrequency(1000000); // start slow (2 MHz)
//   SPI.setDataMode(SPI_MODE0);
//   SPI.setBitOrder(MSBFIRST);


//   dwm_reset();

//   // uint32_t id = dwm_read_device_id();
//   // Serial.printf("DEV_ID = 0x%08X\n", id);
// }

// void loop() {
//   uint32_t id = dwm_read_device_id();
//   Serial.printf("DEV_ID = 0x%08X\n", id);
//     delay(100);

// }
#include <Arduino.h>
#include "dw1000.h"

// Pins for CS and RST
#define DW_CS_PIN 15
#define DW_RST_PIN 16

void setup() {
  // Initialize Serial for debug output
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial to be ready (for some boards)

  Serial.println("Initializing DW1000...");

  // Initialize DW1000
  dw1000_init(DW_CS_PIN, DW_RST_PIN);
  
  // Give some time for the device to settle
  delay(10);


}
void loop() {
    uint8_t dev_id_bytes[4] = {0};
    dw1000_read(0x00, dev_id_bytes, 4);

    Serial.print("Read bytes: ");
    for (int i = 0; i < 4; i++) {
        Serial.print("0x");
        if (dev_id_bytes[i] < 16) Serial.print("0");
        Serial.print(dev_id_bytes[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    delay(1000);
}

// void loop() {
//   // Nothing to do here
//     // Read device ID
//   uint8_t dev_id_arr[4] = {0};
//   dw1000_read(0x00,dev_id_arr,4);
//   // uint32_t dev_id = dw1000_read_device_id();

//   // Print device ID
//   // Serial.print("DW1000 Device ID: 0x");
//     uint32_t dev_id = ((uint32_t)dev_id_arr[3] << 24) |
//                       ((uint32_t)dev_id_arr[2] << 16) |
//                       ((uint32_t)dev_id_arr[1] << 8)  |
//                       ((uint32_t)dev_id_arr[0]);

//     Serial.print("DW1000 Device ID: 0x");
//     Serial.println(dev_id, HEX);
//   // Serial.println("------------", HEX);


//   delay(1000);



// }
