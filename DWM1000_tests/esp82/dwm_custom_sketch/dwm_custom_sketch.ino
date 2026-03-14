#include <DW1000_custom.h>
#define PIN_CS  D8
#define PIN_RST D0

DW1000_custom dw(PIN_CS, PIN_RST);

void setup() {
  Serial.begin(115200);
  delay(1000); // give time for the serial port to initialize
  Serial.println("Starting DW1000 TX test...");
  dw.begin();
}


void loop() {
  const char msg[] = "Hello Pi!";

  dw.clearTransmitStatus();
  dw.sendData((uint8_t*)msg, sizeof(msg));
  

  // wait until TX is done
  while (!dw.isTransmitDone()) {
    Serial.println("Hi");
    delay(1);
  }

  Serial.println("Message sent!");
  delay(1000); // 1-second interval
}
