#include <SPI.h>

#define PIN_CS  D8
#define PIN_RST D0

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("DW1000 Simple SPI Test");
  
  // Setup pins
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  
  // Reset
  digitalWrite(PIN_RST, LOW);
  delay(10);
  digitalWrite(PIN_RST, HIGH);
  delay(50);
  
  // Initialize SPI
  SPI.begin();
  
  // Try different frequencies
  uint32_t frequencies[] = {1000000, 2000000, 500000, 250000};
  
  for (uint32_t freq : frequencies) {
    SPI.beginTransaction(SPISettings(freq, MSBFIRST, SPI_MODE0));
    
    digitalWrite(PIN_CS, LOW);
    SPI.transfer(0x00);  // Read device ID
    
    Serial.print(freq / 1000);
    Serial.print(" kHz: ");
    
    uint8_t bytes[4];
    for (int i = 0; i < 4; i++) {
      bytes[i] = SPI.transfer(0x00);
      if (bytes[i] < 0x10) Serial.print("0");
      Serial.print(bytes[i], HEX);
      Serial.print(" ");
    }
    
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
    
    Serial.println();
    delay(100);
  }
}

void loop() {}