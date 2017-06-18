#include <Arduino.h>
#include <Wire.h>

#include <VirtualWire.h>

#define DEBUG_ENABLED

#ifdef DEBUG_ENABLED
#define DBG(x) x
#else
#define DBG(x)
#endif

#define EV_NONE 0
#define EV_MODEL 1
#define EV_KEYS 2

/*
NES Classic controller connector:
  ________
  | 1 2 3 |
  | 4   5 |
  |__---__|

1. SDA (green)
2. Device Detect (white) / NC
3- VCC 3.3V (red)
4. GND (black)
5. SCL (yellow)
*/
#define RF_POWER_PIN 5
#define RF_RX_PIN 6
#define RXLED 17
#define LED_PIN 15

#define I2C_ADDR 0x52

uint8_t _registers[256];
uint8_t _model[8] = {0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00};
uint8_t _keys[21] = {0x84, 0x86, 0x86, 0x86, 0x00, 0x00, 0xFF, 0xFF,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t event_type = EV_NONE;
uint8_t last_byte = 0;
uint32_t last_time=0;

void writePressedKeys() {
  for(size_t i=0; i < sizeof(_keys);i++) {
    Wire.write(_keys[i]);
  }
}

void writeModel() {
  for(int i=0; i < sizeof(_keys);i++) {
    Wire.write(_model[i]);
  }
}

void setRegister(uint8_t addr, uint8_t value) {
  //Serial1.println("setRegister("+String(addr, HEX)+", "+String(value, HEX)+")");
  switch (addr) {
    case 0xFE:
      _model[4] = value;
      break;
  }
}

void requestEvent() {
  switch(event_type) {
    case EV_MODEL:
      writeModel();
      break;
    case EV_KEYS:
      writePressedKeys();
      break;
    default:
      Serial1.print("unexpected event: 0x"+String(event_type, HEX)+"last byte: 0x"+String(last_byte,HEX));
  }
  event_type = EV_NONE;
}

void receiveEvent(int howMany) {
  while (Wire.available() >= 1) {
    last_byte = Wire.read(); // receive byte as a character

    switch (last_byte) {
      case 0xF0:
      case 0xFB:
      case 0xFE:
        setRegister(last_byte, (uint8_t)Wire.read());
        break;
      case 0xFA:
        event_type = EV_MODEL;
        break;
      case 0x00:
        event_type = EV_KEYS;
        break;
      default:
        Serial1.print("rcv: 0x"+String(last_byte, HEX));
    }

  }
}


void setup()
{
  pinMode(RF_POWER_PIN, OUTPUT);
  digitalWrite(RF_POWER_PIN, HIGH);

  DBG(Serial1.begin(115200);)
  DBG(while (!Serial1);)             // Leonardo: wait for serial monitor
  Wire.begin(I2C_ADDR);                // join i2c bus with address I2C_ADDR
  Wire.onReceive(receiveEvent); // register for receive event
  Wire.onRequest(requestEvent); // register for request event

  vw_set_rx_pin(RF_RX_PIN);  // Setup receive pin.
  vw_setup(2600); // Transmission speed in bits per second.
  vw_rx_start(); // Start the PLL receiver.
}

void loop()
{
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  if(vw_get_message(buf, &buflen)) // non-blocking I/O
  {
    if(buflen >= 2) {
      _keys[6] = buf[0];
      _keys[7] = buf[1];
      Serial1.print(_keys[6], HEX);
      Serial1.print(_keys[7], HEX);
      Serial1.print(buflen, DEC);
      Serial1.print(" ");
      Serial1.println(millis()-last_time, DEC);
      last_time=millis();
    }
    if(_keys[6] == 0xFF && _keys[7] == 0xFF) {
      digitalWrite(RXLED, HIGH);
    } else {
      digitalWrite(RXLED, LOW);
    }
  }
}
