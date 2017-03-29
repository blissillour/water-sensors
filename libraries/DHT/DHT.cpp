

#include "DHT.h"

#define DHT_SUCCESS 0
#define DHT_TIMEOUT_ERROR 1  
#define DHT_CHECKSUM_ERROR 2 

DHT::DHT(uint8_t dht_Pin){
	DHT::dht_Pin = dht_Pin;
	pinMode(dht_Pin, INPUT_PULLUP);
}

byte DHT::readDHT11(float* temperature, float* humidity) {
  
  byte data[5];
  byte ret = readDHTxx(data, 18, 1000);
  
  if (ret != DHT_SUCCESS) 
    return ret;
    
  *humidity = data[0];
  *temperature = data[2];

  return DHT_SUCCESS;
}

byte DHT::readDHT22(float* temperature, float* humidity) {
  
  byte data[5];
  byte ret = readDHTxx(data, 1, 1000);
  
  if (ret != DHT_SUCCESS) 
    return ret;
    
  float fh = data[0];
  fh *= 256;
  fh += data[1];
  fh *= 0.1;
  *humidity = fh;
 
  float ft = data[2] & 0x7f;
  ft *= 256;
  ft += data[3];
  ft *= 0.1;
  if (data[2] & 0x80) {
    ft *= -1;
  }
  *temperature = ft;

  /* Ok */
  return DHT_SUCCESS;
}

byte DHT::readDHTxx(byte* data, unsigned long start_time, unsigned long timeout) {
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
 
  uint8_t bit = digitalPinToBitMask(dht_Pin);
  uint8_t port = digitalPinToPort(dht_Pin);
  volatile uint8_t *ddr = portModeRegister(port);   
  volatile uint8_t *out = portOutputRegister(port); 
  volatile uint8_t *in = portInputRegister(port);   
  
  unsigned long max_cycles = microsecondsToClockCycles(timeout);
 
  *out |= bit;  // PULLUP
  *ddr &= ~bit; // INPUT
  delay(100);   
 
  *ddr |= bit;  // OUTPUT
  *out &= ~bit; // LOW
  delay(start_time); 
  noInterrupts();
  
  *out |= bit;  // PULLUP
  delayMicroseconds(40);
  *ddr &= ~bit; // INPUT
 
  timeout = 0;
  while(!(*in & bit)) { 
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }
    
  timeout = 0;
  while(*in & bit) {
    if (++timeout == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
  }

  for (byte i = 0; i < 40; ++i) {
 
    unsigned long cycles_low = 0;
    while(!(*in & bit)) {
      if (++cycles_low == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }

    unsigned long cycles_high = 0;
    while(*in & bit) {
      if (++cycles_high == max_cycles) {
        interrupts();
        return DHT_TIMEOUT_ERROR;
      }
    }
    
    data[i / 8] <<= 1;
    if (cycles_high > cycles_low) {
      data[i / 8] |= 1;
    }
  }
  
  interrupts();
 
  byte checksum = (data[0] + data[1] + data[2] + data[3]) & 0xff;
  if (data[4] != checksum)
    return DHT_CHECKSUM_ERROR; 
  else
    return DHT_SUCCESS;
}
