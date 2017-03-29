
#ifndef dht_h
#define dht_h

#include <Arduino.h>

class DHT
{
public:

	DHT(uint8_t dht_Pin);
	byte readDHT11(float* temperature, float* humidity);
	byte readDHT22(float* temperature, float* humidity);
	
private:
	byte readDHTxx(byte* data, unsigned long start_time, unsigned long timeout);
	
protected:
	uint8_t dht_Pin;

};
#endif