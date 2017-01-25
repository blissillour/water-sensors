/*
  WaterSensor.h - Library for using sensors in Waziup Fish Farming MVP
  Created by Benjamin Lissillour, December, 2016.
*/

#ifndef WaterSensor_h
#define WaterSensor_h

#include <Arduino.h>
#include <OneWire.h> 
#include <Wire.h>   
#include <DHT.h> 


class WaterSensor
{
public:

	WaterSensor(uint8_t DS18S20_Pin,uint8_t DHT_Pin, uint8_t PH_address,uint8_t DO_address);
	float getTemperatureValue();

	void wakeUpPHSensor();
	void wakeUpDOSensor();
	void wakeUpSensor(int address);

	void sleepPHSensor();
	void sleepDOSensor();
	void sleepSensor(int address);

	void sendTempToPHSensor(float temp);
	void sendTempToDOSensor(float temp);
	void sendTempToSensor(int address, float temp);

	void getPHSensorValue(char* data);
	void getDOSensorValue(char* data);
	void getSensorValue(int address, char* data);

	void getInboxTemperatureHumidityValue(float* temperature, float* humidity);
	
protected:

	uint8_t DS18S20_Pin;
	uint8_t PH_address;
	uint8_t DO_address;
	OneWire *ds;
	DHT *dht;

};


extern WaterSensor waterSensor;

#endif

