
#include "WaterSensor.h"

WaterSensor::WaterSensor(uint8_t DS18S20_Pin, uint8_t DHT_Pin, uint8_t PH_address,uint8_t DO_address)
{
	WaterSensor::DS18S20_Pin = DS18S20_Pin;
	WaterSensor::PH_address = PH_address;
	WaterSensor::DO_address = DO_address;
	WaterSensor::ds = new OneWire(DS18S20_Pin); 
	WaterSensor::dht = new DHT(DHT_Pin); 

}

void WaterSensor::wakeUpPHSensor()
{
	wakeUpSensor(WaterSensor::PH_address);
}

void WaterSensor::wakeUpDOSensor()
{
	wakeUpSensor(WaterSensor::DO_address);
}


void WaterSensor::wakeUpSensor(int address)
{
	Wire.beginTransmission(address); //call the circuit by its ID number.
    Wire.write('r');        //transmit the command that was sent through the serial port.
    Wire.endTransmission();          //end the I2C data transmission.
    delay(200);                    //wait the correct amount of time for the circuit to complete its instruction.
    Wire.beginTransmission(address); //call the circuit by its ID number.
    Wire.write('r');        //transmit the command that was sent through the serial port.
    Wire.endTransmission();          //end the I2C data transmission.
    delay(200);                    //wait the correct amount of time for the circuit to complete its instruction.
    Wire.beginTransmission(address); //call the circuit by its ID number.
    Wire.write('r');        //transmit the command that was sent through the serial port.
    Wire.endTransmission();          //end the I2C data transmission.
}

void WaterSensor::sleepPHSensor()
{
	sleepSensor(WaterSensor::PH_address);
}

void WaterSensor::sleepDOSensor()
{
	sleepSensor(WaterSensor::DO_address);
}

void WaterSensor::sleepSensor(int address)
{
	Wire.beginTransmission(address); //call the circuit by its ID number.
    Wire.write("SLEEP");        //transmit the command that was sent through the serial port.
    Wire.endTransmission();          //end the I2C data transmission.
    delay(100);                    //wait the correct amount of time for the circuit to complete its instruction.
}

void WaterSensor::sendTempToPHSensor(float temp)
{
	sendTempToSensor(WaterSensor::PH_address,temp);
}

void WaterSensor::sendTempToDOSensor(float temp)
{
	sendTempToSensor(WaterSensor::DO_address,temp);
}

void WaterSensor::sendTempToSensor(int address, float temp)
{
	if (temp < -20 || temp > 70) {
		return;
	}

	String tempString = String(temp);
	tempString = "T," + tempString;

	Wire.beginTransmission(address); //call the circuit by its ID number.

	char charBuf[10];
	tempString.toCharArray(charBuf,20);

	Wire.write(charBuf);        //transmit the command that was sent through the serial port.

	Wire.endTransmission();          //end the I2C data transmission.
	delay(100);                    //wait the correct amount of time for the circuit to complete its instruction.
}

void WaterSensor::getPHSensorValue(char* data)
{
	byte flag = 0;               
	byte i = 0; 
	getSensorValue(WaterSensor::PH_address,data);
	if (strlen(data) < 2){
		strcpy(data,"NR");
	}

	for (i = 0; i < 20; i++) {          //Set the precision of the value 2 digit after the dot
		if (data[i] == '.') {          
		  flag = i;                    
		}
		if (flag != 0 && flag+3 == i){                    
			data[i] = '\0';
			break;
		}
	}
}

void WaterSensor::getDOSensorValue(char* data)
{
	
	
	byte flag = 0;               
	byte i = 0; 			
	char* DO,sat;
	
	getSensorValue(WaterSensor::DO_address,data);
	
	if (strlen(data) < 2){
		strcpy(data,"NR");
	}
	
	if (isDigit(data[0])) {
		for (i = 0; i < 20; i++) {          //Step through each char
			if (data[i] == ',') {          //do we see a ','
			  flag = 1;                       //if so we set the var flag to 1 by doing this we can identify if the string being sent from the DO circuit is a CSV string containing tow values
			}
		}

		if (flag ==1){                    //if we see the there was a â€˜,â€™ in the string array
			data = strtok(data, ",");        //let's pars the string at each comma
		}
	}
}

void WaterSensor::getSensorValue(int address, char* data)
{
	byte in_char = 0;                //used as a 1 byte buffer to store in bound bytes from the pH Circuit.
    byte i = 0;

    Wire.beginTransmission(address); //call the circuit by its ID number.

    Wire.write('r');        //transmit the command that was sent through the serial port.

    Wire.endTransmission();          //end the I2C data transmission.

    delay(1800);                    //wait the correct amount of time for the circuit to complete its instruction.

    Wire.requestFrom(address, 20, 1); //call the circuit and request 20 bytes (this may be more than we need)
    Wire.read();             //the first byte is the response code, we read this separately.
    
    while (Wire.available()) {         //are there bytes to receive.
      in_char = Wire.read();           //receive a byte.
      data[i] = in_char;            //load this byte into our array.
      i += 1;                          //incur the counter for the array element.
      if (in_char == 0) {              //if we see that we have been sent a null command.
		data[i] = '\0';
        i = 0;                         //reset the counter i to 0.
        Wire.endTransmission();        //end the I2C data transmission.
        break;                         //exit the while loop.
      }
    }
}

float WaterSensor::getTemperatureValue()
{

	byte data[12];
	byte addr[8];

	if ( !ds->search(addr)) {
		//no more sensors on chain, reset search
		ds->reset_search();
		return -1000;
	}

	if ( OneWire::crc8( addr, 7) != addr[7]) {
		Serial.println("CRC is not valid!");
		return -1000;
	}

	if ( addr[0] != 0x10 && addr[0] != 0x28) {
		Serial.print("Device is not recognized");
		return -1000;
	}

	ds->reset();
	ds->select(addr);
	ds->write(0x44,1); // start conversion, with parasite power on at the end

	delay(750); // Wait for temperature conversion to complete

	byte present = ds->reset();
	ds->select(addr);    
	ds->write(0xBE); // Read Scratchpad


	for (int i = 0; i < 9; i++) { // we need 9 bytes
		data[i] = ds->read();
	}

	ds->reset_search();

	byte MSB = data[1];
	byte LSB = data[0];

	float tempRead = ((MSB << 8) | LSB); //using two's compliment
	float TemperatureSum = tempRead / 16;

	return TemperatureSum;
  
}


void WaterSensor::getInboxTemperatureHumidityValue(float* temperature, float* humidity)
{
	if (dht->readDHT22(temperature, humidity) != 0){
		*temperature = -1000;
		*humidity = -1000;
	}
}
