#include <WaterSensor.h>

#define address_PH 99         // default PH sensor adress on I2C port
#define address_DO 97         // default DO sensor adress on I2C port
#define DS18S20_Pin 2         // digital pin of the temperature sensor (outside the box)
#define DHT_PIN 3             // digital pin of the humidity and temperature sensor (inside the box)

#define TemperatureSensorsPowerPin 8
#define EzoDOSensorPowerPin 6
#define EzoPHSensorPowerPin 7

float temperatureBox, humidityBox, temperature;
char dataPH[10] , dataDO[10];

WaterSensor waterSensor(DS18S20_Pin,DHT_PIN, address_PH,address_DO);

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(38400);

  Wire.begin();           //enable I2C port.

  pinMode(TemperatureSensorsPowerPin,OUTPUT);    // Switch on power for temp and humdity sensor
  pinMode(EzoDOSensorPowerPin,OUTPUT);          // Switch on power for DO sensor
  pinMode(EzoPHSensorPowerPin,OUTPUT);          // Switch on power for PH sensor

  digitalWrite(TemperatureSensorsPowerPin,HIGH);
  digitalWrite(EzoPHSensorPowerPin,HIGH);
  digitalWrite(EzoDOSensorPowerPin,HIGH);

}

void loop(void)
{
  dataPH[0] = '\0';
  dataDO[0] = '\0';

  delay(200);

  // Wake up DO and PH sensors 
  waterSensor.wakeUpPHSensor();
  waterSensor.wakeUpDOSensor();

  temperature = waterSensor.getTemperatureValue();
  Serial.print(F("Temp in water : "));
  Serial.println(temperature);

  delay(1000);

  waterSensor.getInboxTemperatureHumidityValue(&temperatureBox,&humidityBox);
  Serial.print(F("Temp in box : "));
  Serial.println(temperatureBox);
  Serial.print(F("Humidity : "));
  Serial.println(humidityBox);

  delay(1000);

  waterSensor.sendTempToDOSensor(temperature);   // Send the temperature to DO sensor to precise the reading value
  waterSensor.getDOSensorValue(dataDO);          // Receive the DO value
  Serial.print(F("DO : "));
  Serial.println(dataDO);

  delay(1000);

  waterSensor.sendTempToPHSensor(temperature);     // Send the temperature to PH sensor to precise the reading of the value
  waterSensor.getPHSensorValue(dataPH);            // Receive the PH value
  Serial.print(F("PH : "));
  Serial.println(dataPH);

  delay(1000);

  waterSensor.sleepPHSensor();             // Put the PH sensor in sleep mode to save battery power
  waterSensor.sleepDOSensor();             // Put the DO sensor in sleep mode to save battery power
  delay(4000);
}
