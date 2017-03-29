
#include <SPI.h>  
#include "SX1272.h"
#include <WaterSensor.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <EEPROM.h>

#define address_ph 99         // default PH sensor adress on I2C port
#define address_DO 97         // default DO sensor adress on I2C port
#define DS18S20_Pin 2         // digital pin of the temperature sensor (outside the box)
#define DHT_PIN 3             // digital pin of the humidity and temperature sensor (inside the box)

#define TemperatureSensorsPowerPin 8
#define EzoDOSensorPowerPin 7
#define EzoPHSensorPowerPin 6
#define VoltagePin  A2


// LoRa module configuration
#define DEFAULT_DEST_ADDR 1   
#define LORAMODE  1
#define node_addr 6

// Define the sleeping time period of the device 
#define SLEEP_LOOP 100        // one loop = 8 sec, But the process take a little more time so count about 9 sec for each loop
                              // Example sleep_loop = 4 -> 4*9 = 36 sec . About 400 for one hour.

#define RESISTOR1 47000.0        // RESISTOR to calculate voltage
#define RESISTOR2 10000.0

#define ADDR_EEPROM_REBOOT 0   // If the Arduino havent reboot since last sleep loop because of a failure, ADDR_EEPROM_REBOOT is set to 0
                               // otherwise if there was a failure since last sleep loop ADDR_EEPROM_REBOOT is set to 0
                               // After a long  sleep period reset ADDR_EEPROM_REBOOT must be set to 1

volatile int nbr_remaining = 0;   
float temperatureBox, humidityBox, temperature, batteryVoltage;
char dataPH[10] , dataDO[10];


WaterSensor waterSensor(DS18S20_Pin,DHT_PIN, address_ph,address_DO);

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(38400);
  
  Wire.begin();           //enable I2C port.

  pinMode(TemperatureSensorsPowerPin,OUTPUT);    // Switch on power for temp and humdity sensor
  pinMode(EzoDOSensorPowerPin,OUTPUT);          // Switch on power for DO sensor
  pinMode(EzoPHSensorPowerPin,OUTPUT);          // Switch on power for PH sensor


  configure_wdt();        // configure the watchdogtimer to prevent any failure


  // Power ON the LoRa module
  sx1272.ON();

  // Set transmission mode and print the result
  sx1272.setMode(LORAMODE);

  sx1272._enableCarrierSense=true;

  sx1272.setPower('M');

    
  // Select frequency channel
  sx1272.setChannel(CH_10_868);

  sx1272.setCRC_ON();

  // Set the node address
  sx1272.setNodeAddress(node_addr);

}


void loop(void)
{
  Serial.println(F("Loop Start"));
  uint8_t r_size;

  char buffDataToSend[100];

  sx1272.CarrierSense();
  sx1272.setPacketType(PKT_TYPE_DATA);
  
  while (1) {

      Serial.println(F("System awake"));
      delay(200);
      pinMode(TemperatureSensorsPowerPin,OUTPUT);    // Switch on power for temp and humdity sensor
      pinMode(EzoDOSensorPowerPin,OUTPUT);          // Switch on power for DO sensor
      pinMode(EzoPHSensorPowerPin,OUTPUT);          // Switch on power for PH sensor



      digitalWrite(TemperatureSensorsPowerPin,HIGH);
      digitalWrite(EzoPHSensorPowerPin,HIGH);
      digitalWrite(EzoDOSensorPowerPin,HIGH);
      
      
      dataPH[0] = '\0';
      dataDO[0] = '\0';

      delay(200);


      // Wake up DO and PH sensors 
      waterSensor.wakeUpPHSensor();
      wdt_reset();
      waterSensor.wakeUpDOSensor();
      wdt_reset();




      // Receive temperature from DS18S20 sensor
      temperature = waterSensor.getTemperatureValue(); //will take about 750mss to run
      Serial.print(F("Temp in water : "));
      Serial.println(temperature);
      delay(1000);

      wdt_reset();

      delay(1000);

      wdt_reset();
      waterSensor.sendTempToPHSensor(temperature);     // Send the temperature to PH sensor to precise the reading of the value
      waterSensor.getPHSensorValue(dataPH);            // Receive the PH value
      Serial.print(F("PH : "));
      Serial.println(dataPH);

      delay(1000);

      wdt_reset();
      waterSensor.sendTempToDOSensor(temperature);   // Send the temperature to DO sensor to precise the reading value
      waterSensor.getDOSensorValue(dataDO);          // Receive the DO value
      Serial.print(F("DO : "));
      Serial.println(dataDO);

      waterSensor.getInboxTemperatureHumidityValue(&temperatureBox,&humidityBox);
      Serial.print(F("Temp in box : "));
      Serial.println(temperatureBox);
      Serial.print(F("Humidity : "));
      Serial.println(humidityBox);
      
      delay(1000);
  
      waterSensor.sleepPHSensor();             // Put the PH sensor in sleep mode to save battery power
      waterSensor.sleepDOSensor();             // Put the DO sensor in sleep mode to save battery power


      delay(200);
      wdt_reset();

      batteryVoltage = getBatteryVoltage();

      Serial.print(F("Battery voltage : "));
      Serial.println(batteryVoltage);

      
      int failure = createMessage(buffDataToSend);
      Serial.println(F("Message : "));
      Serial.println(buffDataToSend);
      
      int e = sx1272.sendPacketTimeout(DEFAULT_DEST_ADDR, buffDataToSend);
      wdt_reset();

      Serial.print(F("Packet sent, state "));
      Serial.println(e);

      pinMode(EzoPHSensorPowerPin,INPUT);
      pinMode(EzoDOSensorPowerPin,INPUT);
      pinMode(TemperatureSensorsPowerPin,INPUT);

      Serial.println(F("System sleep"));
      Serial.flush();
      delay(200);
      sleep(SLEEP_LOOP);

    }          
}

int createMessage(char* buff){

      int returnCode = 0;
  
      char dataTemp[9] , dataHumBox[9], dataTempBox[9], dataBatteryVoltage[9];
      char str[20];

      if (temperatureBox > 65 || temperatureBox < -40){
          strcpy(dataTempBox,"NR");
      } else {
          dtostrf(temperatureBox,2,2,dataTempBox);
      }

      if (humidityBox > 100 || humidityBox < 0){
          strcpy(dataHumBox,"NR");
      } else {
          dtostrf(humidityBox,2,2,dataHumBox);
      }

      if (temperature > 70 || temperature < -40){
          strcpy(dataTemp,"NR");
      } else {
          dtostrf(temperature,2,2,dataTemp);
      }

      dtostrf(batteryVoltage,2,2,dataBatteryVoltage);
      
      strcpy (str,dataTemp);
      sprintf(buff,"\\!WT/C/%s#",str);
   
      strcpy (str,dataPH);
      sprintf(buff,"%sPH/NA/%s#",buff,str);
      if (strcmp(str,"NR")==0){
        returnCode = 1;
      }
      
      strcpy (str,dataDO);
      sprintf(buff,"%sDO/mgL/%s#",buff,str);
      if (strcmp(str,"NR")==0){
        returnCode = 1;
      }
      
      strcpy (str,dataTempBox);
      sprintf(buff,"%sT/C/%s#",buff,str);
      
      strcpy (str,dataHumBox);
      sprintf(buff,"%sHUM/per/%s#",buff,str);

      strcpy (str,dataBatteryVoltage);
      sprintf(buff,"%sBAT/V/%s#",buff,str);

      return returnCode;
}


// interrupt raised by the watchdog firing
ISR(WDT_vect)
{
    if(nbr_remaining > 0)
    {
        nbr_remaining = nbr_remaining - 1;
        wdt_reset();
    }
    else
    {
        if (EEPROM.read(ADDR_EEPROM_REBOOT) == 0){
          EEPROM.write(ADDR_EEPROM_REBOOT, 1);
          Serial.println(F("System failure"));
          Serial.println(F("System reboots"));
          Serial.flush();
          MCUSR = 0;                          // reset flags
         
          WDTCSR |= 0b00011000;               // Enter config mode.
          WDTCSR =  0b00001000 | 0b000000;    // clr WDIE (interrupt enable...7th from left)
                                              // set WDE (reset enable...4th from left), and set delay interval
                                              // reset system in 16 ms...
                                              // unless wdt_disable() in loop() is reached first
                                         
          while(1);          
        } else {
              Serial.println(F("System failure"));
              Serial.println(F("Sleep and reboot"));
              Serial.flush();
              delay(200);
               configure_wdt();
              sleep(SLEEP_LOOP);              
              Serial.flush();
              MCUSR = 0;                          // reset flags
           
              WDTCSR |= 0b00011000;               // Enter config mode.
              WDTCSR =  0b00001000 | 0b000000;    // clr WDIE (interrupt enable...7th from left)
                                                // set WDE (reset enable...4th from left), and set delay interval
                                                // reset system in 16 ms...
                                                // unless wdt_disable() in loop() is reached first                           
              while(1);    
          
        }

    }
}

void configure_wdt(void)
{
 
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags

                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds

  sei();                           // re-enable interrupts 
}

void sleep(int ncycles)
{ 

  nbr_remaining = ncycles;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
  power_all_disable();
  wdt_reset();

  // While it remains sleep loop we stay in sleep mode
  while (nbr_remaining > 0){ 
    sleep_mode();                 // Activate sleep mode
    sleep_disable();              // Disable wake after watchdog is triggered
  }
 
  power_all_enable();

  EEPROM.write(ADDR_EEPROM_REBOOT, 0);
}

float getBatteryVoltage()
{
   int rawVin;
   int sumRawVin = 0;
   analogReference(INTERNAL);
  /* Mesure la tension en VIN et la référence interne à 1.1 volts */
  for (byte y=0; y<30; y++){
    rawVin = analogRead(VoltagePin);
    sumRawVin=sumRawVin+rawVin;
    delay(5);
  }
  rawVin = sumRawVin/30;
  float real_v = (rawVin * 1.1 / 1024.0) / (RESISTOR2/(RESISTOR1+RESISTOR2));
  if (real_v < 0.1) {
     real_v=0.0;
  }
  
  return real_v;
  
}


int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
