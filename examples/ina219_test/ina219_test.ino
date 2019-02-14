/**********************************************
* INA219 library example
* 10 May 2012 by johngineer
*
* 9 January 2016 Flavius Bindea: changed default values and begin()
*
* this code is public domain.
**********************************************/


#include <Wire.h>
#include <INA219.h>

INA219 monitor;


void setup()
{
  Serial.begin(115200);
  bool result = monitor.begin();
  return;
  Serial.print("Monitor begin: ");
  Serial.println(result ? "true" : "false");
  // begin calls:
  // configure() with default values RANGE_32V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT, CONT_SH_BUS
  // calibrate() with default values D_SHUNT=0.1, D_V_BUS_MAX=32, D_V_SHUNT_MAX=0.2, D_I_MAX_EXPECTED=2
  // in order to work directly with ADAFruit's INA219B breakout
}

void loop()
{
  Serial.println("******************");
  
  Serial.print("raw shunt voltage: ");
  int16_t shuntVoltageRaw;
  if (monitor.shuntVoltageRaw(&shuntVoltageRaw)) {
    Serial.println(shuntVoltageRaw);
  } else {
    Serial.println("failed");
  }
  
  Serial.print("raw bus voltage:   ");
  int16_t busVoltageRaw;
  if (monitor.busVoltageRaw(&busVoltageRaw)) {
    Serial.println(busVoltageRaw);
  } else {
    Serial.println("failed");
  }
  
  Serial.println("--");
  
  Serial.print("shunt voltage: ");
  float shuntVoltage;
  if (monitor.shuntVoltage(&shuntVoltage)) {
    Serial.print(shuntVoltage * 1000, 4);
    Serial.println(" mV");
  } else {
    Serial.println("failed");
  }
  
  Serial.print("shunt current: ");
  float shuntCurrent;
  if (monitor.shuntCurrent(&shuntCurrent)) {
    Serial.print(shuntCurrent * 1000, 4);
    Serial.println(" mA");
  } else {
    Serial.println("failed");
  }
  
  Serial.print("bus voltage:   ");
  float busVoltage;
  if (monitor.busVoltage(&busVoltage)) {
    Serial.print(busVoltage, 4);
    Serial.println(" V");
  } else {
    Serial.println("failed");
  }
  
  Serial.print("bus power:     ");
  float busPower;
  if (monitor.busPower(&busPower)) {
    Serial.print(busPower * 1000, 4);
    Serial.println(" mW");
  } else {
    Serial.println("failed");
  }
  
  Serial.println(" ");
  Serial.println(" ");
  
  delay(1000);

}


