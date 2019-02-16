/**********************************************
* INA219 library example
* 9 January 2016 by Flavius Bindea
*
* this code is public domain.
**********************************************/


#include <Wire.h>
#include <INA219.h>

#define MAX_CURRENT 20    /* In our case this is enough even shunt is capable to 50 A*/
#define SHUNT_R   0.00125   /* Shunt resistor in ohm */

INA219 monitor(&Wire, 0x41);
bool beginResult;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  beginResult = monitor.begin();
  // setting up our configuration
  // default values are RANGE_32V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT, CONT_SH_BUS
  monitor.configure(INA219::RANGE_16V, INA219::GAIN_2_80MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);
  
  // calibrate with our values
  monitor.calibrate(SHUNT_R, MAX_CURRENT);
}

void loop()
{
  Serial.println("******************");

  Serial.print("Monitor begin: ");
  Serial.println(beginResult ? "true" : "false");
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
