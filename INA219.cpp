/******************************************************************************
* TI INA219 hi-side i2c current/power monitor Library
*
* http://www.ti.com/product/ina219
*
* 6 May 2012 by John De Cristofaro
*
*
* Tested at standard i2c 100kbps signaling rate.
*
* This library does not handle triggered conversion modes. It uses the INA219
* in continuous conversion mode. All reads are from continous conversions.
*
* A note about the gain (PGA) setting:
*	The gain of the ADC pre-amplifier is programmable in the INA219, and can
*	be set between 1/8x (default) and unity. This allows a shunt voltage 
*	range of +/-320mV to +/-40mV respectively. Something to keep in mind,
*	however, is that this change in gain DOES NOT affect the resolution
*	of the ADC, which is fixed at 1uV. What it does do is increase noise
*	immunity by exploiting the integrative nature of the delta-sigma ADC.
*	For the best possible reading, you should set the gain to the range
*	of voltages that you expect to see in your particular circuit. See
*	page 15 in the datasheet for more info about the PGA.
*
* Known bugs:
*     * may return unreliable values if not connected to a bus or at
*	bus currents below 10uA.
*
* Arduino 1.0 compatible as of 6/6/2012
*
* Dependencies:
*    * Arduino Wire library
*
* MIT license
******************************************************************************/

#include "INA219.h"
#include <util/delay.h>
namespace{
// config. register bit labels
const uint8_t RST =	15;
const uint8_t BRNG = 13;
const uint8_t PG1 = 12;
const uint8_t PG0 = 11;
const uint8_t BADC4 = 10;
const uint8_t BADC3	= 9;
const uint8_t BADC2	= 8;
const uint8_t BADC1	= 7;
const uint8_t SADC4	= 6;
const uint8_t SADC3	= 5;
const uint8_t SADC2	= 4;
const uint8_t SADC1	= 3;
const uint8_t MODE3	= 2;
const uint8_t MODE2	= 1;
const uint8_t MODE1	= 0;
};

INA219::INA219(t_i2caddr addr): i2c_address(addr) {
}

void INA219::begin() {
    Wire.begin();
}

void INA219::calibrate(float shunt_val, float v_shunt_max, float v_bus_max, float i_max_expected) {
    uint16_t cal,digits;
    float i_max_possible, min_lsb, max_lsb, swap;

#if (INA219_DEBUG == 1)
     float max_current,max_before_overflow,max_shunt_v,max_shunt_v_before_overflow,max_power;
#endif
    r_shunt = shunt_val;

    i_max_possible = v_shunt_max / r_shunt;
    min_lsb = i_max_expected / 32767;
    max_lsb = i_max_expected / 4096;

    current_lsb = min_lsb;
    digits=0;

    /* From datasheet: This value was selected to be a round number near the Minimum_LSB. This selection allows for good resolution with a rounded LSB.
	 * eg. 0.000610 -> 0.000700
	*/
    while( current_lsb > 0.0 ){//If zero there is something weird...
        if( (uint16_t)current_lsb / 1){
        	current_lsb = (uint16_t) current_lsb + 1;
        	current_lsb /= pow(10,digits);
        	break;
        }
        else{
        	digits++;
            current_lsb *= 10.0;
        }
    };

    swap = (0.04096)/(current_lsb*r_shunt);
    cal = (uint16_t)swap;
    power_lsb = current_lsb * 20;

#if (INA219_DEBUG == 1)
      max_current = current_lsb*32767;
      max_before_overflow =  max_current > i_max_possible?i_max_possible:max_current;

      max_shunt_v = max_before_overflow*r_shunt;
      max_shunt_v_before_overflow = max_shunt_v > v_shunt_max?v_shunt_max:max_shunt_v;

      max_power = v_bus_max * max_before_overflow;
      Serial.print("v_bus_max:     "); Serial.println(v_bus_max, 8);
      Serial.print("v_shunt_max:   "); Serial.println(v_shunt_max, 8);
      Serial.print("i_max_possible:        "); Serial.println(i_max_possible, 8);
      Serial.print("i_max_expected: "); Serial.println(i_max_expected, 8);
      Serial.print("min_lsb:       "); Serial.println(min_lsb, 12);
      Serial.print("max_lsb:       "); Serial.println(max_lsb, 12);
      Serial.print("current_lsb:   "); Serial.println(current_lsb, 12);
      Serial.print("power_lsb:     "); Serial.println(power_lsb, 8);
      Serial.println("  ");
      Serial.print("cal:           "); Serial.println(cal);
      Serial.print("r_shunt:       "); Serial.println(r_shunt);
      Serial.print("max_before_overflow:       "); Serial.println(max_before_overflow,8);
      Serial.print("max_shunt_v_before_overflow:       "); Serial.println(max_shunt_v_before_overflow,8);
      Serial.print("max_power:       "); Serial.println(max_power,8);

#endif
      write16(CAL_R, cal);

}

void INA219::configure(  t_range range,  t_gain gain,  t_adc  bus_adc,  t_adc shunt_adc,  t_mode mode) {
  config = 0;

  config |= (range << BRNG | gain << PG0 | bus_adc << BADC1 | shunt_adc << SADC1 | mode);

  write16(CONFIG_R, config);
}

#define INA_RESET        0xFFFF    // send to CONFIG_R to reset unit
void INA219::reset(){
  write16(CONFIG_R, INA_RESET);
  _delay_ms(5);
}

int16_t INA219::shuntVoltageRaw() const {
  return read16(V_SHUNT_R);
}

float INA219::shuntVoltage() const {
  float temp;
  temp = read16(V_SHUNT_R);
  return (temp / 100000);
}

int16_t INA219::busVoltageRaw() const {
  return read16(V_BUS_R);
}


float INA219::busVoltage() const {
  int16_t temp;
  temp = read16(V_BUS_R);
  temp >>= 3;
  return (temp * 0.004);
}

float INA219::shuntCurrent() const {
  return (read16(I_SHUNT_R) * current_lsb);
}

float INA219::busPower() const {
  return (read16(P_BUS_R) * power_lsb);
}


/**********************************************************************
*             INTERNAL I2C FUNCTIONS                  *
**********************************************************************/

void INA219::write16(t_reg a, uint16_t d) const {
  uint8_t temp;
  temp = (uint8_t)d;
  d >>= 8;
  Wire.beginTransmission(i2c_address); // start transmission to device

  #if ARDUINO >= 100
    Wire.write(a); // sends register address to read from
    Wire.write((uint8_t)d);  // write data hibyte 
    Wire.write(temp); // write data lobyte;
  #else
    Wire.send(a); // sends register address to read from
    Wire.send((uint8_t)d);  // write data hibyte 
    Wire.send(temp); // write data lobyte;
  #endif

  Wire.endTransmission(); // end transmission
  delay(1);
}

int16_t INA219::read16(t_reg a) const {
  uint16_t ret;

  // move the pointer to reg. of interest, null argument
  write16(a, 0);
  
  Wire.requestFrom((int)i2c_address, 2);    // request 2 data bytes

  #if ARDUINO >= 100
    ret = Wire.read(); // rx hi byte
    ret <<= 8;
    ret |= Wire.read(); // rx lo byte
  #else
    ret = Wire.receive(); // rx hi byte
    ret <<= 8;
    ret |= Wire.receive(); // rx lo byte
  #endif

  Wire.endTransmission(); // end transmission

  return ret;
}
