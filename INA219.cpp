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
* The gain of the ADC pre-amplifier is programmable in the INA219, and can
* be set between 1/8x (default) and unity. This allows a shunt voltage 
* range of +/-320mV to +/-40mV respectively. Something to keep in mind,
* however, is that this change in gain DOES NOT affect the resolution
* of the ADC, which is fixed at 1uV. What it does do is increase noise
* immunity by exploiting the integrative nature of the delta-sigma ADC.
* For the best possible reading, you should set the gain to the range
* of voltages that you expect to see in your particular circuit. See
* page 15 in the datasheet for more info about the PGA.
*
* Known bugs:
*     * may return unreliable values if not connected to a bus or at
* bus currents below 10uA.
*
* Arduino 1.0 compatible as of 6/6/2012
*
* Dependencies:
*    * Arduino Wire library
*
* MIT license
******************************************************************************/

#include "INA219.h"

#ifdef __avr__
  #include <util/delay.h>
#endif

#define INA219_DEBUG 0

namespace{
// config. register bit labels
const uint8_t RST   = 15;
const uint8_t BRNG  = 13;
const uint8_t PG1   = 12;
const uint8_t PG0   = 11;
const uint8_t BADC4 = 10;
const uint8_t BADC3 = 9;
const uint8_t BADC2 = 8;
const uint8_t BADC1 = 7;
const uint8_t SADC4 = 6;
const uint8_t SADC3 = 5;
const uint8_t SADC2 = 4;
const uint8_t SADC1 = 3;
const uint8_t MODE3 = 2;
const uint8_t MODE2 = 1;
const uint8_t MODE1 = 0;
};

#define CNVR_B 1  // conversion ready bit in bus voltage register V_BUS_R 
#define OVF_B  0  // math overflow bit in bus voltage register V_BUS_R 
#define INA_RESET        0xFFFF    // send to CONFIG_R to reset unit

INA219::INA219(uint8_t addr) : INA219(&Wire, addr) {
}

INA219::INA219(TwoWire *i2c, uint8_t addr): _i2c(i2c), _i2c_address(addr), _r_shunt(0), _current_lsb(0), _power_lsb(0), _config(0), _cal(0), _ready(false), _overflow(false) {
}

bool INA219::begin() {
    return configure() && calibrate();
}

bool INA219::calibrate(float shunt_val, float i_max_expected) {
    uint16_t digits;
    float min_lsb, swap;

    _r_shunt = shunt_val;

    min_lsb = i_max_expected / 32767;

    _current_lsb = min_lsb;
    digits=0;

    /* From datasheet: This value was selected to be a round number near the Minimum_LSB.
     * This selection allows for good resolution with a rounded LSB.
     * eg. 0.000610 -> 0.000700
     */
    while( _current_lsb > 0.0 ){//If zero there is something weird...
        if( (uint16_t)_current_lsb / 1){
            _current_lsb = (uint16_t) _current_lsb + 1;
            _current_lsb /= pow(10,digits);
            break;
        }
        else{
            digits++;
            _current_lsb *= 10.0;
        }
    };

    swap = (0.04096)/(_current_lsb*_r_shunt);
    _cal = (uint16_t)swap;
    _power_lsb = _current_lsb * 20;

#if (INA219_DEBUG == 1)
      float max_current,max_lsb;
      max_lsb = i_max_expected / 4096;
      max_current = _current_lsb*32767;

      Serial.print("max_current:        "); Serial.println(max_current, 8);
      Serial.print("i_max_expected: "); Serial.println(i_max_expected, 8);
      Serial.print("min_lsb:       "); Serial.println(min_lsb, 12);
      Serial.print("max_lsb:       "); Serial.println(max_lsb, 12);
      Serial.print("_current_lsb:   "); Serial.println(_current_lsb, 12);
      Serial.print("_power_lsb:     "); Serial.println(_power_lsb, 8);
      Serial.println("  ");
      Serial.print("_cal:           "); Serial.println(_cal);
      Serial.print("_r_shunt:       "); Serial.println(_r_shunt, 6);
      Serial.println("  ");
#endif
      return write16(CAL_R, _cal);
}

bool INA219::configure(  t_range range,  t_gain gain,  t_adc  bus_adc,  t_adc shunt_adc,  t_mode mode) {
  _config = (range << BRNG | gain << PG0 | bus_adc << BADC1 | shunt_adc << SADC1 | mode);
#if (INA219_DEBUG == 1)
  Serial.print("Config: 0x"); Serial.println(_config,HEX);
#endif
  return write16(CONFIG_R, _config);
}

bool INA219::reset() {
  bool result = write16(CONFIG_R, INA_RESET);
  delay(5);
  return result;
}

bool INA219::shuntVoltageRaw(int16_t *value) const {
  return read16(V_SHUNT_R, value);
}

bool INA219::shuntVoltage(float *voltage) const {
  int16_t value;
  bool result = shuntVoltageRaw(&value);
  if (voltage) {
    *voltage = ((float)value / 100000);
  }
  return result;
}

bool INA219::busVoltageRaw(int16_t *value) {
  int16_t busVoltageRegister;
  bool result = read16(V_BUS_R, &busVoltageRegister);
  _overflow = bitRead(busVoltageRegister, OVF_B);     // overflow bit
  _ready    = bitRead(busVoltageRegister, CNVR_B);    // ready bit
  if (value) {
    *value = busVoltageRegister;
  }
  return result;
}

bool INA219::busVoltage(float *voltage) {
  int16_t value;
  bool result = busVoltageRaw(&value);
  if (voltage) {
    value >>= 3;
    *voltage = ((float)value * 0.004);
  }
  return result;
}

bool INA219::shuntCurrentRaw(int16_t *current) const {
  int16_t value;
  bool result = read16(I_SHUNT_R, &value);
  if (current) {
    *current = value;
  }
  return result;
}

bool INA219::shuntCurrent(float *current) const {
  int16_t value;
  bool result = shuntCurrentRaw(&value);
  if (current) {
    *current = (float)value * _current_lsb;
  }
  return result;
}

bool INA219::busPower(float *voltage) const {
  int16_t value;
  bool result = read16(P_BUS_R, &value);
  if (voltage) {
    *voltage = (float)value * _power_lsb;
  }
  return result;
}

/**************************************************************************/
/*! 
    @brief  Rewrites the last config register
*/
/**************************************************************************/
bool INA219::reconfig() const {
#if (INA219_DEBUG == 1)
  Serial.print("Reconfigure with Config: 0x"); Serial.println(_config,HEX);
#endif
  return write16(CONFIG_R, _config);
}

/**************************************************************************/
/*! 
    @brief  Rewrites the last calibration
*/
/**************************************************************************/
bool INA219::recalibrate() const {
#if (INA219_DEBUG == 1)
  Serial.print("Recalibrate with cal: "); Serial.println(_cal);
#endif
  return write16(CAL_R, _cal);
}

/**************************************************************************/
/*! 
    @brief  returns conversion ready bite from last bus voltage read
    
    @note page 30:
          Although the data from the last conversion can be read at any time,
          the INA219 Conversion Ready bit (CNVR) indicates when data from
          a conversion is available in the data output registers.
          The CNVR bit is set after all conversions, averaging, 
          and multiplications are complete.
          CNVR will clear under the following conditions:
          1.) Writing a new mode into the Operating Mode bits in the 
              Configuration Register (except for Power-Down or Disable)
          2.) Reading the Power Register
          
          page 15:
          The Conversion Ready bit clears under these
          conditions:
          1. Writing to the Configuration Register, except
          when configuring the MODE bits for Power Down
          or ADC off (Disable) modes;
          2. Reading the Status Register;
          3. Triggering a single-shot conversion with the
          Convert pin.
*/
/**************************************************************************/
bool INA219::ready() const {
  return _ready;
}

/**************************************************************************/
/*! 
    @brief  returns overflow bite from last bus voltage read
    
    @note The Math Overflow Flag (OVF) is set when the Power or
          Current calculations are out of range. It indicates that
          current and power data may be meaningless.
*/
/**************************************************************************/
bool INA219::overflow() const {
  return _overflow;
}

/**********************************************************************
*             INTERNAL I2C FUNCTIONS                  *
**********************************************************************/

bool INA219::write16(t_reg a, uint16_t d) const {
  uint8_t temp;
  temp = (uint8_t)d;
  d >>= 8;
  _i2c->beginTransmission(_i2c_address); // start transmission to device
  _i2c->write(a); // sends register address to read from
  _i2c->write((uint8_t)d);  // write data hibyte
  _i2c->write(temp); // write data lobyte;
  bool result = _i2c->endTransmission() == 0; // end transmission
  delay(1);
  return result;
}

bool INA219::read16(t_reg a, int16_t *value) const {
  // move the pointer to reg. of interest, null argument
  if (!write16(a, 0)) {
    return false;
  }
  
  _i2c->requestFrom(_i2c_address, 2);    // request 2 data bytes
  bool result = false;
  if (_i2c->available() == 2) {
    uint16_t ret = 0;
    result = true;
    ret = _i2c->read(); // rx hi byte
    ret <<= 8;
    ret |= _i2c->read(); // rx lo byte
    if (value) {
      *value = ret;
    }
  }
  while (_i2c->available() > 0) {
    _i2c->read();
  }
  return result;
}
