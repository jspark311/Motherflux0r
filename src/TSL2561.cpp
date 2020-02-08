/*
* This file started out as an adafruit driver. I have mutated it.
*   ---J. Ian Lindsay
*/

/*!
 * @file Adafruit_TSL2561_U.cpp
 *
 * @mainpage Adafruit TSL2561 Light/Lux sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's TSL2561 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit TSL2561 breakout: http://www.adafruit.com/products/439
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit_Sensor">
 * Adafruit_Sensor</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 *   @section  HISTORY
 *
 *   v2.0 - Rewrote driver for Adafruit_Sensor and Auto-Gain support, and
 *          added lux clipping check (returns 0 lux on sensor saturation)
 *   v1.0 - First release (previously TSL2561)
*/
/**************************************************************************/

#include "TSL2561.h"

/*******************************************************************************
* Internal constants
*******************************************************************************/
#define TSL2561_LUX_LUXSCALE      (14)      ///< Scale by 2^14
#define TSL2561_LUX_RATIOSCALE    (9)       ///< Scale ratio by 2^9
#define TSL2561_LUX_CHSCALE       (10)      ///< Scale channel values by 2^10
#define TSL2561_LUX_CHSCALE_TINT0 (0x7517)  ///< 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_LUX_CHSCALE_TINT1 (0x0FE7)  ///< 322/81 * 2^TSL2561_LUX_CHSCALE

#define TSL2561_CLEAR_BIT         (0x40)    ///< Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT          (0x20)    ///< 1 = read/write word (rather than byte)
#define TSL2561_BLOCK_BIT         (0x10)    ///< 1 = using block read/write


// T, FN and CL package values
#define TSL2561_LUX_K1T           (0x0040)  ///< 0.125 * 2^RATIO_SCALE
#define TSL2561_LUX_B1T           (0x01f2)  ///< 0.0304 * 2^LUX_SCALE
#define TSL2561_LUX_M1T           (0x01be)  ///< 0.0272 * 2^LUX_SCALE
#define TSL2561_LUX_K2T           (0x0080)  ///< 0.250 * 2^RATIO_SCALE
#define TSL2561_LUX_B2T           (0x0214)  ///< 0.0325 * 2^LUX_SCALE
#define TSL2561_LUX_M2T           (0x02d1)  ///< 0.0440 * 2^LUX_SCALE
#define TSL2561_LUX_K3T           (0x00c0)  ///< 0.375 * 2^RATIO_SCALE
#define TSL2561_LUX_B3T           (0x023f)  ///< 0.0351 * 2^LUX_SCALE
#define TSL2561_LUX_M3T           (0x037b)  ///< 0.0544 * 2^LUX_SCALE
#define TSL2561_LUX_K4T           (0x0100)  ///< 0.50 * 2^RATIO_SCALE
#define TSL2561_LUX_B4T           (0x0270)  ///< 0.0381 * 2^LUX_SCALE
#define TSL2561_LUX_M4T           (0x03fe)  ///< 0.0624 * 2^LUX_SCALE
#define TSL2561_LUX_K5T           (0x0138)  ///< 0.61 * 2^RATIO_SCALE
#define TSL2561_LUX_B5T           (0x016f)  ///< 0.0224 * 2^LUX_SCALE
#define TSL2561_LUX_M5T           (0x01fc)  ///< 0.0310 * 2^LUX_SCALE
#define TSL2561_LUX_K6T           (0x019a)  ///< 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6T           (0x00d2)  ///< 0.0128 * 2^LUX_SCALE
#define TSL2561_LUX_M6T           (0x00fb)  ///< 0.0153 * 2^LUX_SCALE
#define TSL2561_LUX_K7T           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7T           (0x0018)  ///< 0.00146 * 2^LUX_SCALE
#define TSL2561_LUX_M7T           (0x0012)  ///< 0.00112 * 2^LUX_SCALE
#define TSL2561_LUX_K8T           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8T           (0x0000)  ///< 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8T           (0x0000)  ///< 0.000 * 2^LUX_SCALE

// CS package values
#define TSL2561_LUX_K1C           (0x0043)  ///< 0.130 * 2^RATIO_SCALE
#define TSL2561_LUX_B1C           (0x0204)  ///< 0.0315 * 2^LUX_SCALE
#define TSL2561_LUX_M1C           (0x01ad)  ///< 0.0262 * 2^LUX_SCALE
#define TSL2561_LUX_K2C           (0x0085)  ///< 0.260 * 2^RATIO_SCALE
#define TSL2561_LUX_B2C           (0x0228)  ///< 0.0337 * 2^LUX_SCALE
#define TSL2561_LUX_M2C           (0x02c1)  ///< 0.0430 * 2^LUX_SCALE
#define TSL2561_LUX_K3C           (0x00c8)  ///< 0.390 * 2^RATIO_SCALE
#define TSL2561_LUX_B3C           (0x0253)  ///< 0.0363 * 2^LUX_SCALE
#define TSL2561_LUX_M3C           (0x0363)  ///< 0.0529 * 2^LUX_SCALE
#define TSL2561_LUX_K4C           (0x010a)  ///< 0.520 * 2^RATIO_SCALE
#define TSL2561_LUX_B4C           (0x0282)  ///< 0.0392 * 2^LUX_SCALE
#define TSL2561_LUX_M4C           (0x03df)  ///< 0.0605 * 2^LUX_SCALE
#define TSL2561_LUX_K5C           (0x014d)  ///< 0.65 * 2^RATIO_SCALE
#define TSL2561_LUX_B5C           (0x0177)  ///< 0.0229 * 2^LUX_SCALE
#define TSL2561_LUX_M5C           (0x01dd)  ///< 0.0291 * 2^LUX_SCALE
#define TSL2561_LUX_K6C           (0x019a)  ///< 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6C           (0x0101)  ///< 0.0157 * 2^LUX_SCALE
#define TSL2561_LUX_M6C           (0x0127)  ///< 0.0180 * 2^LUX_SCALE
#define TSL2561_LUX_K7C           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7C           (0x0037)  ///< 0.00338 * 2^LUX_SCALE
#define TSL2561_LUX_M7C           (0x002b)  ///< 0.00260 * 2^LUX_SCALE
#define TSL2561_LUX_K8C           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8C           (0x0000)  ///< 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8C           (0x0000)  ///< 0.000 * 2^LUX_SCALE

// Auto-gain thresholds
#define TSL2561_AGC_THI_13MS      (4850)    ///< Max value at Ti 13ms = 5047
#define TSL2561_AGC_TLO_13MS      (100)     ///< Min value at Ti 13ms = 100
#define TSL2561_AGC_THI_101MS     (36000)   ///< Max value at Ti 101ms = 37177
#define TSL2561_AGC_TLO_101MS     (200)     ///< Min value at Ti 101ms = 200
#define TSL2561_AGC_THI_402MS     (63000)   ///< Max value at Ti 402ms = 65535
#define TSL2561_AGC_TLO_402MS     (500)     ///< Min value at Ti 402ms = 500

// Clipping thresholds
#define TSL2561_CLIPPING_13MS     (4900)    ///< # Counts that trigger a change in gain/integration
#define TSL2561_CLIPPING_101MS    (37000)   ///< # Counts that trigger a change in gain/integration
#define TSL2561_CLIPPING_402MS    (65000)   ///< # Counts that trigger a change in gain/integration


volatile static bool tsl_irq_fired = false;

/* ISR */
void tsl_isr_fxn() {   tsl_irq_fired = true;   }


/*
* Constructor
*/
TSL2561::TSL2561(uint8_t a, uint8_t _i_pin) : _IRQ_PIN(_i_pin), _ADDR(a) {}


/*
* Destructor
*/
TSL2561::~TSL2561() {}


/*!
* @brief  Initializes I2C connection and settings. Attempts to determine if the
*         sensor is contactable, then sets up a default integration time and
*         gain. Then powers down the chip.
* @returns 0 if sensor is found and initialized, negative otherwise.
*/
int8_t TSL2561::init(TwoWire* b) {
  int8_t ret = -1;
  _ll_pin_init();
  _tsl_clear_flag(TSL2561_FLAG_INITIALIZED);
  if (nullptr != b) {
    _i2c = b;
    /* Make sure we're actually connected */
    uint8_t x = _read8(TSL2561_REGISTER_ID);
    ret = -2;
    _tsl_set_flag(TSL2561_FLAG_DEVICE_PRESENT, !(x & 0x05)); // ID code for TSL2561. Excludes TSL2560.
    if (devFound()) {
      ret = highGain(false);
      if (0 == ret) {   // Set default integration time and gain.
        ret = enabled(true);
        _tsl_set_flag(TSL2561_FLAG_INITIALIZED, (0 == ret));
      }
    }
  }
  return ret;
}


/*
* Poll the class for updates.
*/
int8_t TSL2561::poll() {
  int8_t ret = -3;
  if (initialized() && enabled()) {
    ret = 0;
    if (255 != _IRQ_PIN) {
      if (tsl_irq_fired) {
        ret = (0 <= calculateLux()) ? 1 : -1;
        tsl_irq_fired = !digitalRead(_IRQ_PIN);
      }
    }
    else {
      uint32_t now = millis();
      uint32_t r_interval = 403;
      switch (integrationTime()) {
        case TSLIntegrationTime::MS_13:    r_interval -= 88;   // No break
        case TSLIntegrationTime::MS_101:   r_interval -= 301;  // No break
        case TSLIntegrationTime::MS_402:
          if ((now - _last_read) >= r_interval) {
            ret = (0 <= calculateLux()) ? 1 :-1;
          }
          break;
        case TSLIntegrationTime::INVALID:
          ret = -2;
          break;
      }
    }
  }
  return ret;
}


/*!
* @brief      Sets the integration time for the TSL2561. Higher time means
*             more light captured (better for low light conditions) but will
* take longer to run readings.
* @param time The amount of time we'd like to add up values
*/
int8_t TSL2561::integrationTime(TSLIntegrationTime time) {
  int8_t ret = -1;
  if (devFound()) {
    /* Update the timing register */
    uint8_t val = _tsl_flag(TSL2561_FLAG_GAIN_16X) ? 0x10 : 0x00;
    switch (time) {
      case TSLIntegrationTime::MS_13:
      case TSLIntegrationTime::MS_101:
      case TSLIntegrationTime::MS_402:
        val |= (uint8_t) time;
        if (0 == _write8(0x80 | TSL2561_REGISTER_TIMING, val)) {
          _tsl_clear_flag(TSL2561_FLAG_INTEGRATION_MASK);
          _tsl_set_flag(((uint8_t) time) << 6);
          ret = 0;
        }
        break;
      case TSLIntegrationTime::INVALID:
        ret = -2;
        break;
    }
  }
  return ret;
}


/*!
* @brief  Adjusts the gain on the TSL2561 (adjusts the sensitivity to light)
* @param gain The value we'd like to set the gain to
*/
int8_t TSL2561::highGain(bool x) {
  int8_t ret = -1;
  if (devFound()) {
    /* Update the timing register */
    uint8_t val = (x ? 0x10 : 0x00) | ((_flags >> 6) & 0x03);
    if (0 == _write8(0x80 | TSL2561_REGISTER_TIMING, val)) {
      ret = 0;
      _tsl_set_flag(TSL2561_FLAG_GAIN_16X, x);
    }
  }
  return ret;
}


/*!
* @brief  Gets the broadband (mixed lighting) and IR only values from
*         the TSL2561, adjusting gain if auto-gain is enabled
* @param  broadband Pointer to a uint16_t we will fill with a sensor
*                   reading from the IR+visible light diode.
* @param  ir Pointer to a uint16_t we will fill with a sensor the
*            IR-only light diode.
*/
int8_t TSL2561::getLuminosity() {
  int8_t ret = -1;
  bool valid = false;
  if (initialized()) {
    /* If Auto gain disabled get a single reading and continue */
    if (!autogain()) {
      _read_data_registers(&_broadband, &_infrared);
      ret = 0;
    }
    else {
      /* Read data until we find a valid range */
      bool _agcCheck = false;
      do {
        uint16_t _b, _ir;
        uint16_t _hi, _lo;

        /* Get the hi/low threshold for the current integration time */
        switch(integrationTime()) {
          case TSLIntegrationTime::MS_13:
            _hi = TSL2561_AGC_THI_13MS;
            _lo = TSL2561_AGC_TLO_13MS;
            break;
          case TSLIntegrationTime::MS_101:
            _hi = TSL2561_AGC_THI_101MS;
            _lo = TSL2561_AGC_TLO_101MS;
            break;
          case TSLIntegrationTime::MS_402:
            _hi = TSL2561_AGC_THI_402MS;
            _lo = TSL2561_AGC_TLO_402MS;
            break;
          default:
            ret = -2;
            return ret;
        }

        _read_data_registers(&_b, &_ir);

        /* Run an auto-gain check if we haven't already done so ... */
        if (!_agcCheck) {
          if ((_b < _lo) && (!highGain())) {
            /* Increase the gain and try again */
            highGain(true);
            /* Drop the previous conversion results */
            _read_data_registers(&_b, &_ir);
            /* Set a flag to indicate we've adjusted the gain */
            _agcCheck = true;
          }
          else if ((_b > _hi) && highGain()) {
            /* Drop gain to 1x and try again */
            highGain(false);
            /* Drop the previous conversion results */
            _read_data_registers(&_b, &_ir);
            /* Set a flag to indicate we've adjusted the gain */
            _agcCheck = true;
          }
          else {
            /* Nothing to look at here, keep moving ....
            Reading is either valid, or we're already at the chips limits */
            _broadband = _b;
            _infrared = _ir;
            ret = 0;
            valid = true;
          }
        }
        else {
          /* If we've already adjusted the gain once, just return the new results.
          This avoids endless loops where a value is at one extreme pre-gain,
          and the the other extreme post-gain */
          _broadband = _b;
          _infrared = _ir;
          ret = 0;
          valid = true;
        }
      } while (!valid);
    }
  }
  return ret;
}


/**
* Enable the device by setting the control bit to 0x03
*/
int8_t TSL2561::enabled(bool x) {
  int8_t ret = -1;
  if (devFound()) {
    ret = _write8(0x80 | TSL2561_REGISTER_CONTROL, x ? 0x03 : 0x00);
    if (0 == ret) {
      _tsl_set_flag(TSL2561_FLAG_ENABLED, x);
    }
  }
  return ret;
}


/*!
* @brief  Converts the raw sensor values to the standard SI lux equivalent.
* @param  broadband The 16-bit sensor reading from the IR+visible light diode.
* @param  ir The 16-bit sensor reading from the IR-only light diode.
* @returns negative on error, 0 on nominal return, or 1 on saturation.
*/
int8_t TSL2561::calculateLux() {
  int8_t ret = getLuminosity();
  uint16_t clipThreshold;
  unsigned long chScale;
  if (0 <= ret) {
    // Make sure the sensor isn't saturated and get the correct
    //   scale depending on the intergration time.
    switch (integrationTime()) {
      case TSLIntegrationTime::MS_13:
        clipThreshold = TSL2561_CLIPPING_13MS;
        chScale = TSL2561_LUX_CHSCALE_TINT0;
        break;
      case TSLIntegrationTime::MS_101:
        clipThreshold = TSL2561_CLIPPING_101MS;
        chScale = TSL2561_LUX_CHSCALE_TINT1;
        break;
      case TSLIntegrationTime::MS_402:
        clipThreshold = TSL2561_CLIPPING_402MS;
        chScale = (1 << TSL2561_LUX_CHSCALE);
        break;
      default:
        ret = -2;
        return ret;
    }

    /* Return 65536 lux if the sensor is saturated */
    if ((_broadband > clipThreshold) || (_infrared > clipThreshold)) {
      _lux = 65536;
      ret = 1;
    }
    else {
      /* Scale for gain (1x or 16x) */
      if (!highGain()) {    chScale = chScale << 4;    }

      /* Scale the channel values */
      unsigned long channel0 = (_broadband * chScale) >> TSL2561_LUX_CHSCALE;
      unsigned long channel1 = (_infrared * chScale) >> TSL2561_LUX_CHSCALE;

      /* Find the ratio of the channel values (Channel1/Channel0) */
      unsigned long ratio1 = (channel0 != 0) ? ((channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0) : 0;

      /* round the ratio value */
      unsigned long ratio = (ratio1 + 1) >> 1;

      unsigned int b, m;

      #ifdef TSL2561_PACKAGE_CS
        if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C)) {  b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;  }
        else if (ratio <= TSL2561_LUX_K2C) {  b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;   }
        else if (ratio <= TSL2561_LUX_K3C) {  b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;   }
        else if (ratio <= TSL2561_LUX_K4C) {  b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;   }
        else if (ratio <= TSL2561_LUX_K5C) {  b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;   }
        else if (ratio <= TSL2561_LUX_K6C) {  b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;   }
        else if (ratio <= TSL2561_LUX_K7C) {  b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;   }
        else if (ratio > TSL2561_LUX_K8C) {   b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;   }
      #else
        if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T)) {  b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;  }
        else if (ratio <= TSL2561_LUX_K2T) {  b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;   }
        else if (ratio <= TSL2561_LUX_K3T) {  b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;   }
        else if (ratio <= TSL2561_LUX_K4T) {  b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;   }
        else if (ratio <= TSL2561_LUX_K5T) {  b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;   }
        else if (ratio <= TSL2561_LUX_K6T) {  b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;   }
        else if (ratio <= TSL2561_LUX_K7T) {  b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;   }
        else if (ratio > TSL2561_LUX_K8T) {   b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;   }
      #endif

      long temp = ((channel0 * b) - (channel1 * m));
      if (temp < 0) {
        temp = 0;   // Do not allow negative lux value
        ret = -3;
      }
      else {
        temp += (1 << (TSL2561_LUX_LUXSCALE - 1));  // Round lsb (2^(LUX_SCALE-1))
        _lux = temp >> TSL2561_LUX_LUXSCALE;   // Strip off fractional portion.
        ret = 0;
      }
    }
  }
  return ret;
}


/*
* Idempotently setup the low-level pin details.
*/
int8_t TSL2561::_ll_pin_init() {
  int8_t ret = 0;   // This function cannot fail.
  if (!_tsl_flag(TSL2561_FLAG_PINS_CONFIGURED)) {
    if (255 != _IRQ_PIN) {
      pinMode(_IRQ_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(_IRQ_PIN), tsl_isr_fxn, FALLING);
    }
    _tsl_set_flag(TSL2561_FLAG_PINS_CONFIGURED);
  }
  return ret;
}


/*!
* @brief  Writes a register and an 8 bit value over I2C
* @param  reg I2C register to write the value to
* @param  value The 8-bit value we're writing to the register
*/
int8_t TSL2561::_write8 (uint8_t reg, uint8_t value) {
  _i2c->beginTransmission(_ADDR);
  _i2c->write(reg);
  _i2c->write(value);
  return _i2c->endTransmission();
}


/*!
* @brief  Reads an 8 bit value over I2C
* @param  reg I2C register to read from
* @returns 8-bit value containing single byte data read
*/
uint8_t TSL2561::_read8(uint8_t reg) {
  _i2c->beginTransmission(_ADDR);
  _i2c->write(reg);
  _i2c->endTransmission();
  _i2c->requestFrom((int) _ADDR, 1);
  return _i2c-> read();
}


/*!
* @brief  Reads a 16 bit values over I2C
* @param  reg I2C register to read from
* @returns 16-bit value containing 2-byte data read
*/
uint16_t TSL2561::_read16(uint8_t reg) {
  uint16_t x, t;
  _i2c->beginTransmission(_ADDR);
  _i2c->write(reg);
  _i2c->endTransmission();
  _i2c->requestFrom((int) _ADDR, 2);
  t = _i2c->read();
  x = _i2c->read();
  x <<= 8;
  x |= t;
  return x;
}


/**
* Read luminosity on both channels.
*/
void TSL2561::_read_data_registers(uint16_t* bb, uint16_t* ir) {
  /* Reads a two byte value from channel 0 (visible + infrared) */
  *bb = _read16(0x80 | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);

  /* Reads a two byte value from channel 1 (infrared) */
  *ir = _read16(0x80 | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
  _last_read = millis();
}
