/*
* This file started out as a SparkFun driver. See header file for
*   my change list.
*                                                            ---J. Ian Lindsay
*/

/*
  This is a library written for the Panasonic Grid-EYE AMG88
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14568

  Written by Nick Poole @ SparkFun Electronics, January 11th, 2018

  The GridEYE from Panasonic is an 8 by 8 thermopile array capable
  of detecting temperature remotely at 64 discrete points.

  This library handles communication with the GridEYE and provides
  methods for manipulating temperature registers in Celsius,
  Fahrenheit and raw values.

  https://github.com/sparkfun/SparkFun_GridEYE_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AMG88xx.h"

volatile static bool amg_irq_fired = false;

/* ISR */
void amg_isr_fxn() {   amg_irq_fired = true;   }



/*
* Constructor
*/
GridEYE::GridEYE(uint8_t addr, uint8_t irq) : _ADDR(addr), _IRQ_PIN(irq) {}

/*
* Detructor
*/
GridEYE::~GridEYE() {}


/*
* Init the sensor on the given bus.
* Successful init() means the sensor is running at 10FPS.
*/
int8_t GridEYE::init(TwoWire* b) {
  int8_t ret = -1;
  _ll_pin_init();  // Idempotent. Ok to call twice.
  _amg_clear_flag(GRIDEYE_FLAG_INITIALIZED);
  for (uint8_t i = 0; i < 64; i++) {
    _frame[i] = 0;   // Zero the local framebuffer.
  }
  if (nullptr != b) {
    _i2c = b;
    if (0 == setFramerate10FPS()) {
      _amg_set_flag(GRIDEYE_FLAG_DEVICE_PRESENT);
      if (0 == enabled(true)) {
        _amg_set_flag(GRIDEYE_FLAG_INITIALIZED);
        ret = 0;
      }
    }
  }
  return ret;
}


/*
* Poll the class for updates.
* Returns...
*   -3 if not initialized and enabled.
*   -1 if the frame needed to be read, but doing so failed.
*   0  if nothing needs doing.
*   1  if a frame was read and is waiting.
*/
int8_t GridEYE::poll() {
  int8_t ret = -3;
  if (initialized() && enabled()) {
    ret = 0;
    if (255 != _IRQ_PIN) {
      if (amg_irq_fired) {
        ret = (0 == _read_full_frame()) ? 1 : -1;
        amg_irq_fired = !digitalRead(_IRQ_PIN);
      }
    }
    else {
      uint32_t now = millis();
      uint32_t r_interval = isFramerate10FPS() ? 100 : 1000;
      if ((now - _last_read) >= r_interval) {
        ret = (0 == _read_full_frame()) ? 1 : -1;
      }
    }
  }
  return ret;
}


/**
*
*/
float GridEYE::getPixelTemperature(uint8_t pixel) {
  int16_t temperature = _dev_int16_to_float(getPixelRaw(pixel));
  return _normalize_units_returned(temperature * 0.25);
}


/**
*
*/
float GridEYE::getDeviceTemperature() {
  int16_t temperature = _dev_int16_to_float(_read_registers(THERMISTOR_REGISTER_LSB, 2));
  return _normalize_units_returned(temperature * 0.0625);
}


/**
*
*/
int16_t GridEYE::getDeviceTemperatureRaw() {
  return _read_registers(THERMISTOR_REGISTER_LSB, 2);
}


/**
*
*/
int8_t GridEYE::setFramerate1FPS() {
  int8_t ret = _write_register(FRAMERATE_REGISTER, 1);
  if (0 == ret) {
    _amg_clear_flag(GRIDEYE_FLAG_10FPS);
  }
  return ret;
}


/**
*
*/
int8_t GridEYE::setFramerate10FPS() {
  int8_t ret = _write_register(FRAMERATE_REGISTER, 0);
  if (0 == ret) {
    _amg_set_flag(GRIDEYE_FLAG_10FPS);
  }
  return ret;
}


/**
*
*/
int8_t GridEYE::enabled(bool x) {
  int8_t ret = _write_register(POWER_CONTROL_REGISTER, x ? 0x00 : 0x10);
  if (0 == ret) {
    _amg_set_flag(GRIDEYE_FLAG_ENABLED, x);
  }
  return ret;
}


/**
*
*/
int8_t GridEYE::standby60seconds(){
  return _write_register(POWER_CONTROL_REGISTER, 0x20);
}


/**
*
*/
int8_t GridEYE::standby10seconds(){
  return _write_register(POWER_CONTROL_REGISTER, 0x21);
}


/**
*
*/
int8_t GridEYE::interruptPinEnable(){
  int16_t ICRValue = _read_registers(INT_CONTROL_REGISTER, 1);
  ICRValue |= (1 << 0);
  return _write_register(INT_CONTROL_REGISTER, ICRValue & 0xFF);
}


/**
*
*/
int8_t GridEYE::interruptPinDisable(){
  int16_t ICRValue = _read_registers(INT_CONTROL_REGISTER, 1);
  ICRValue &= ~(1 << 0);
  return _write_register(INT_CONTROL_REGISTER, ICRValue & 0xFF);
}


/**
*
*/
int8_t GridEYE::setInterruptModeAbsolute(){
  int16_t ICRValue = _read_registers(INT_CONTROL_REGISTER, 1);
  ICRValue |= (1 << 1);
  return _write_register(INT_CONTROL_REGISTER, ICRValue & 0xFF);
}


/**
*
*/
int8_t GridEYE::setInterruptModeDifference(){
  int16_t ICRValue = _read_registers(INT_CONTROL_REGISTER, 1);
  ICRValue &= ~(1 << 1);
  return _write_register(INT_CONTROL_REGISTER, ICRValue & 0xFF);
}


/**
*
*/
bool GridEYE::interruptPinEnabled(){
  int16_t ICRValue = _read_registers(INT_CONTROL_REGISTER, 1);
  return (ICRValue & (1 << 0));
}


/**
*
*/
bool GridEYE::interruptFlagSet(){
  int16_t StatRegValue = _read_registers(STATUS_REGISTER, 1);
  return (StatRegValue & (1 << 1));
}


/**
*
*/
bool GridEYE::pixelTemperatureOutputOK(){
  int16_t StatRegValue = _read_registers(STATUS_REGISTER, 1);
  return !(StatRegValue & (1 << 2));
}


/**
*
*/
bool GridEYE::deviceTemperatureOutputOK(){
  int16_t StatRegValue = _read_registers(STATUS_REGISTER, 1);
  return !(StatRegValue & (1 << 3));
}


/**
*
*/
int8_t GridEYE::clearInterruptFlag(){
  return _write_register(STATUS_CLEAR_REGISTER, 0x02);
}


/**
*
*/
int8_t GridEYE::clearPixelTemperatureOverflow(){
  return _write_register(STATUS_CLEAR_REGISTER, 0x04);
}


/**
*
*/
int8_t GridEYE::clearDeviceTemperatureOverflow(){
  return _write_register(STATUS_CLEAR_REGISTER, 0x08);
}


/**
*
*/
int8_t GridEYE::clearAllOverflow(){
  return _write_register(STATUS_CLEAR_REGISTER, 0x0C);
}


/**
*
*/
int8_t GridEYE::clearAllStatusFlags(){
  return _write_register(STATUS_CLEAR_REGISTER, 0x0E);
}


/**
*
*/
bool GridEYE::pixelInterruptSet(uint8_t pixel){
  unsigned char interruptTableRegister = INT_TABLE_REGISTER_INT0 + (pixel >> 3);
  uint8_t pixelPosition = (pixel & 0x07);
  int16_t interruptTableRow = _read_registers(interruptTableRegister, 1);
  return (interruptTableRow & (1 << pixelPosition));
}


/**
*
*/
int8_t GridEYE::movingAverage(bool x) {
  int8_t ret = -1;
  if (0 == _write_register(RESERVED_AVERAGE_REGISTER, 0x50)) {
    if (0 == _write_register(RESERVED_AVERAGE_REGISTER, 0x45)) {
      if (0 == _write_register(RESERVED_AVERAGE_REGISTER, 0x57)) {
        if (0 == _write_register(AVERAGE_REGISTER, x ? 0x20 : 0x00)) {
          if (0 == _write_register(RESERVED_AVERAGE_REGISTER, 0x00)) {
            _amg_set_flag(GRIDEYE_FLAG_HW_AVERAGING, x);
            ret = 0;
          }
        }
      }
    }
  }
  return ret;
}


/**
* Reads all device registers and configures the class appropriately.
* TODO: Finish this
*/
int8_t GridEYE::refresh() {
  int8_t ret = 0;
  int16_t AVGRegValue = _read_registers(AVERAGE_REGISTER, 1);
  _amg_set_flag(GRIDEYE_FLAG_HW_AVERAGING, (AVGRegValue & (1 << 5)));
  return ret;
}


/**
*
*/
int8_t GridEYE::setUpperInterruptValue(float degrees){
  bool isNegative = false;
  float DegreesC = _normalize_units_accepted(degrees);
  if (DegreesC < 0) {
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  int16_t temperature = round(DegreesC*4);
  if (isNegative) {
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  int8_t ret = _write_register(INT_LEVEL_REGISTER_UPPER_LSB, temperature & 0xFF);
  if (0 == ret) {
    ret = _write_register(INT_LEVEL_REGISTER_UPPER_MSB, temperature >> 8);
  }
  return ret;
}


/**
*
*/
int8_t GridEYE::setUpperInterruptValueRaw(int16_t regValue){
  int8_t ret = _write_register(INT_LEVEL_REGISTER_UPPER_LSB, regValue & 0xFF);
  if (0 == ret) {
    ret = _write_register(INT_LEVEL_REGISTER_UPPER_MSB, regValue >> 8);
  }
  return ret;
}


/**
*
*/
int8_t GridEYE::setLowerInterruptValue(float degrees){
  bool isNegative = false;
  float DegreesC = _normalize_units_accepted(degrees);
  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }

  int16_t temperature = round(DegreesC*4);
  if(isNegative) {
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  int8_t ret = _write_register(INT_LEVEL_REGISTER_LOWER_LSB, temperature & 0xFF);
  if (0 == ret) {
    ret = _write_register(INT_LEVEL_REGISTER_LOWER_MSB, temperature >> 8);
  }
  return ret;
}


/**
*
*/
int8_t GridEYE::setLowerInterruptValueRaw(int16_t regValue){
  int8_t ret = _write_register(INT_LEVEL_REGISTER_LOWER_LSB, regValue & 0xFF);
  if (0 == ret) {
    ret = _write_register(INT_LEVEL_REGISTER_LOWER_MSB, regValue >> 8);
  }
  return ret;
}


/**
*
*/
int8_t GridEYE::setInterruptHysteresis(float degrees){
  bool isNegative = false;
  float DegreesC = _normalize_units_accepted(degrees);
  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }

  int16_t temperature = round(DegreesC*4);
  if (isNegative) {
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  int8_t ret = _write_register(INT_LEVEL_REGISTER_HYST_LSB, temperature & 0xFF);
  if (0 == ret) {
    ret = _write_register(INT_LEVEL_REGISTER_HYST_MSB, temperature >> 8);
  }
  return ret;
}


/**
*
*/
int8_t GridEYE::setInterruptHysteresisRaw(int16_t regValue) {
  int8_t ret = _write_register(INT_LEVEL_REGISTER_HYST_LSB, regValue & 0xFF);
  if (0 == ret) {
    ret = _write_register(INT_LEVEL_REGISTER_HYST_MSB, regValue >> 8);
  }
  return ret;
}


/**
*
*/
float GridEYE::getUpperInterruptValue() {
  int16_t temperature = _dev_int16_to_float(_read_registers(INT_LEVEL_REGISTER_UPPER_LSB, 2));
  return _normalize_units_returned(temperature * 0.25);
}


/**
*
*/
int16_t GridEYE::getUpperInterruptValueRaw() {
  return _read_registers(INT_LEVEL_REGISTER_UPPER_LSB, 2);
}


/**
*
*/
float GridEYE::getLowerInterruptValue() {
  int16_t temperature = _dev_int16_to_float(_read_registers(INT_LEVEL_REGISTER_LOWER_LSB, 2));
  return _normalize_units_returned(temperature * 0.25);
}


/**
*
*/
int16_t GridEYE::getLowerInterruptValueRaw() {
  return _read_registers(INT_LEVEL_REGISTER_LOWER_LSB, 2);
}


/**
*
*/
float GridEYE::getInterruptHysteresis() {
  int16_t temperature = _dev_int16_to_float(_read_registers(INT_LEVEL_REGISTER_HYST_LSB, 2));
  return _normalize_units_returned(temperature * 0.25);
}


/**
*
*/
int16_t GridEYE::getInterruptHysteresisRaw() {
  return _read_registers(INT_LEVEL_REGISTER_HYST_LSB, 2);
}


/**
*
*/
int8_t GridEYE::_write_register(uint8_t reg, uint8_t val) {
  _i2c->beginTransmission(_ADDR);
  _i2c->write(reg);
  _i2c->write(val);
  return _i2c->endTransmission();
}


/**
*
*/
int16_t GridEYE::_read_registers(uint8_t reg, uint8_t len) {
  int16_t result = 0;
  _i2c->beginTransmission(_ADDR);
  _i2c->write(reg);
  _i2c->endTransmission(false);
  _i2c->requestFrom(_ADDR, len);

  while(_i2c->available()) {      // client may send less than requested
    uint8_t lsb = _i2c->read();   // Get bytes from sensor
    uint8_t msb = _i2c->read();
    result = (uint16_t)msb << 8 | lsb;   // concat bytes into int
  }
  _i2c->endTransmission();
  return result;
}


/*
* Idempotently setup the low-level pin details.
*/
int8_t GridEYE::_ll_pin_init() {
  int8_t ret = 0;
  if (!_amg_flag(GRIDEYE_FLAG_PINS_CONFIGURED)) {
    if (255 != _IRQ_PIN) {
      pinMode(_IRQ_PIN, INPUT);
      attachInterrupt(digitalPinToInterrupt(_IRQ_PIN), amg_isr_fxn, FALLING);
      interruptPinEnable();
    }
    _amg_set_flag(GRIDEYE_FLAG_PINS_CONFIGURED);
  }
  return ret;
}


/*
* Read the entire frame.
*/
int8_t GridEYE::_read_full_frame() {
  int8_t ret = -1;
  if (initialized() && enabled()) {
    uint8_t dev_reg = TEMPERATURE_REGISTER_START;
    uint8_t offset = 0;
    const uint8_t READ_BLOCK_SIZE  = 16;
    const uint8_t READ_BLOCK_COUNT = 128 / READ_BLOCK_SIZE;
    // Break up the 128-byte transfer into blocks to accomodate small buffers.
    for (uint8_t i = 0; i < READ_BLOCK_COUNT; i++) {
      _i2c->beginTransmission(_ADDR);
      _i2c->write(dev_reg);
      _i2c->endTransmission(false);
      _i2c->requestFrom((uint8_t) _ADDR, (uint8_t) READ_BLOCK_SIZE);
      while(_i2c->available()) {      // client may send less than requested
        uint8_t lsb = _i2c->read();
        uint8_t msb = _i2c->read();
        _frame[offset++] = (uint16_t)msb << 8 | lsb;
      }
      _i2c->endTransmission();
      dev_reg += READ_BLOCK_SIZE;
    }
    _last_read = millis();
    ret = (64 == offset) ? 0 : -2;
  }
  return ret;
}


/**
* Used to automatically convert from Fahrenheit if that is how the class is
*   configured to operate.
*/
float GridEYE::_normalize_units_accepted(float temperature) {
  if (unitsFahrenheit()) {
    temperature = (temperature - 32) / 1.8;
  }
  return temperature;
}


/**
* Used to automatically convert to Fahrenheit if that is how the class is
*   configured to operate.
*/
float GridEYE::_normalize_units_returned(float temperature) {
  if (unitsFahrenheit()) {
    temperature = temperature * 1.8 + 32;
  }
  return temperature;
}


/**
* A conversion function to justify the device's idea of a signed 12-bit
*   against our own. Returns a proper sign-extended representation of the
*   temperature. Temperature is reported as 12-bit twos complement.
*/
int16_t GridEYE::_dev_int16_to_float(int16_t temperature) {
  if (temperature & (1 << 11)) {    // Check if temperature is negative
    temperature &= ~(1 << 11);      // If temperature is negative, mask out the
    temperature = temperature * -1; // sign bit and make the float negative.
  }
  return temperature;
}
