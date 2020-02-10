/*
A C++ interface to the ICM-20948
*/

#ifndef _ICM_20948_H_
#define _ICM_20948_H_

#include "util/ICM_20948_C.h" // The C backbone

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// SPI defines
#define ICM_20948_SPI_DEFAULT_FREQ 4000000
#define ICM_20948_SPI_DEFAULT_ORDER MSBFIRST
#define ICM_20948_SPI_DEFAULT_MODE SPI_MODE0

#define ICM_20948_ARD_UNUSED_PIN 0xFF


// Forward declarations of TwoWire and Wire for board/variant combinations that don't have a default 'SPI'
class TwoWire;
extern TwoWire Wire;

// Forward declarations of SPIClass and SPI for board/variant combinations that don't have a default 'SPI'
class SPIClass;
extern SPIClass SPI;


// Base
class ICM_20948 {
  public:
    ICM_20948(); // Constructor

    ICM_20948_AGMT_t agmt;          // Acceleometer, Gyroscope, Magenetometer, and Temperature data
    ICM_20948_AGMT_t getAGMT(); // Updates the agmt field in the object and also returns a copy directly

    float magX(); // micro teslas
    float magY(); // micro teslas
    float magZ(); // micro teslas

    float accX(); // milli g's
    float accY(); // milli g's
    float accZ(); // milli g's

    float gyrX(); // degrees per second
    float gyrY(); // degrees per second
    float gyrZ(); // degrees per second

    float temp(); // degrees celsius

    ICM_20948_Status_e status;                                              // Status from latest operation
    const char *statusString(ICM_20948_Status_e stat = ICM_20948_Stat_NUM); // Returns a human-readable status message. Defaults to status member, but prints string for supplied status if supplied

    // Device Level
    ICM_20948_Status_e setBank(uint8_t bank);                                // Sets the bank
    ICM_20948_Status_e swReset();                                        // Performs a SW reset
    ICM_20948_Status_e sleep(bool on = false);                               // Set sleep mode for the chip
    ICM_20948_Status_e lowPower(bool on = true);                             // Set low power mode for the chip
    ICM_20948_Status_e setClockSource(ICM_20948_PWR_MGMT_1_CLKSEL_e source); // Choose clock source
    ICM_20948_Status_e checkID();                                        // Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI

    bool dataReady();    // Returns 'true' if data is ready
    uint8_t getWhoAmI(); // Return whoami in out prarmeter
    bool isConnected();  // Returns true if communications with the device are sucessful

    // Internal Sensor Options
    ICM_20948_Status_e setSampleMode(uint8_t sensor_id_bm, uint8_t lp_config_cycle_mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
    ICM_20948_Status_e setFullScale(uint8_t sensor_id_bm, ICM_20948_fss_t fss);
    ICM_20948_Status_e setDLPFcfg(uint8_t sensor_id_bm, ICM_20948_dlpcfg_t cfg);
    ICM_20948_Status_e enableDLPF(uint8_t sensor_id_bm, bool enable);
    ICM_20948_Status_e setSampleRate(uint8_t sensor_id_bm, ICM_20948_smplrt_t smplrt);

    // Interrupts on INT and FSYNC Pins
    ICM_20948_Status_e clearInterrupts();

    ICM_20948_Status_e cfgIntActiveLow(bool active_low);
    ICM_20948_Status_e cfgIntOpenDrain(bool open_drain);
    ICM_20948_Status_e cfgIntLatch(bool latching);         // If not latching then the interrupt is a 50 us pulse
    ICM_20948_Status_e cfgIntAnyReadToClear(bool enabled); // If enabled, *ANY* read will clear the INT_STATUS register. So if you have multiple interrupt sources enabled be sure to read INT_STATUS first
    ICM_20948_Status_e cfgFsyncActiveLow(bool active_low);
    ICM_20948_Status_e cfgFsyncIntMode(bool interrupt_mode); // Can ue FSYNC as an interrupt input that sets the I2C Master Status register's PASS_THROUGH bit

    ICM_20948_Status_e intEnableI2C(bool enable);
    ICM_20948_Status_e intEnableDMP(bool enable);
    ICM_20948_Status_e intEnablePLL(bool enable);
    ICM_20948_Status_e intEnableWOM(bool enable);
    ICM_20948_Status_e intEnableWOF(bool enable);
    ICM_20948_Status_e intEnableRawDataReady(bool enable);
    ICM_20948_Status_e intEnableOverflowFIFO(uint8_t bm_enable);
    ICM_20948_Status_e intEnableWatermarkFIFO(uint8_t bm_enable);

    // Interface Options
    ICM_20948_Status_e i2cMasterPassthrough(bool passthrough = true);
    ICM_20948_Status_e i2cMasterEnable(bool enable = true);
    ICM_20948_Status_e i2cMasterReset();

    //Used for configuring slaves 0-3
    ICM_20948_Status_e i2cMasterConfigureSlave(uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool Rw = true, bool enable = true, bool data_only = false, bool grp = false, bool swap = false);
    ICM_20948_Status_e i2cMasterSLV4Transaction(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr = true);

    //Used for configuring the Magnetometer
    ICM_20948_Status_e i2cMasterSingleW(uint8_t addr, uint8_t reg, uint8_t data);
    uint8_t i2cMasterSingleR(uint8_t addr, uint8_t reg);

    // Default Setup
    ICM_20948_Status_e startupDefault();

    // direct read/write
    ICM_20948_Status_e read(uint8_t reg, uint8_t *pdata, uint32_t len);
    ICM_20948_Status_e write(uint8_t reg, uint8_t *pdata, uint32_t len);

  protected:
    ICM_20948_Device_t _device;

    float getTempC(int16_t val);
    float getGyrDPS(int16_t axis_val);
    float getAccMG(int16_t axis_val);
    float getMagUT(int16_t axis_val);


  private:
    /*
    ICM_20948_Status_e ICM_20948_link_serif(ICM_20948_Device_t *pdev, const ICM_20948_Serif_t *s); // Links a SERIF structure to the device

    // use the device's serif to perform a read or write
    ICM_20948_Status_e ICM_20948_execute_r(ICM_20948_Device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len); // Executes a R or W witht he serif vt as long as the pointers are not null
    ICM_20948_Status_e ICM_20948_execute_w(ICM_20948_Device_t *pdev, uint8_t regaddr, uint8_t *pdata, uint32_t len);

    // Single-shot I2C on Master IF
    ICM_20948_Status_e ICM_20948_i2c_master_slv4_txn(ICM_20948_Device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len, bool Rw, bool send_reg_addr);
    ICM_20948_Status_e ICM_20948_i2c_master_single_w(ICM_20948_Device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data);
    ICM_20948_Status_e ICM_20948_i2c_master_single_r(ICM_20948_Device_t *pdev, uint8_t addr, uint8_t reg, uint8_t *data);

    // Device Level
    ICM_20948_Status_e ICM_20948_set_bank(ICM_20948_Device_t *pdev, uint8_t bank);                   // Sets the bank
    ICM_20948_Status_e ICM_20948_sw_reset(ICM_20948_Device_t *pdev);                         // Performs a SW reset
    ICM_20948_Status_e ICM_20948_sleep(ICM_20948_Device_t *pdev, bool on);                       // Set sleep mode for the chip
    ICM_20948_Status_e ICM_20948_low_power(ICM_20948_Device_t *pdev, bool on);                     // Set low power mode for the chip
    ICM_20948_Status_e ICM_20948_set_clock_source(ICM_20948_Device_t *pdev, ICM_20948_PWR_MGMT_1_CLKSEL_e source); // Choose clock source
    ICM_20948_Status_e ICM_20948_get_who_am_i(ICM_20948_Device_t *pdev, uint8_t *whoami);               // Return whoami in out prarmeter
    ICM_20948_Status_e ICM_20948_check_id(ICM_20948_Device_t *pdev);                         // Return 'ICM_20948_Stat_Ok' if whoami matches ICM_20948_WHOAMI
    ICM_20948_Status_e ICM_20948_data_ready(ICM_20948_Device_t *pdev);                         // Returns 'Ok' if data is ready

    // Interrupt Configuration
    ICM_20948_Status_e ICM_20948_int_pin_cfg(ICM_20948_Device_t *pdev, ICM_20948_INT_PIN_CFG_t *write, ICM_20948_INT_PIN_CFG_t *read); // Set the INT pin configuration
    ICM_20948_Status_e ICM_20948_int_enable(ICM_20948_Device_t *pdev, ICM_20948_INT_enable_t *write, ICM_20948_INT_enable_t *read);  // Write and or read the interrupt enable information. If non-null the write operation occurs before the read, so as to verify that the write was successful

    // Internal Sensor Options
    ICM_20948_Status_e ICM_20948_set_sample_mode(ICM_20948_Device_t *pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_LP_CONFIG_CYCLE_e mode); // Use to set accel, gyro, and I2C master into cycled or continuous modes
    ICM_20948_Status_e ICM_20948_set_full_scale(ICM_20948_Device_t *pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_fss_t fss);
    ICM_20948_Status_e ICM_20948_set_dlpf_cfg(ICM_20948_Device_t *pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_dlpcfg_t cfg);
    ICM_20948_Status_e ICM_20948_enable_dlpf(ICM_20948_Device_t *pdev, ICM_20948_InternalSensorID_bm sensors, bool enable);
    ICM_20948_Status_e ICM_20948_set_sample_rate(ICM_20948_Device_t *pdev, ICM_20948_InternalSensorID_bm sensors, ICM_20948_smplrt_t smplrt);

    // Interface Things
    ICM_20948_Status_e ICM_20948_i2c_master_passthrough(ICM_20948_Device_t *pdev, bool passthrough);
    ICM_20948_Status_e ICM_20948_i2c_master_enable(ICM_20948_Device_t *pdev, bool enable);
    ICM_20948_Status_e ICM_20948_i2c_master_reset(ICM_20948_Device_t *pdev);
    ICM_20948_Status_e ICM_20948_i2c_master_configure_slave(ICM_20948_Device_t *pdev, uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool Rw, bool enable, bool data_only, bool grp, bool swap);

    // Higher Level
    ICM_20948_Status_e ICM_20948_get_agmt(ICM_20948_Device_t *pdev, ICM_20948_AGMT_t *p);
    ICM_20948_Status_e ICM_20948_get_agmt(ICM_20948_Device_t *pdev, ICM_20948_AGMT_t *p);
    */
};


/* I2C */
class ICM_20948_I2C : public ICM_20948 {
  public:
    TwoWire*          _i2c;
    uint8_t           _addr;
    uint8_t           _ad0;
    bool              _ad0val;
    ICM_20948_Serif_t _serif;

    ICM_20948_I2C(); // Constructor
    virtual ICM_20948_Status_e begin(TwoWire &wirePort = Wire, bool ad0val = true, uint8_t ad0pin = ICM_20948_ARD_UNUSED_PIN);

  protected:
  private:
};


/* SPI */
class ICM_20948_SPI : public ICM_20948 {
  public:
    SPIClass*          _spi;
    SPISettings        _spisettings;
    uint8_t            _cs;
    ICM_20948_Serif_t _serif;

    ICM_20948_SPI(); // Constructor
    ICM_20948_Status_e begin(uint8_t csPin, SPIClass &spiPort = SPI, uint32_t SPIFreq = ICM_20948_SPI_DEFAULT_FREQ);

  protected:
  private:
};

#endif /* _ICM_20948_H_ */
