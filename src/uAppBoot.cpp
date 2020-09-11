#include <CppPotpourri.h>
#include <SensorFilter.h>
#include <SX8634.h>
#include <SPIAdapter.h>
#include <I2CAdapter.h>

#include <Audio.h>

#include "Motherflux0r.h"
#include "uApp.h"
#include "ICM20948.h"
#include "DRV425.h"
#include "VL53L0X.h"

extern SSD13xx display;
extern SX8634* touch;

extern SPIAdapter spi0;
extern I2CAdapter i2c0;
extern I2CAdapter i2c1;

// Magnetic pipeline
extern TripleAxisTerminus     mag_vect;
extern TripleAxisCompass      compass;
extern TripleAxisFork         mag_fork;
extern TripleAxisSingleFilter mag_filter;
extern TripleAxisConvention   mag_conv;

// Inertial pipeline
extern TripleAxisTerminus     down;
extern TripleAxisFork         imu_fork;
extern TripleAxisConvention   tilt_conv;

/* Sensor representations... */
extern DRV425 magneto;
extern GridEYE grideye;
extern VEML6075 uv;
extern ICM_20948_SPI imu;
extern TSL2561 tsl2561;
extern BME280I2C baro;
extern VL53L0X tof;

/* Audio objects... */
extern float volume_left_output;
extern float volume_right_output;
extern float volume_pink_noise;
extern float mix_synth_to_fft;
extern float mix_queueL_to_fft;
extern float mix_queueR_to_fft;
extern float mix_noise_to_fft;
extern float mix_synth_to_line;
extern float mix_queue_to_line;
extern float mix_noise_to_line;
extern AudioSynthNoisePink      pinkNoise;
extern AudioSynthWaveformSine   sineL;
extern AudioSynthWaveformSine   sineR;
extern AudioPlayQueue           queueL;
extern AudioPlayQueue           queueR;
extern AudioMixer4              mixerL;
extern AudioMixer4              mixerR;
extern AudioMixer4              mixerFFT;
extern AudioAmplifier           ampR;
extern AudioAmplifier           ampL;
extern AudioAnalyzeFFT256       fft256_1;

extern SensorFilter<float> graph_array_ana_light;


bool   _display_init_called   = false;
bool   _display_init_complete = false;
bool   _lux_init_called       = false;
bool   _lux_init_complete     = false;
bool   _mag_init_called       = false;
bool   _mag_init_complete     = false;
bool   _audio_init_called     = false;
bool   _audio_init_complete   = false;
bool   _usb_init_called       = false;
bool   _usb_init_complete     = false;
bool   _imu_init_called       = false;
bool   _imu_init_complete     = false;
bool   _tof_init_called       = false;
bool   _tof_init_complete     = false;
bool   _baro_init_called      = false;
bool   _baro_init_complete    = false;
bool   _uv_init_called        = false;
bool   _uv_init_complete      = false;
bool   _touch_init_called     = false;
bool   _touch_init_complete   = false;
bool   _grideye_init_called   = false;
bool   _grideye_init_complete = false;

bool   _boot_complete_wait    = false;

uint16_t serial_timeout = 0;




uAppBoot::uAppBoot() : uApp("Boot", (Image*) &display) {}

uAppBoot::~uAppBoot() {}


/*******************************************************************************
* Lifecycle callbacks from the super class
*******************************************************************************/
/**
* Called by superclass on activation of lifecycle. This function should prepare
*   the class as if it were freshly instantiated. Redraw will not happen until
*   the tick following this function returning 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppBoot::_lc_on_preinit() {
  int8_t ret = 1;
  for (uint8_t i = 0; i < 100; i++) {
    // Collect a baseline for use in tuning display intensity.
    // TODO: Data should come in passively as a stream, and not require this.
    graph_array_ana_light.feedFilter(analogRead(ANA_LIGHT_PIN) / 1024.0);
  }
  display.init(&spi0);
  _display_init_called = true;
  return ret;
}


/**
* Called by superclass to perform the first draw. Input will be processed and
*   the display redrawn after this function returns.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppBoot::_lc_on_active() {
  int8_t ret = 0;
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppBoot::_lc_on_teardown() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppBoot::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppBoot::_process_user_input() {
  int8_t ret = 1;

  if (_slider_current != _slider_pending) {
    _slider_current = _slider_pending;
  }
  if (_buttons_current != _buttons_pending) {
    if (_boot_complete_wait) {
      // Interpret a cancel press as a return to APP_SELECT.
      uApp::setAppActive(AppID::APP_SELECT);
      ret = -1;
    }
    _buttons_current = _buttons_pending;
  }
  return ret;
}



/*
* Draws the app.
*/
void uAppBoot::_redraw_window() {
  float percent_setup = 0.0;
  char* init_step_str = (char*) "";
  int cursor_height   = 26;
  if (_boot_complete_wait) {
    FB->setCursor(4, 14);
    FB->writeString("Boot complete");
    touch->setMode(SX8634OpMode::ACTIVE);
    return;
  }

  if (!_display_init_complete) {
    _display_init_complete = display.enabled();
    if (_display_init_complete) {
      FB->fill(BLACK);
      FB->setTextColor(WHITE);
      FB->setCursor(14, 0);
      FB->setTextSize(1);
      FB->writeString("Motherflux0r\n");
      FB->setTextSize(0);
      draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, true, false, percent_setup);
    }
    return;
  }
  percent_setup += 0.08;

  if (!_touch_init_complete) {
    if (!_touch_init_called) {
      init_step_str = (char*) "Touchpad        ";
      draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
      FB->setTextColor(WHITE);
      FB->setCursor(4, 14);
      FB->writeString(init_step_str);
      _touch_init_called = (0 == touch->init(&Wire));
    }
    _touch_init_complete = touch->deviceFound();
    if (_touch_init_complete) {
      touch->poll();
      touch->setLongpress(800, 0);   // 800ms is a long-press. No rep.
    }
    return;
  }
  percent_setup += 0.08;

  if (!_usb_init_complete) {
    if (!_usb_init_called) {
      init_step_str = (char*) "USB        ";
      draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
      FB->setTextColor(WHITE);
      FB->setCursor(4, 14);
      FB->writeString(init_step_str);
      _usb_init_called = true;
    }
    if (!Serial && (100 > serial_timeout)) {
      serial_timeout++;
    }
    else {
      if (Serial) {
        while (Serial.available()) {
          Serial.read();
        }
        //FB->writeString("Serial\n");
      }
      _usb_init_complete = true;
    }
    return;
  }
  percent_setup += 0.08;

  if (!_grideye_init_complete) {
    _grideye_init_complete = grideye.initialized();
    if (!_grideye_init_called) {
      init_step_str = (char*) "GridEye         ";
      draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
      FB->setTextColor(WHITE);
      FB->setCursor(4, 14);
      FB->writeString(init_step_str);
      _grideye_init_called = true;
      _grideye_init_complete = (0 != grideye.init(&i2c1));  // Abort init wait if it failed.
    }
    return;
  }
  percent_setup += 0.08;

//  if (!_uv_init_complete) {
//    if (!_uv_init_called) {
//      init_step_str = (char*) "UVI      ";
//      draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
//      FB->setTextColor(WHITE);
//      FB->setCursor(4, 14);
//      FB->writeString(init_step_str);
//      _uv_init_called = true; //(VEML6075_ERROR_SUCCESS == uv.init(&Wire1));
//      uv.init(&Wire1);
//    }
//    _uv_init_complete = uv.initialized();
//    return;
//  }
//  percent_setup += 0.08;

  if (!_lux_init_complete) {
    if (!_lux_init_called) {
      init_step_str = (char*) "Lux  ";
      draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
      FB->setTextColor(WHITE);
      FB->setCursor(4, 14);
      FB->writeString(init_step_str);
      tsl2561.assignBusInstance(&i2c1);
      _lux_init_called = true; //(0 == tsl2561.init(&Wire1));
      tsl2561.init();
    }
    if (tsl2561.initialized()) {
      _lux_init_complete = true;
      tsl2561.integrationTime(TSLIntegrationTime::MS_101);
    }
    return;
  }
  percent_setup += 0.08;

  //if (!_baro_init_complete) {
  //  _baro_init_complete = baro.initialized();
  //  if (!_baro_init_called) {
  //    init_step_str = (char*) "Baro    ";
  //    draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  //    FB->setTextColor(WHITE);
  //    FB->setCursor(4, 14);
  //    FB->writeString(init_step_str);
  //    _baro_init_called = true;
  //    _baro_init_complete = (0 != baro.init(&Wire1));  // Abort init wait if it failed.
  //  }
  //  return;
  //}
  //percent_setup += 0.08;

  //if (!_mag_init_complete) {
  //  if (!_mag_init_called) {
  //    init_step_str = (char*) "Magnetometer    ";
  //    draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
  //    FB->setTextColor(WHITE);
  //    FB->setCursor(4, 14);
  //    FB->writeString(init_step_str);
  //    magneto.attachPipe(&mag_conv);   // Connect the driver to its pipeline.
  //    _mag_init_called = (0 == magneto.init(&i2c1, &spi0));
  //  }
  //  _mag_init_complete = grideye.initialized();
  //  if (_mag_init_complete) {
  //    // TODO: This is just to prod the compass into returning a complete
  //    //   dataset. It's bogus until there is an IMU.
  //    Vector3f gravity(0.0, 0.0, 1.0);
  //    Vector3f gravity_err(0.002, 0.002, 0.002);
  //    compass.pushVector(SpatialSense::ACC, &gravity, &gravity_err);   // Set gravity, initially.
  //  }
  //  return;
  //}
  //percent_setup += 0.08;

  if (!_audio_init_complete) {
    if (!_audio_init_called) {
      _audio_init_called = true;
      _audio_init_complete = true;
      sineL.amplitude(1.0);
      sineL.frequency(440);
      sineL.phase(0);
      sineR.amplitude(0.8);
      sineR.frequency(660);
      sineR.phase(0);
      mixerFFT.gain(0, mix_queueL_to_fft);
      mixerFFT.gain(1, mix_queueR_to_fft);
      mixerFFT.gain(2, mix_noise_to_fft);
      mixerFFT.gain(3, 0.0);
      mixerL.gain(0, mix_queue_to_line);
      mixerL.gain(1, mix_noise_to_line);
      mixerL.gain(2, mix_synth_to_line);
      mixerL.gain(3, 0.0);
      mixerR.gain(0, mix_queue_to_line);
      mixerR.gain(1, mix_noise_to_line);
      mixerR.gain(2, mix_synth_to_line);
      mixerR.gain(3, 0.0);
      pinkNoise.amplitude(volume_pink_noise);
      ampL.gain(volume_left_output);
      ampR.gain(volume_right_output);
      init_step_str = (char*) "Audio           ";
      draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
      FB->setTextColor(WHITE);
      FB->setCursor(4, 14);
      FB->writeString(init_step_str);
    }
    return;
  }
  percent_setup += 0.08;

//  if (!_imu_init_complete) {
//    if (!_imu_init_called) {
//      init_step_str = (char*) "Inertial        ";
//      draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
//      FB->setTextColor(WHITE);
//      FB->setCursor(4, 14);
//      FB->writeString(init_step_str);
//      _imu_init_called = (ICM_20948_Stat_Ok == imu.begin(IMU_CS_PIN, SPI, 6000000));
//    }
//    _imu_init_complete = (ICM_20948_Stat_Ok == imu.checkID());
//    if (_imu_init_complete) {
//      imu.swReset();
//      imu.sleep(false);
//      imu.lowPower(false);
//      imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr | ICM_20948_Internal_Mag), ICM_20948_Sample_Mode_Continuous);
//      ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
//      myFSS.a = gpm2;         // gpm2, gpm4, gpm8, gpm16
//      myFSS.g = dps250;       // dps250, dps500, dps1000, dps2000
//      imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
//      // Set up Digital Low-Pass Filter configuration
//      ICM_20948_dlpcfg_t myDLPcfg;      // Similar to FSS, this uses a configuration structure for the desired sensors
//      myDLPcfg.a = acc_d473bw_n499bw;   // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
//                                        // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
//                                        // acc_d111bw4_n136bw
//                                        // acc_d50bw4_n68bw8
//                                        // acc_d23bw9_n34bw4
//                                        // acc_d11bw5_n17bw
//                                        // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
//                                        // acc_d473bw_n499bw
//      myDLPcfg.g = gyr_d361bw4_n376bw5;    // gyr_d196bw6_n229bw8, gyr_d151bw8_n187bw6, gyr_d119bw5_n154bw3, gyr_d51bw2_n73bw3, gyr_d23bw9_n35bw9, gyr_d11bw6_n17bw8, gyr_d5bw7_n8bw9, gyr_d361bw4_n376bw5
//      imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
//      // Choose whether or not to use DLPF
//      ICM_20948_Status_e accDLPEnableStat = imu.enableDLPF((ICM_20948_Internal_Gyr | ICM_20948_Internal_Acc), true);
//      if (255 != IMU_IRQ_PIN) {
//        pinMode(IMU_IRQ_PIN, GPIOMode::INPUT);
//        //imu.cfgIntActiveLow(true);
//        //imu.cfgIntOpenDrain(false);
//        //imu.cfgIntLatch(true);          // IRQ is a 50us pulse.
//        //imu.intEnableRawDataReady(true);
//        //setPinFxn(IMU_IRQ_PIN, IRQCondition::FALLING, imu_isr_fxn);
//      }
//    }
//    return;
//  }
//  percent_setup += 0.08;

//  if (!_tof_init_complete) {
//    if (!_tof_init_called) {
//      init_step_str = (char*) "ToF         ";
//      draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, percent_setup);
//      FB->setTextColor(WHITE);
//      FB->setCursor(4, 14);
//      FB->writeString(init_step_str);
//      tof.setTimeout(500);
//      _tof_init_called = (tof.init(&Wire1));
//      _tof_init_complete = _tof_init_called;
//    }
//    if (_tof_init_complete) {
//      tof.startContinuous(100);
//    }
//    return;
//  }
  draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, false, false, 1.0);
  _boot_complete_wait = true;
  return;
}
