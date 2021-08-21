#include <CppPotpourri.h>
#include <SensorFilter.h>
#include <SPIAdapter.h>
#include <I2CAdapter.h>

#include <Audio.h>

#include "Motherflux0r.h"
#include "uApp.h"
#include "SensorGlue.h"

extern SSD13xx display;
extern SX8634* touch;

extern SPIAdapter spi0;
extern I2CAdapter i2c0;
extern I2CAdapter i2c1;

extern ManuvrPMU pmu;

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

#define UAPP_BOOT_FLAG_INIT_DISPLAY         0x00000001
#define UAPP_BOOT_FLAG_INIT_LUX             0x00000002
#define UAPP_BOOT_FLAG_INIT_MAG_GPIO        0x00000004
#define UAPP_BOOT_FLAG_INIT_BARO            0x00000008
#define UAPP_BOOT_FLAG_INIT_GRIDEYE         0x00000010
#define UAPP_BOOT_FLAG_INIT_AUDIO           0x00000020
#define UAPP_BOOT_FLAG_INIT_TOF             0x00000040
#define UAPP_BOOT_FLAG_INIT_UV              0x00000080
#define UAPP_BOOT_FLAG_INIT_TOUCH           0x00000100
#define UAPP_BOOT_FLAG_INIT_IMU             0x00000200
#define UAPP_BOOT_FLAG_INIT_USB             0x00000400
#define UAPP_BOOT_FLAG_INIT_PMU_CHARGER     0x00000800
#define UAPP_BOOT_FLAG_INIT_PMU_GUAGE       0x00001000
#define UAPP_BOOT_FLAG_INIT_GPS             0x00002000
#define UAPP_BOOT_FLAG_INIT_MAG_ADC         0x00004000
#define UAPP_BOOT_FLAG_INIT_STORAGE         0x00008000
#define UAPP_BOOT_FLAG_INIT_CONF_LOADED     0x00010000

#define UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE   0x80000000

typedef struct {
  const char* const str;
  const uint32_t flag_mask;
} UAppInitPoint;

// Items on the init list should be in order of desired initialization.
const UAppInitPoint INIT_LIST[] = {
  {"Display",          UAPP_BOOT_FLAG_INIT_DISPLAY       },
  {"Touchpad",         UAPP_BOOT_FLAG_INIT_MAG_GPIO      },
  {"NV Storage",       UAPP_BOOT_FLAG_INIT_STORAGE       },
  {"Conf Load",        UAPP_BOOT_FLAG_INIT_CONF_LOADED   },
  {"USB",              UAPP_BOOT_FLAG_INIT_LUX           },
  {"Barometer",        UAPP_BOOT_FLAG_INIT_BARO          },
  {"GridEye",          UAPP_BOOT_FLAG_INIT_GRIDEYE       },
  {"Audio",            UAPP_BOOT_FLAG_INIT_AUDIO         },
  {"ToF",              UAPP_BOOT_FLAG_INIT_TOF           },
  {"UVI",              UAPP_BOOT_FLAG_INIT_UV            },
  {"Touchpad",         UAPP_BOOT_FLAG_INIT_TOUCH         },
  {"Inertial",         UAPP_BOOT_FLAG_INIT_IMU           },
  {"USB",              UAPP_BOOT_FLAG_INIT_USB           },
  {"PMU Charger",      UAPP_BOOT_FLAG_INIT_PMU_CHARGER   },
  {"PMU Guage",        UAPP_BOOT_FLAG_INIT_PMU_GUAGE     },
  {"GPS",              UAPP_BOOT_FLAG_INIT_GPS           },
  {"Magnetometer ADC", UAPP_BOOT_FLAG_INIT_MAG_ADC       },
  {"Boot Complete",    UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE }
};

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
    if (_init_done_flags.value(UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE)) {
      // Interpret a button press as exiting APP_BOOT.
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
  const uint8_t INIT_LIST_LEN = sizeof(INIT_LIST) / sizeof(UAppInitPoint);
  const float   INIT_LIST_PERCENT = 100.0 / (float) INIT_LIST_LEN;
  const uint8_t BLOCK_WIDTH  = (96/INIT_LIST_LEN);  //INIT_LIST_PERCENT * 96;
  const uint8_t BLOCK_HEIGHT = 11;
  bool     continue_looping = display.enabled();   // Don't loop if no display.
  uint8_t  i = 0;

  while (continue_looping & (i < INIT_LIST_LEN)) {
    if (!_init_sent_flags.value(INIT_LIST[i].flag_mask)) {
      // This item has not seen an init call succeed.
      bool ret_local = false;
      switch (INIT_LIST[i].flag_mask) {
        case UAPP_BOOT_FLAG_INIT_DISPLAY:
          FB->fill(BLACK);
          FB->setTextColor(WHITE, BLACK);
          FB->setCursor(14, 0);
          FB->setTextSize(1);
          FB->writeString("Motherflux0r");
          FB->setTextSize(0);
          FB->fillRect(0, 11, BLOCK_WIDTH*INIT_LIST_LEN, BLOCK_HEIGHT, YELLOW);
          FB->fillRect(0, 24, BLOCK_WIDTH*INIT_LIST_LEN, BLOCK_HEIGHT, YELLOW);
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_LUX:
          ret_local = (0 == tsl2561.init());
          break;
        case UAPP_BOOT_FLAG_INIT_MAG_GPIO:
          mag_filter.init();
          magneto.attachPipe(&mag_conv);   // Connect the driver to its pipeline.
          ret_local = (0 == magneto.init(&i2c1, &spi0));
          break;
        case UAPP_BOOT_FLAG_INIT_BARO:
          ret_local = (0 == baro.init());
          break;
        case UAPP_BOOT_FLAG_INIT_GRIDEYE:
          ret_local = (0 == grideye.init(&i2c1));
          break;
        case UAPP_BOOT_FLAG_INIT_AUDIO:
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
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_TOF:
          // tof.setTimeout(500);
          // ret_local = (0 == tof.init());
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_UV:
          ret_local = (0 == uv.init());
          break;
        case UAPP_BOOT_FLAG_INIT_TOUCH:
          ret_local = (0 == touch->reset());
          break;
        case UAPP_BOOT_FLAG_INIT_IMU:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_USB:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_PMU_CHARGER:
          ret_local = (0 == pmu.init(&i2c0));
          break;
        case UAPP_BOOT_FLAG_INIT_PMU_GUAGE:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_GPS:
          ret_local = (0 == gps.init());
          break;
        case UAPP_BOOT_FLAG_INIT_MAG_ADC:
          if (_init_done_flags.value(UAPP_BOOT_FLAG_INIT_MAG_GPIO)) {
            ret_local = magneto.power();
          }
          break;
        case UAPP_BOOT_FLAG_INIT_STORAGE:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_CONF_LOADED:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE:
          ret_local = true;
          break;
        default:  return;  // TODO: Failure
      }
      if (ret_local) {
        // If calling the init sequence succeeded, mark it as having been done.
        //draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, (0 == i), false, (INIT_LIST_PERCENT*(i+1)));
        display.fillRect(BLOCK_WIDTH*i, 11, BLOCK_WIDTH, BLOCK_HEIGHT, BLUE);
        _init_sent_flags.set(INIT_LIST[i].flag_mask);
        continue_looping = false;
        _last_init_sent = millis();
      }
    }
    else {
      display.fillRect(BLOCK_WIDTH*i, 11, BLOCK_WIDTH, BLOCK_HEIGHT, GREEN);
    }

    if (!_init_done_flags.value(INIT_LIST[i].flag_mask)) {
      // This item has not seen an init succeed.
      bool ret_local = false;
      switch (INIT_LIST[i].flag_mask) {
        case UAPP_BOOT_FLAG_INIT_DISPLAY:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_LUX:
          ret_local = tsl2561.initialized();
          if (ret_local) {
            tsl2561.integrationTime(TSLIntegrationTime::MS_101);
          }
          break;
        case UAPP_BOOT_FLAG_INIT_MAG_GPIO:
          if (magneto.gpio.initialized()) {
            // TODO: This is just to prod the compass into returning a complete
            //   dataset. It's bogus until there is an IMU.
            Vector3f gravity(0.0, 0.0, 1.0);
            Vector3f gravity_err(0.002, 0.002, 0.002);
            compass.pushVector(SpatialSense::ACC, &gravity, &gravity_err);   // Set gravity, initially.
            ret_local = magneto.power(true);
          }
          break;
        case UAPP_BOOT_FLAG_INIT_BARO:
          ret_local = baro.initialized();
          break;
        case UAPP_BOOT_FLAG_INIT_GRIDEYE:
          ret_local = grideye.initialized();
          break;
        case UAPP_BOOT_FLAG_INIT_AUDIO:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_TOF:
          ret_local = true;
          if (ret_local) {
            // tof.startContinuous(100);
          }
          break;
        case UAPP_BOOT_FLAG_INIT_UV:
          if (uv.initialized()) {
            //ret_local = (VEML6075Err::SUCCESS == uv.enabled(true));
            ret_local = (VEML6075Err::SUCCESS == uv.setIntegrationTime(VEML6075IntTime::IT_100MS));
          }
          break;
        case UAPP_BOOT_FLAG_INIT_TOUCH:
          if (touch->devFound()) {
            touch->poll();
            ret_local = touch->deviceReady();
            if (ret_local) {
              touch->setLongpress(800, 0);   // 800ms is a long-press. No rep.
              touch->setMode(SX8634OpMode::ACTIVE);
            }
          }
          break;
        case UAPP_BOOT_FLAG_INIT_IMU:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_USB:
          if (!Serial && (100 > serial_timeout)) {
            serial_timeout++;
          }
          else {
            if (Serial) {
              // Drain any leading characters.
              while (Serial.available()) {  Serial.read();   }
            }
            ret_local = true;
          }
          break;
        case UAPP_BOOT_FLAG_INIT_PMU_CHARGER:
          //ret_local = pmu.bq24155.initComplete();
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_PMU_GUAGE:
          //ret_local = pmu.ltc294x.initComplete();
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_GPS:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_MAG_ADC:
          //ret_local = magneto.configured();
          ret_local = magneto.power();
          break;
        case UAPP_BOOT_FLAG_INIT_STORAGE:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_CONF_LOADED:
          ret_local = true;
          break;
        case UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE:
          ret_local = (_init_sent_flags.raw == (_init_done_flags.raw | UAPP_BOOT_FLAG_INIT_BOOT_COMPLETE));
          break;
        default:  return;  // TODO: Failure
      }
      if (ret_local) {
        // If init succeeded, mark it as complete.
        _init_done_flags.set(INIT_LIST[i].flag_mask);
        //draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, (0 == i), false, (INIT_LIST_PERCENT*(i+1)));
        display.fillRect(BLOCK_WIDTH*i, 24, BLOCK_WIDTH, BLOCK_HEIGHT, BLUE);
        const uint UNFILLED_STR_LEN = strlen(INIT_LIST[i].str);
        char filled_list_str[17] = {0, };
        for (uint8_t stri = 0; stri < sizeof(filled_list_str); stri++) {
          filled_list_str[stri] = (stri < UNFILLED_STR_LEN) ? *(INIT_LIST[i].str + stri) : ' ';
        }
        FB->setTextColor(WHITE, BLACK);
        FB->setCursor(0, 36);
        FB->writeString(filled_list_str);
        continue_looping = false;
      }
    }
    else {
      display.fillRect(BLOCK_WIDTH*i, 24, BLOCK_WIDTH, BLOCK_HEIGHT, GREEN);
    }
    i++;
  }

  if (wrap_accounted_delta(_last_init_sent, millis()) >= UAPP_BOOT_INIT_TIMEOUT) {
    for (uint8_t n = 0; n < INIT_LIST_LEN; n++) {
      if (!_init_sent_flags.value(INIT_LIST[n].flag_mask)) {
        _init_sent_flags.set(INIT_LIST[n].flag_mask);
        display.fillRect(BLOCK_WIDTH*n, 11, BLOCK_WIDTH, BLOCK_HEIGHT, RED);
      }
      if (!_init_done_flags.value(INIT_LIST[n].flag_mask)) {
        _init_done_flags.set(INIT_LIST[n].flag_mask);
        display.fillRect(BLOCK_WIDTH*n, 24, BLOCK_WIDTH, BLOCK_HEIGHT, RED);
      }
    }
  }
  return;
}

//  if (!_imu_init_complete) {
//    if (!_imu_init_called) {
//      init_step_str = (char*) "Inertial        ";
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
