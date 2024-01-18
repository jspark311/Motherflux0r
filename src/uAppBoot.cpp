#include "Motherflux0r.h"
#include "uApp.h"
#include "SensorGlue.h"


// Items on the boot checklist list, as shown to the user.
static const uint32_t INIT_LIST[] = {
  CHKLST_BOOT_INIT_DISPLAY,
  CHKLST_BOOT_INIT_UI,
  CHKLST_BOOT_INIT_SPI0,
  CHKLST_BOOT_INIT_I2C0,
  CHKLST_BOOT_INIT_I2C1,
  CHKLST_BOOT_INIT_MAG_GPIO,
  CHKLST_BOOT_INIT_STORAGE,
  CHKLST_BOOT_INIT_CONF_LOAD,
  CHKLST_BOOT_AUDIO_STACK,
  CHKLST_BOOT_INIT_TOUCH_READY,
  CHKLST_BOOT_INIT_CONSOLE,
  CHKLST_BOOT_INIT_PMU_CHARGER,
  CHKLST_BOOT_INIT_PMU_GUAGE,
  CHKLST_BOOT_INIT_GPS,
  CHKLST_BOOT_INIT_COMMS,
  CHKLST_BOOT_INIT_GPIO,
  CHKLST_BOOT_MASK_BOOT_COMPLETE
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
    if (checklist_boot.all_steps_have_passed(CHKLST_BOOT_MASK_BOOT_COMPLETE)) {
      // Interpret a button press as exiting APP_BOOT.
      uApp::setAppActive(AppID::APP_SELECT);
      ret = -1;
    }
    _buttons_current = _buttons_pending;
  }
  return ret;
}


static bool todo_remove_this = false;

/*
* Draws the app.
*/
void uAppBoot::_redraw_window() {
  const uint8_t INIT_LIST_LEN = sizeof(INIT_LIST) / sizeof(INIT_LIST[0]);
  const float   INIT_LIST_PERCENT = 100.0 / (float) INIT_LIST_LEN;
  const uint8_t BLOCK_WIDTH  = (96/INIT_LIST_LEN);  //INIT_LIST_PERCENT * 96;
  const uint8_t BLOCK_HEIGHT = 11;

  // Don't render if no display.
  if (checklist_boot.all_steps_have_passed(CHKLST_BOOT_INIT_DISPLAY)) {
    if (!todo_remove_this) {
      todo_remove_this = true;
      FB->fill(BLACK);
      FB->setTextColor(WHITE, BLACK);
      FB->setCursor(14, 0);
      FB->setTextSize(1);
      FB->writeString("Motherflux0r");
      FB->setTextSize(0);
      FB->fillRect(0, 11, BLOCK_WIDTH*INIT_LIST_LEN, BLOCK_HEIGHT, YELLOW);
      FB->fillRect(0, 24, BLOCK_WIDTH*INIT_LIST_LEN, BLOCK_HEIGHT, YELLOW);
    }
  }

  uint8_t i = 0;
  bool    continue_looping = true;

  while (continue_looping & (i < INIT_LIST_LEN)) {
    bool dispatched = (checklist_boot.all_steps_dispatched(INIT_LIST[i]));
    bool complete   = (checklist_boot.all_steps_have_passed(INIT_LIST[i]));
    bool failed     = (checklist_boot.failed_steps(true) & INIT_LIST[i]);

    if (dispatched) {
      if (failed) {
        display.fillRect(BLOCK_WIDTH*i, 11, BLOCK_WIDTH, BLOCK_HEIGHT, RED);
      }
      else if (complete) {
        display.fillRect(BLOCK_WIDTH*i, 11, BLOCK_WIDTH, BLOCK_HEIGHT, GREEN);
      }
      else {
        //draw_progress_bar_horizontal(0, 11, 95, 12, GREEN, (0 == i), false, (INIT_LIST_PERCENT*(i+1)));
        display.fillRect(BLOCK_WIDTH*i, 11, BLOCK_WIDTH, BLOCK_HEIGHT, BLUE);
        continue_looping = false;
      }
    }
    i++;
  }
  // if (millis_since(_last_init_sent) >= UAPP_BOOT_INIT_TIMEOUT) {
  //   for (uint8_t n = 0; n < INIT_LIST_LEN; n++) {
  //     if (!_init_sent_flags.value(INIT_LIST[n])) {
  //       _init_sent_flags.set(INIT_LIST[n]);
  //       display.fillRect(BLOCK_WIDTH*n, 11, BLOCK_WIDTH, BLOCK_HEIGHT, RED);
  //     }
  //     if (!_init_done_flags.value(INIT_LIST[n])) {
  //       _init_done_flags.set(INIT_LIST[n]);
  //       display.fillRect(BLOCK_WIDTH*n, 24, BLOCK_WIDTH, BLOCK_HEIGHT, RED);
  //     }
  //   }
  // }
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
