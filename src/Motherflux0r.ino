#include "Motherflux0r.h"

#include <math.h>
#include <Arduino.h>
#include <StringBuilder.h>
#include <SensorFilter.h>
#include <ParsingConsole.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <SX8634.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include "VEML6075.h"
#include <ICM_20948.h>
#include "BME280.h"
#include "AMG88xx.h"
#include "DRV425.h"
#include "TSL2561.h"
#include "TMP102.h"


/*
CHANGELOG:
===========================================================================
1.0:
---------------------------------------------------------------------------
*/

/*******************************************************************************
* Global constants
*******************************************************************************/

/* Audio related */
static const uint16_t BIN_INDICIES[] = {
  0,   0,   1,   1,   2,   2,   3,   3,   4,   4,   5,   5,   6,   6,   7,   7,
  8,   8,   9,   9,   10,  10,  11,  11,  12,  12,  13,  13,  14,  14,  15,  15,
  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  24,
  25,  26,  27,  28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,
  41,  42,  43,  44,  45,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,
  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  72,
  73,  74,  75,  76,  77,  78,  79,  81,  82,  84,  85,  87,  88,  90,  91,  93,
  94,  96,  97,  99,  100, 102, 103, 105, 106, 108, 109, 111, 112, 114, 115, 117,
  118, 120, 121, 123, 124, 126, 127, 129, 130, 132, 133, 136, 137, 140, 141, 144,
  145, 148, 149, 152, 153, 156, 157, 160, 161, 164, 165, 168, 169, 172, 173, 176,
  177, 180, 181, 184, 185, 188, 189, 193, 194, 198, 199, 203, 204, 208, 209, 213,
  214, 218, 219, 223, 224, 228, 229, 233, 234, 238, 239, 243, 244, 249, 250, 255
};

// Thermopile constants
const float THERM_TEMP_MAX = 150.0;
const float THERM_TEMP_MIN = 0.0;


/*******************************************************************************
* Globals
*******************************************************************************/
Adafruit_SSD1331 display = Adafruit_SSD1331(&SPI, DISPLAY_CS_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN);

/* Audio library... */
AudioSynthWaveformSine   sineL;          //xy=86.00000762939453,296.0000877380371
AudioSynthWaveformSine   sineR;          //xy=86.00000762939453,329.00012397766113
AudioSynthNoisePink      pinkNoise;          //xy=86.0000114440918,364.0000858306885
//AudioInputAnalog         light_adc(ANA_LIGHT_PIN);  //xy=88.00000381469727,389.0000171661377
//AudioInputAnalog         mic_adc(MIC_ANA_PIN);      //xy=89.00000762939453,422.4285945892334
AudioPlayQueue           queueL;         //xy=97.0000228881836,200.71429824829102
AudioPlayQueue           queueR;         //xy=97.0000228881836,236.00013542175293
AudioMixer4              mixerL;         //xy=329.00008392333984,224.71430587768555
AudioMixer4              mixerR;         //xy=330.00008392333984,290.7142753601074
AudioMixer4              mixerFFT;         //xy=338.0000457763672,357.0000801086426
AudioAnalyzeFFT256       fft256_1;       //xy=491.00001525878906,357.00015449523926
AudioAmplifier           ampR;           //xy=468.0000534057617,285.0000114440918
AudioAmplifier           ampL;           //xy=469.0000190734863,233.0000057220459
AudioOutputI2S           i2s_dac;           //xy=495.00001525878906,269.0000629425049

// AudioConnection          patchCord1(mic_adc, 0, mixerFFT, 3);
// AudioConnection          patchCord2(mic_adc, 0, mixerR, 3);
// AudioConnection          patchCord3(mic_adc, 0, mixerL, 3);
AudioConnection          patchCord4(sineL, 0, mixerFFT, 0);
AudioConnection          patchCord5(sineL, 0, mixerL, 1);
AudioConnection          patchCord6(sineR, 0, mixerR, 1);
AudioConnection          patchCord7(pinkNoise, 0, mixerFFT, 1);
// AudioConnection          patchCord8(light_adc, 0, mixerFFT, 2);
// AudioConnection          patchCord9(light_adc, 0, mixerR, 2);
// AudioConnection          patchCord10(light_adc, 0, mixerL, 2);
// AudioConnection          patchCord11(queueL, 0, mixerL, 0);
// AudioConnection          patchCord12(queueR, 0, mixerR, 0);
AudioConnection          patchCord13(mixerL, ampL);
AudioConnection          patchCord14(mixerR, ampR);
AudioConnection          patchCord15(mixerFFT, fft256_1);
AudioConnection          patchCord16(ampR, 0, i2s_dac, 1);
AudioConnection          patchCord17(ampL, 0, i2s_dac, 0);


uint8_t fft_bars_shown[96];


BME280Settings baro_settings(
  0x76,
  BME280OSR::X1,
  BME280OSR::X1,
  BME280OSR::X1,
  BME280Mode::Forced,
  BME280StandbyTime::StandbyTime_1000ms,
  BME280Filter::Off
);

/* Touch board */
static const SX8634Opts _touch_opts(
  SX8634_DEFAULT_I2C_ADDR,  // i2c address
  TOUCH_RESET_PIN,          // Reset pin. Output. Active low.
  TOUCH_IRQ_PIN             // IRQ pin. Input. Active low. Needs pullup.
);
static SX8634* touch = nullptr;


/* Sensor representations... */
DRV425 magnetometer(DRV425_ADC_IRQ_PIN, DRV425_GPIO_IRQ_PIN, 255, DRV425_CS_PIN);   // No GPIO reset pin.
TMP102 tmp102(0x49, 255);    // No connection to the alert pin.
GridEYE grideye(0x69, AMG8866_IRQ_PIN);
VEML6075 uv;
ICM_20948_SPI imu;
TSL2561 tsl2561(0x39, TSL2561_IRQ_PIN);
BME280I2C baro(baro_settings);

/* Immediate data... */
static Vector3f64 grav;       // Gravity vector from the IMU.
static Vector3f64 acc_vect;   // Acceleration vector from the IMU.
static Vector3f64 gyr_vect;   // Gyroscopic vector from the IMU.
static Vector3f64 mag_vect0;  // Magnetism vector from the IMU.
static Vector3f64 mag_vect1;  // Magnetism vector from the DRV425 complex.
static float    temperature       = 0.0;    // TMP102
static float    altitude          = 0.0;    // BME280
static float    dew_point         = 0.0;    // BME280
static float    sea_level         = 0.0;    // BME280
static float    battery_voltage   = 0.0;    //
static float    therm_pixels[64];
static float    therm_field_min   = THERM_TEMP_MAX;
static float    therm_field_max   = THERM_TEMP_MIN;
static float    therm_field_stdev = 0.0;
static double   therm_field_sum   = 0.0;

/* Data buffers for sensors. */
static SensorFilter<float> graph_array_pressure(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_humidity(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_air_temp(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_psu_temp(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_uva(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_uvb(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_uvi(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_ana_light(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_visible(FilteringStrategy::RAW, 96, 0);
static SensorFilter<float> graph_array_therm_mean(FilteringStrategy::RAW, 96, 0);

/* Cheeseball async support stuff. */
static uint8_t  update_disp_rate  = 30;     // Update in Hz for the display

static uint32_t boot_time         = 0;      // millis() at boot.
static uint32_t config_time       = 0;      // millis() at end of setup().
static uint32_t off_time_vib      = 0;      // millis() when vibrator should be disabled.
static uint32_t off_time_led_r    = 0;      // millis() when LED_R should be disabled.
static uint32_t off_time_led_g    = 0;      // millis() when LED_G should be disabled.
static uint32_t off_time_led_b    = 0;      // millis() when LED_B should be disabled.
static uint32_t off_time_display  = 0;      // millis() when the display should be blanked.
static uint32_t last_interaction  = 0;      // millis() when the user last interacted.
static uint32_t disp_update_last  = 0;      // millis() when the display last updated.
static uint32_t disp_update_next  = 0;      // millis() when the display next updates.

/* Console junk... */
ParsingConsole console(128);
static const TCode arg_list_0[]       = {TCode::NONE};
static const TCode arg_list_1_str[]   = {TCode::STR,   TCode::NONE};
static const TCode arg_list_1_uint[]  = {TCode::UINT,  TCode::NONE};
static const TCode arg_list_1_float[] = {TCode::FLOAT, TCode::NONE};
static const TCode arg_list_2_uint[]  = {TCode::UINT,  TCode::UINT,  TCode::NONE};
static const TCode arg_list_3_uint[]  = {TCode::UINT,  TCode::UINT,  TCode::UINT,  TCode::NONE};
static const TCode arg_list_4_uuff[]  = {TCode::UINT,  TCode::UINT,  TCode::FLOAT, TCode::FLOAT, TCode::NONE};
static const TCode arg_list_4_float[] = {TCode::FLOAT, TCode::FLOAT, TCode::FLOAT, TCode::FLOAT, TCode::NONE};

/* Application tracking and interrupts... */
static AppID    active_app          = AppID::APP_SELECT;
static AppID    drawn_app           = AppID::META;
static AppID    app_page            = AppID::TRICORDER;
static AppID    app_previous        = AppID::APP_SELECT;
static bool     dirty_button        = false;
static bool     dirty_slider        = false;
static bool     imu_irq_fired       = false;


/*******************************************************************************
* ISRs
*******************************************************************************/
void imu_isr_fxn() {         imu_irq_fired = true;        }



/*******************************************************************************
* LED and vibrator control
* Only have enable functions since disable is done by timer in the main loop.
*******************************************************************************/
void ledOn(uint8_t idx, uint32_t duration, uint16_t intensity = 3500) {
  uint32_t* millis_ptr = nullptr;
  switch (idx) {
    case LED_R_PIN:
      analogWrite(LED_R_PIN, intensity);
      millis_ptr = &off_time_led_r;
      break;
    case LED_G_PIN:
      analogWrite(LED_G_PIN, intensity);
      millis_ptr = &off_time_led_g;
      break;
    case LED_B_PIN:
      analogWrite(LED_B_PIN, intensity);
      millis_ptr = &off_time_led_b;
      break;
    default:
      return;
  }
  *millis_ptr = millis() + duration;
}


void vibrateOn(uint32_t duration, uint16_t intensity = 4095) {
  analogWrite(VIBRATOR_PIN, intensity);
  off_time_vib = millis() + duration;
}


/*
* Taken from:
* https://forum.pjrc.com/threads/32677-Is-there-a-logarithmic-function-for-FFT-bin-selection-for-any-given-of-bands
*/
float FindE(int bands, int bins) {
  float increment=0.1, eTest, n;
  int b, count, d;

  for (eTest = 1; eTest < bins; eTest += increment) {     // Find E through brute force calculations
    count = 0;
    for (b = 0; b < bands; b++) {                         // Calculate full log values
      n = pow(eTest, b);
      d = int(n + 0.5);
      count += d;
    }
    if (count > bins) {     // We calculated over our last bin
      eTest -= increment;   // Revert back to previous calculation increment
      increment /= 10.0;    // Get a finer detailed calculation & increment a decimal point lower
    }
    else
      if (count == bins)    // We found the correct E
        return eTest;       // Return calculated E
    if (increment < 0.0000001)        // Ran out of calculations. Return previous E. Last bin will be lower than (bins-1)
      return (eTest - increment);
  }
  return 0;                 // Return error 0
}

void printFFTBins() {
  float e, n;
  int b, bands, bins, count=0, d;

  bands = 96;                             // Frequency bands; (Adjust to desired value)
  bins = 256;                             // FFT bins; (Adjust to desired value)

  e = FindE(bands, bins);                 // Find calculated E value
  if (e) {                                // If a value was returned continue
    Serial.printf("E = %4.4f\n", e);      // Print calculated E value
    for (b = 0; b < bands; b++) {         // Test and print the bins from the calculated E
      n = pow(e, b);
      d = int(n + 0.5);

      Serial.printf( "%4d ", count);      // Print low bin
      count += d - 1;
      Serial.printf( "%4d\n", count);     // Print high bin
      ++count;
    }
  }
  else
    Serial.println("Error\n");            // Error, something happened
}


/*******************************************************************************
* Display functions
*******************************************************************************/
/*
* Draws the basics of the UI.
* 96x64
*/
void redraw_app_window(const char* title, uint8_t pages, uint8_t active_page) {
  display.fillScreen(BLACK);
  display.setTextSize(0);
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  //display.setTextColor(MAGENTA, BLACK);
  display.print(title);
  //display.drawLine(0, 10, display.width()-1, 10, WHITE);
  //display.drawLine(0, display.height()-1, display.width()-1, display.height()-1, WHITE);
  //display.drawLine(0, 10, 0, display.height()-1, WHITE);
  //display.drawLine(display.width()-1, 10, display.width()-1, display.height()-1, WHITE);
  //display.drawRect(0, 14, display.width()-1, display.height()-15, WHITE);
  display.drawFastHLine(0, 9, display.width(), WHITE);
  //render_button_icon(2, 46, 0, WHITE);
  //render_button_icon(3, 55, 0, WHITE);
  app_previous = drawn_app;
  drawn_app = active_app;
}

/*
* Draws the meta app.
*/
void redraw_meta_window() {
  if (drawn_app != active_app) {
    redraw_app_window("Meta", 0, 0);
    display.setCursor(0, 11);
    display.setTextColor(WHITE);
    display.print("Firmware ");
    display.setTextColor(CYAN);
    display.println(TEST_PROG_VERSION);
    display.setTextColor(WHITE);
    display.println("Touch:  ");
    display.println("Uptime: ");
    display.println("Date:   ");
    //display.print("UV: ");
  }


  display.setTextColor(CYAN, BLACK);
  display.setCursor(47, 19);
  SX8634OpMode tmode = touch->operationalMode();
  display.print(touch->getModeStr(tmode));
  display.setCursor(47, 27);
  display.print(millis() - boot_time);
  //display.setCursor(47, 43);
  //display.setTextColor(uv.devFound() ? GREEN : RED, BLACK);
  //display.print("*");
  //display.setTextColor(uv.initialized() ? GREEN : RED, BLACK);
  //display.print("*");
  //display.setTextColor(uv.enabled() ? GREEN : RED, BLACK);
  //display.print("*");
  //display.print(uv.poll());

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the configurator app.
*/
void redraw_configurator_window() {
  if (drawn_app != active_app) {
    redraw_app_window("Configurator", 0, 0);
    display.setCursor(0, 11);
  }

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Unit will drive while daydreaming.
*/
void redraw_hot_standby_window() {
  if (drawn_app != active_app) {
    display.fillScreen(BLACK);
  }

  if (dirty_button) {
    if (touch->buttonStates() == 0x0028) {
      // Buttons 0 and 2 (and ONLY those buttons) must be
      //   pressed to turn the UI elements back on.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}

/*
* Unit will go to sleep at the wheel.
*/
void redraw_suspended_window() {
  if (drawn_app != active_app) {
    display.fillScreen(BLACK);
    // TODO: Put things into reset states.
    // TODO: Power off non-essential rails.
    // TODO: Scale back the CPU clock.
    // TODO: Set wake sources.
  }

  if (dirty_button) {
    if (touch->buttonStates() == 0x0050) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the data manager app.
*/
void redraw_data_mgmt_window() {
  if (drawn_app != active_app) {
    redraw_app_window("Data Manager", 0, 0);
  }

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the comms app.
*/
void redraw_comms_root_window() {
  if (drawn_app != active_app) {
    redraw_app_window("Comms", 0, 0);
  }

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the I2C app.
*/
void redraw_i2c_probe_window() {
  if (drawn_app != active_app) {
    redraw_app_window("I2C Scanner", 0, 0);
  }

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the touch test app.
*/
void redraw_touch_test_window() {
  if (drawn_app != active_app) {
    redraw_app_window("SX8634 diag", 0, 0);
    display.setCursor(0, 37);
    display.setTextColor(WHITE);
    display.print("Slider:");
    dirty_button = true;
    dirty_slider = true;
  }

  if (dirty_button) {
    const uint8_t x_coords[] = {37, 57, 77, 77, 57, 37};
    const uint8_t y_coords[] = {14, 14, 14, 34, 34, 34};
    for (uint8_t i = 0; i < 6; i++) {
      if (touch->buttonPressed(i)) {
        display.fillRoundRect(x_coords[i], y_coords[i], 18, 18, 4, RED);
        if ((0 == i) && (touch->sliderValue() == 0)) {
          // Interpret a cancel press as a return to APP_SELECT.
          active_app = AppID::APP_SELECT;
        }
      }
      else {
        display.fillRect(x_coords[i], y_coords[i], 18, 18, BLACK);
        display.drawRoundRect(x_coords[i], y_coords[i], 18, 18, 4, GREEN);
      }
    }
    display.setCursor(0, 29);
    display.fillRect(0, 29, 28, 8, BLACK);
    display.setTextColor(RED, BLACK);
    display.print(touch->buttonStates(), HEX);
    dirty_button = false;
  }
  if (dirty_slider) {
    uint16_t sval = 60 - touch->sliderValue();
    dirty_slider = false;
    //display.fillRect(0, 45, 28, 8, BLACK);
    display.setTextSize(0);
    display.setCursor(0, 45);
    display.setTextColor(WHITE, BLACK);
    display.print(sval);

    uint8_t slider_pix = 2 + (((1+sval) / 61.0f) * (display.width()-5));
    display.fillRect(0, 55, display.width()-1, 7, BLACK);
    display.drawRoundRect(0, 54, display.width(), 9, 3, GREEN);
    display.fillRect(slider_pix-1, 55, 3, 7, RED);
  }
}


void add_to_graph_data_array(float* arr, float x) {
  for (uint8_t i = 0; i < 95; i++) {
    *(arr + i) = *(arr + i + 1);
  }
  *(arr + 95) = x;
}




/*
* Draws the tricorder app.
*/
void redraw_tricorder_window() {
  const uint8_t PIXEL_SIZE  = 4;
  const uint8_t TEXT_OFFSET = (PIXEL_SIZE*8)+5;
  const float TEMP_RANGE = THERM_TEMP_MAX - THERM_TEMP_MIN;
  const float BINSIZE_T  = TEMP_RANGE / (PIXEL_SIZE * 8);  // Space of display gives scale size.

  if (drawn_app != active_app) {
    redraw_app_window("Tricorder", 0, 0);
    dirty_slider = true;
  }

  if (dirty_slider) {
    if (touch->sliderValue() <= 45) {
      redraw_app_window("Tricorder", 0, 0);
    }
    else {
      //display.fillRect(0, 11, display.width()-1, display.height()-12, BLACK);
      display.fillScreen(BLACK);
    }
    dirty_slider = false;
  }

  if (touch->sliderValue() <= 7) {
    // Baro
    if (graph_array_humidity.dirty()) {
      draw_graph_obj(
        0, 10, 96, 37, 0x03E0,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
        &graph_array_humidity
      );
      display.setTextSize(0);
      display.setCursor(0, 48);
      display.setTextColor(WHITE);
      display.print("Alt: ");
      display.setTextColor(GREEN, BLACK);
      display.print(altitude);
      display.println("m");
      display.setTextColor(WHITE);
      display.print("Humidity: ");
      display.setTextColor(0x03E0, BLACK);
      display.print(graph_array_humidity.value());
      display.println("%");
    }
  }
  else if (touch->sliderValue() <= 15) {
    // Baro
    if (graph_array_air_temp.dirty()) {
      draw_graph_obj(
        0, 10, 48, 37, 0x83D0,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
        &graph_array_air_temp
      );
    }
    if (graph_array_pressure.dirty()) {
      draw_graph_obj(
        48, 10, 48, 37, 0xFE00,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
        &graph_array_pressure
      );
    }
    display.setTextSize(0);
    display.setCursor(0, 48);
    display.setTextColor(WHITE);
    display.print("Pres: ");
    display.setTextColor(0xFE00, BLACK);
    display.print(graph_array_pressure.value());
    display.println("Pa");
    display.setTextColor(WHITE);
    display.print("Air Temp: ");
    display.setTextColor(0x83D0, BLACK);
    display.print(graph_array_air_temp.value());
    display.println("C");
  }
  else if (touch->sliderValue() <= 22) {
    // TSL2561
    if (graph_array_visible.dirty()) {
      draw_graph_obj(
        0, 10, 96, 45, 0xF100,
        true, touch->buttonPressed(1), touch->buttonPressed(4),
        &graph_array_visible
      );
      display.setTextSize(0);
      display.setCursor(0, 56);
      display.setTextColor(WHITE);
      display.print("Lux:  ");
      display.setTextColor(0xF100, BLACK);
      display.print(graph_array_visible.value());
    }
  }
  else if (touch->sliderValue() <= 30) {
    // Analog light sensor
    graph_array_ana_light.feedFilter(analogRead(ANA_LIGHT_PIN) / 1024.0);
    draw_graph_obj(
      0, 10, 96, 45, 0xFE00,
      true, touch->buttonPressed(1), touch->buttonPressed(4),
      &graph_array_ana_light
    );
    display.setTextSize(0);
    display.setCursor(0, 56);
    display.setTextColor(WHITE);
    display.print("Light:  ");
    display.setTextColor(GREEN, BLACK);
    display.print(graph_array_ana_light.value());
  }
  else if (touch->sliderValue() <= 37) {
    if (touch->buttonPressed(5)) {
      // UVI
      if (graph_array_uvi.dirty()) {
        draw_graph_obj(
          0, 10, 96, 45, 0xF81F,
          true, touch->buttonPressed(1), touch->buttonPressed(4),
          &graph_array_uvi
        );
        display.setTextSize(0);
        display.setCursor(0, 56);
        display.setTextColor(WHITE);
        display.print("UVI:  ");
        display.setTextColor(0xF81F, BLACK);
        display.print(graph_array_uvi.value());
      }
    }
    else {
      if (graph_array_uva.dirty()) {
        draw_graph_obj(
          0, 10, 48, 45, 0x781F,
          true, touch->buttonPressed(1), touch->buttonPressed(4),
          &graph_array_uva
        );
      }
      if (graph_array_uvb.dirty()) {
        draw_graph_obj(
          48, 10, 48, 45, 0xF80F,
          true, touch->buttonPressed(1), touch->buttonPressed(4),
          &graph_array_uvb
        );
      }
      display.setTextSize(0);
      display.setCursor(0, 56);
      display.setTextColor(WHITE);
      display.print("UVA/B: ");
      display.setTextColor(0x781F, BLACK);
      display.print(graph_array_uva.value());
      display.setTextColor(WHITE, BLACK);
      display.print(" / ");
      display.setTextColor(0xF80F, BLACK);
      display.print(graph_array_uvb.value());
    }
  }
  else if (touch->sliderValue() <= 45) {
    // IMU
    display.setCursor(0, 11);
    display.setTextColor(YELLOW, BLACK);
    display.print("IMU");
    //imu.accX();
    //imu.accY();
    //imu.accZ();
    //imu.gyrX();
    //imu.gyrY();
    //imu.gyrZ();
    //imu.magX();
    //imu.magY();
    //imu.magZ();
    //imu.temp();
  }
  else if (touch->sliderValue() <= 52) {
    display.setCursor(0, 11);
    display.setTextColor(YELLOW, BLACK);
    display.print("Magnetometer");
  }
  else {
    // Thermopile
    if (graph_array_therm_mean.dirty()) {
      const float MIDPOINT_T = therm_field_max - ((therm_field_max - therm_field_min) / 2);
      for (uint8_t i = 0; i < 64; i++) {
        uint x = (i & 0x07) * PIXEL_SIZE;
        uint y = (i >> 3) * PIXEL_SIZE;
        float pix_deviation = abs(MIDPOINT_T - therm_pixels[i]);
        uint8_t pix_intensity = BINSIZE_T * (pix_deviation / (therm_field_max - MIDPOINT_T));
        uint16_t color = (therm_pixels[i] <= MIDPOINT_T) ? pix_intensity : (pix_intensity << 11);
        display.fillRect(x, y, PIXEL_SIZE, PIXEL_SIZE, color);
      }
      display.setTextSize(0);
      display.setCursor(TEXT_OFFSET, 0);
      display.setTextColor(RED, BLACK);
      display.print(therm_field_max);

      display.setCursor(TEXT_OFFSET, 12);
      display.setTextColor(WHITE, BLACK);
      display.print("ABS (C)");

      display.setCursor(TEXT_OFFSET, (PIXEL_SIZE*8)-7);
      display.setTextColor(BLUE, BLACK);
      display.print(therm_field_min);

      display.setCursor(0, TEXT_OFFSET);
      display.setTextColor(WHITE);
      display.print("Mean:  ");
      display.setTextColor(RED, BLACK);
      display.println(graph_array_therm_mean.value());

      display.setTextColor(WHITE);
      display.print("Range: ");
      display.setTextColor(RED, BLACK);
      display.println(therm_field_max - therm_field_min);

      display.setTextColor(WHITE);
      display.print("STDEV: ");
      display.setTextColor(RED, BLACK);
      display.println(therm_field_stdev);
    }
  }

  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


/*
* Draws the app selection window.
*/
void redraw_app_select_window() {
  const uint8_t ICON_SIZE    = 32;
  const uint8_t TEXT_OFFSET = ICON_SIZE+11;

  if (drawn_app != active_app) {
    redraw_app_window("Select Function", 0, 0);
    dirty_slider = true;
  }

  if (dirty_slider) {
    display.fillRect(0, 11, display.width()-1, display.height()-12, BLACK);
    display.setCursor(0, TEXT_OFFSET);
    display.setTextColor(WHITE);
    display.print("Fxn: ");
    display.setTextColor(MAGENTA, BLACK);
    if (touch->sliderValue() <= 7) {
      display.println("Touch Diag");
      app_page = AppID::TOUCH_TEST;
    }
    else if (touch->sliderValue() <= 15) {
      display.println("Settings");
      app_page = AppID::CONFIGURATOR;
    }
    else if (touch->sliderValue() <= 22) {
      display.println("Data MGMT");
      app_page = AppID::DATA_MGMT;
    }
    else if (touch->sliderValue() <= 30) {
      display.println("Synth Box");
      app_page = AppID::SYNTH_BOX;
    }
    else if (touch->sliderValue() <= 37) {
      display.println("Comms");
      app_page = AppID::COMMS_TEST;
    }
    else if (touch->sliderValue() <= 45) {
      display.println("Meta");
      app_page = AppID::META;
    }
    else if (touch->sliderValue() <= 52) {
      display.println("I2C Scanner");
      app_page = AppID::I2C_SCANNER;
    }
    else {
      display.println("Tricorder");
      app_page = AppID::TRICORDER;
    }
    dirty_slider = false;
  }

  if (dirty_button) {
    if (touch->buttonPressed(2)) {
      active_app = app_page;  // This will cause the app to switch.
    }
    else if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a request to doze.
      display.fillScreen(BLACK);
      active_app = AppID::HOT_STANDBY;
    }
    dirty_button = false;
  }
}


/*
* Draws the FFT app.
*/
void redraw_fft_window() {
  const float SCALER_PIX = 64;
  const int SHOWN_DECAY  = 1;

  if (drawn_app != active_app) {
    redraw_app_window("FFT", 0, 0);
    //display.fillScreen(BLACK);
  }

  if (dirty_slider) {
    sineL.frequency(10+300*touch->sliderValue());
    sineR.frequency(610+300*touch->sliderValue());
    dirty_slider = false;
  }

  float fft_bins[96];
  for (uint8_t i = 0; i < 96; i++) {
    fft_bins[i] = fft256_1.read(BIN_INDICIES[i << 1], BIN_INDICIES[(i << 1) + 1]);
  }
  for (uint8_t i = 0; i < 96; i++) {
    uint8_t scaled_val = fft_bins[i] * SCALER_PIX;
    uint y_real  = (display.height()-1) - scaled_val;
    uint h_real  = (display.height()-1) - scaled_val;
    display.drawFastVLine(i, 0, display.height()-1, BLACK);
    if (scaled_val >= fft_bars_shown[i]) {
      fft_bars_shown[i] = scaled_val;
      display.drawFastVLine(i, y_real, h_real, WHITE);
    }
    else {
      if (fft_bars_shown[i] > 0) fft_bars_shown[i] = fft_bars_shown[i] - SHOWN_DECAY;
      uint y_decay = (display.height()-1) - fft_bars_shown[i];
      uint h_decay = y_decay - h_real;
      display.drawFastVLine(i, y_decay, h_decay, WHITE);
      display.drawFastVLine(i, y_real, h_real, GREEN);
    }
  }
  if (dirty_button) {
    if (touch->buttonPressed(0)) {
      // Interpret a cancel press as a return to APP_SELECT.
      active_app = AppID::APP_SELECT;
    }
    dirty_button = false;
  }
}


void render_button_icon(uint8_t sym, int x, int y, uint16_t color) {
  const uint8_t ICON_SIZE = 7;
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  switch (sym) {
    case 0:
      x0 = x + (ICON_SIZE >> 1);
      y0 = y;
      x1 = x;
      y1 = y + ICON_SIZE;
      x2 = x + ICON_SIZE;
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case 1:
      x0 = x;
      y0 = y;
      x1 = x + ICON_SIZE;
      y1 = y;
      x2 = x + (ICON_SIZE >> 1);
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case 2:
      x0 = x + ICON_SIZE;
      y0 = y;
      x1 = x;
      y1 = y + (ICON_SIZE >> 1);
      x2 = x + ICON_SIZE;
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case 3:
      x0 = x;
      y0 = y;
      x1 = x + ICON_SIZE;
      y1 = y + (ICON_SIZE >> 1);
      x2 = x;
      y2 = y + ICON_SIZE;
      display.fillTriangle(x0, y0, x1, y1, x2, y2, color);
      break;
    case 4:
      display.drawBitmap(x, y, bitmapPointer(ICON_ACCEPT), 9, 9, color);
      break;
    case 5:
      //display.drawLine(0, 10, display.width()-1, 10, WHITE);
      display.drawBitmap(x, y, bitmapPointer(ICON_CANCEL), 9, 9, color);
      break;
  }
}


/*
* Given a data array, and parameters for the graph, draw the data to the
*   display.
*/
void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  float* dataset, uint32_t data_len
) {
  if (draw_base) {   // Draw the basic frame and axes?
    display.drawFastVLine(x, y, h, WHITE);
    display.drawFastHLine(x, y+h, w, WHITE);
  }
  display.fillRect(x+1, y, w, h-1, BLACK);
  if (w < data_len) {
    dataset += (data_len - w);
    data_len = w;
  }

  float v_max = 0.0;
  float v_min = 0.0;
  for (uint32_t i = 0; i < data_len; i++) {
    float tmp = *(dataset + i);
    v_max = strict_max(v_max, tmp);
    v_min = strict_min(v_min, tmp);
  }
  float h_scale = data_len / w;
  float v_scale = (v_max - v_min) / h;
  for (uint32_t i = 0; i < data_len; i++) {
    uint8_t tmp = *(dataset + i) / v_scale;
    display.writePixel(i + x, (y+h)-tmp, color);
  }
  if (draw_v_ticks) {
    display.drawFastHLine(x+1, y, 2, WHITE);
    display.setCursor(x+2, y);
    display.setTextColor(WHITE);
    display.print(v_max);
    display.setCursor(x+2, (y+h) - 9);
    display.print(v_min);
  }
  if (draw_h_ticks) {
    float last_datum = *(dataset + (data_len-1));
    uint8_t tmp = last_datum / v_scale;
    //display.fillCircle(x+w, tmp+y, 1, color);
    display.setCursor(x, strict_min((uint16_t) ((y+h)-tmp), (uint16_t) (h-1)));
    display.setTextColor(color);
    display.print(*(dataset + (data_len-1)));
  }
}


/*
* Given a filter object, and parameters for the graph, draw the data to the
*   display.
*/
void draw_graph_obj(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_v_ticks, bool draw_h_ticks,
  SensorFilter<float>* filt
) {
  const uint16_t DATA_SIZE = filt->windowSize();
  const uint16_t LAST_SIDX = filt->lastIndex();
  const float*   F_MEM     = filt->memPtr();
  float tmp_data[DATA_SIZE];
  for (uint16_t i = 0; i < DATA_SIZE; i++) {
    tmp_data[i] = *(F_MEM + ((i + LAST_SIDX) % DATA_SIZE));
  }
  draw_graph_obj(x, y, w, h, color, draw_base, draw_v_ticks, draw_h_ticks, tmp_data, (uint32_t) DATA_SIZE);
}


/*
* Given a filter object, and parameters for the graph, draw the data to the
*   display.
*/
void draw_progress_bar(
  int x, int y, int w, int h, uint16_t color,
  bool draw_base, bool draw_val, float percent
) {
  if (draw_base) {   // Draw the basic frame and axes?
    display.fillRect(x, y, w, h, BLACK);
    display.drawRoundRect(x, y, w, h, 3, WHITE);
  }
  uint8_t pix_width = percent * (w-2);
  display.fillRoundRect(x+1, y+1, pix_width, h-2, 3, color);

  if (draw_val && ((h-4) >= 7)) {
    // If we have space to do so, and the application requested it, draw the
    //   progress value in the middle of the bar.
    int txt_x = x+3;
    int txt_y = y+3;
    display.setTextSize(0);
    display.setCursor(txt_x, txt_y);
    display.setTextColor(WHITE);
    display.print(percent * 100.0);
    display.print("%");
  }
}


/*
* Given a vector object, and parameters for the graph, draw the data to the
*   display. The given vector must be normalized.
*/
void draw_3vector(
  int x, int y, int w, int h, uint16_t color,
  bool draw_axes, bool draw_val, float vx, float vy, float vz
) {
  const int PERSPECTIVE_SCALE = 1;
  int origin_x = x + (w >> 1);
  int origin_y = y + (h >> 1);
  if (draw_axes) {   // Draw the axes? The origin is in the middle of the field.
    display.drawFastVLine(origin_x, y, h, WHITE);
    display.drawFastHLine(x, origin_y, w, WHITE);
    display.drawLine(x, (y+h), w, y, WHITE);
    // Only 1/8 of a cube (all vector components are positive).
    //display.drawFastVLine(x, y, h, WHITE);
    //display.drawFastHLine(x, (y+h), w, WHITE);
    //display.drawLine(x, (y+h), w>>1, y>>1, WHITE);
  }
  // Project the vector onto the x/y plane.
  // To give a sense of depth, we use a triangle where only a line is required.
  // We want the y-axis to be northward on the display. So we have to change the
  //   sign of that component.
  Vector3<int> projected(vx * (w >> 1), vy*(h >> 1) * -1, 0);   // TODO: z is unimplemented
  int x1 = origin_x + projected.x - PERSPECTIVE_SCALE;
  int y1 = origin_y + projected.y;
  int x2 = origin_x + projected.x;
  int y2 = origin_y + projected.y - PERSPECTIVE_SCALE;
  display.fillTriangle(origin_x, origin_y, x1, y1, x2, y2, color);
}

/*
* Called at the frame-rate interval for the display.
*/
void updateDisplay() {
  switch (active_app) {
    case AppID::APP_SELECT:    redraw_app_select_window();   break;
    case AppID::TOUCH_TEST:    redraw_touch_test_window();   break;
    case AppID::CONFIGURATOR:  redraw_configurator_window(); break;
    case AppID::DATA_MGMT:     redraw_data_mgmt_window();    break;
    case AppID::SYNTH_BOX:     redraw_fft_window();          break;
    case AppID::COMMS_TEST:    redraw_comms_root_window();   break;
    case AppID::META:          redraw_meta_window();         break;
    case AppID::I2C_SCANNER:   redraw_i2c_probe_window();    break;
    case AppID::TRICORDER:     redraw_tricorder_window();    break;
    case AppID::HOT_STANDBY:   redraw_hot_standby_window();  break;
    case AppID::SUSPEND:       redraw_suspended_window();    break;
  }
}



/*******************************************************************************
* Sensor service functions
*******************************************************************************/

/*
* Reads the VEML6075 and adds the data to the pile.
*/
int8_t read_uv_sensor() {
  int8_t ret = 0;
  graph_array_uva.feedFilter(uv.uva());
  graph_array_uvb.feedFilter(uv.uvb());
  graph_array_uvi.feedFilter(uv.index());
  return ret;
}


/*
* Reads the BME280 and adds the data to the pile.
*/
int8_t read_baro_sensor() {
  int8_t ret = 0;
  altitude  = baro.Altitude(baro.pres());
  dew_point = baro.DewPoint(baro.temp(), baro.hum());
  sea_level = baro.EquivalentSeaLevelPressure(altitude, baro.temp(), baro.pres());
  graph_array_humidity.feedFilter(baro.hum());
  graph_array_air_temp.feedFilter(baro.temp());
  graph_array_pressure.feedFilter(baro.pres());
  return ret;
}


/*
* Reads the IMU and optionally integrates for orientation.
*/
int8_t read_imu() {
  int8_t ret = -1;
  return ret;
}


/*
* Reads the TSL2561 and adds the data to the pile.
*/
int8_t read_visible_sensor() {
  int8_t ret = 0;
  ret = graph_array_visible.feedFilter(1.0 * tsl2561.getLux());
  return ret;
}


/*
* Reads the TMP102 (near the PSU and battery) and adds the data to the pile.
*/
int8_t read_battery_temperature_sensor() {
  return graph_array_psu_temp.feedFilter(tmp102.temperature());
}


/*
* Reads the GridEye sensor and adds the data to the pile.
*/
int8_t read_thermopile_sensor() {
  int8_t ret = 0;
  therm_field_min   = THERM_TEMP_MAX;   // Reset range markers.
  therm_field_max   = THERM_TEMP_MIN;   // Reset range markers.
  therm_field_sum   = 0.0;
  for (uint8_t i = 0; i < 8; i++) {
    for (uint8_t n = 0; n < 8; n++) {
      uint8_t pix_idx = (7 - n) | (i << 3);  // Sensor is rotated 90-deg.
      uint8_t arr_idx = (n << 3) | (i);
      therm_pixels[arr_idx] = grideye.getPixelTemperature(pix_idx);
      therm_field_min = strict_min(therm_pixels[arr_idx], therm_field_min);
      therm_field_max = strict_max(therm_pixels[arr_idx], therm_field_max);
      therm_field_sum += therm_pixels[arr_idx];
    }
  }
  float therm_field_mean = therm_field_sum / 64.0;
  graph_array_therm_mean.feedFilter(therm_field_mean);
  double deviation_sum = 0.0;
  for (uint8_t i = 0; i < 64; i++) {
    deviation_sum += sq(therm_pixels[i] - therm_field_mean);
  }
  therm_field_stdev = sqrt(deviation_sum / 64);
  return ret;
}



/*******************************************************************************
* Touch callbacks
*******************************************************************************/

static void cb_button(int button, bool pressed) {
  last_interaction = millis();
  if (pressed) {
    vibrateOn(19);
  }
  else {
  }
  ledOn(LED_B_PIN, 2, 3500);
  dirty_button = true;
}


static void cb_slider(int slider, int value) {
  last_interaction = millis();
  ledOn(LED_R_PIN, 60, 3500);
  dirty_slider = true;
}


static void cb_longpress(int button, uint32_t duration) {
  ledOn(LED_G_PIN, 50, 3500);
}


/*******************************************************************************
* Console callbacks
*******************************************************************************/

int callback_help(StringBuilder* text_return, StringBuilder* args) {
  if (0 < args->count()) {
    console.printHelp(text_return, args->position_trimmed(0));
  }
  else {
    console.printHelp(text_return);
  }
  return 0;
}

int callback_print_history(StringBuilder* text_return, StringBuilder* args) {
  console.printHistory(text_return);
  return 0;
}

int callback_reboot(StringBuilder* text_return, StringBuilder* args) {
  text_return->concat("Unimplemented\n");
  return 0;
}


int callback_touch_info(StringBuilder* text_return, StringBuilder* args) {
  touch->printDebug(text_return);
  return 0;
}

int callback_touch_reset(StringBuilder* text_return, StringBuilder* args) {
  text_return->concat("SX8634 reset ");
  text_return->concat((0 == touch->reset()) ? "succeded" : "failed");
  return 0;
}

int callback_touch_mode(StringBuilder* text_return, StringBuilder* args) {
  if (args->count() > 0) {
    int mode_int = args->position_as_int(0);
    switch (mode_int) {
      case 0:
      case 1:
      case 2:
      case 3:
        if (0 == touch->setMode((SX8634OpMode) mode_int)) {
          text_return->concatf("SX8634 mode set to %s.\n", SX8634::getModeStr((SX8634OpMode) mode_int));
        }
        else {
          text_return->concat("SX8634 mode set failed.\n");
        }
        break;
      default:
        return -1;
    }
  }
  else {
    text_return->concatf("SX8634 mode set to %s.\n", SX8634::getModeStr(touch->operationalMode()));
  }
  return 0;
}


int callback_display_test(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = args->position_as_int(0);
  uint32_t millis_0 = millis();
  uint32_t millis_1 = millis_0;
  switch (arg0) {
    case 0:
      display.fillScreen(BLACK);
      break;
    case 1:
      redraw_app_window("Test App Title", 0, 0);
      break;
    case 2:
      redraw_touch_test_window();
      break;
    case 3:
      redraw_tricorder_window();
      break;
    case 4:
      redraw_fft_window();
      break;
    case 5:
      display.setAddrWindow(0, 0, 96, 64);
      for (uint8_t h = 0; h < 64; h++) {
        for (uint8_t w = 0; w < 96; w++) {
          if (w > 83) {
            display.writePixel(w, h, WHITE);
          } else if (w > 71) {
            display.writePixel(w, h, BLUE);
          } else if (w > 59) {
            display.writePixel(w, h, GREEN);
          } else if (w > 47) {
            display.writePixel(w, h, CYAN);
          } else if (w > 35) {
            display.writePixel(w, h, RED);
          } else if (w > 23) {
            display.writePixel(w, h, MAGENTA);
          } else if (w > 11) {
            display.writePixel(w, h, YELLOW);
          } else {
            display.writePixel(w, h, BLACK);
          }
        }
      }
      display.endWrite();
      break;
    case 6:
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_THERMO), 14, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_IMU), 32, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_GPS), 32, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_LIGHT), 28, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_UVI), 32, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_SOUND), 40, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_RH), 22, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_MIC), 19, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_MAGNET), 22, 32, 0xFFFF);
      delay(5000);
      display.fillScreen(BLACK);
      display.drawBitmap(0, 0, bitmapPointer(ICON_BATTERY), 52, 32, 0xFFFF);
      delay(5000);
      break;
    case 7:
      render_button_icon(0, 0, 0, WHITE);
      render_button_icon(1, 10, 0, WHITE);
      render_button_icon(2, 20, 0, WHITE);
      render_button_icon(3, 30, 0, WHITE);
      render_button_icon(4, 0, 20, WHITE);
      render_button_icon(5, 0, 20, WHITE);
      break;
    case 8:
      display.fillCircle(32, 32, 7, 0xFF00);
      display.drawCircle(32, 32, 14, 0x00FF);
      break;
    case 9:    // Progress bar test
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar(0, 0, 95, 12, BLUE, true, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar(0, 14, 95, 7, BLUE, true, false, 0.0);  delay(250);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar(0, 14, 95, 7, BLUE, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar(0, 23, 32, 7, BLUE, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar(0, 23, 32, 7, BLUE, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar(34, 23, 51, 7, RED, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar(34, 23, 51, 7, RED, false, false, (i * 0.01));
        delay(40);
      }

      draw_progress_bar(0, 34, 95, 14, GREEN, true, false, 0.0);
      for (uint8_t i = 0; i <= 100; i++) {
        draw_progress_bar(0, 34, 95, 14, GREEN, false, true, (i * 0.01));
        delay(40);
      }
      break;
    case 10:    // Vector display test
      display.fillScreen(BLACK);
      draw_3vector(0, 0, 50, 50, RED,    true,  false, 1.0, 0.0, 0.0);
      draw_3vector(0, 0, 50, 50, GREEN,  false, false, 0.0, 1.0, 0.0);
      draw_3vector(0, 0, 50, 50, BLUE,   false, false, 0.0, 0.0, 1.0);
      draw_3vector(0, 0, 50, 50, YELLOW, false, false, 1.0, 1.0, 0.0);
      draw_3vector(0, 0, 50, 50, CYAN,   false, false, 0.13, 0.65, 0.0);
      break;
    default:
      return -1;
  }
  millis_1 = millis();
  text_return->concatf("Display update took %ums\n", millis_1-millis_0);
  return 0;
}

int callback_vibrator_test(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = (0 < args->count()) ? args->position_as_int(0) : 40;  // Duration
  int arg1 = (1 < args->count()) ? args->position_as_int(1) : 4095;  // Intensity
  vibrateOn(arg0, arg1);
  return 0;
}

int callback_led_test(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = args->position_as_int(0);
  int arg1 = (1 < args->count()) ? args->position_as_int(1) : 10;    // Duration
  int arg2 = (2 < args->count()) ? args->position_as_int(2) : 2048;  // Intensity
  switch (arg0) {
    case 0:   ledOn(LED_R_PIN, arg1, arg2);  break;
    case 1:   ledOn(LED_G_PIN, arg1, arg2);  break;
    case 2:   ledOn(LED_B_PIN, arg1, arg2);  break;
    default:  return -1;
  }
  return 0;
}

int callback_fft_mix(StringBuilder* text_return, StringBuilder* args) {
  float arg0 = args->position_as_double(0);
  float arg1 = args->position_as_double(1);
  float arg2 = args->position_as_double(2);
  float arg3 = args->position_as_double(3);
  mixerFFT.gain(0, arg0);  // sineL
  mixerFFT.gain(1, arg1);  // sineR
  mixerFFT.gain(2, arg2);  // light_adc
  mixerFFT.gain(3, arg3);  // mic_adc
  return 0;
}

int callback_aout_mix(StringBuilder* text_return, StringBuilder* args) {
  float arg0 = args->position_as_double(0);
  float arg1 = args->position_as_double(1);
  float arg2 = args->position_as_double(2);
  float arg3 = args->position_as_double(3);
  mixerL.gain(0, arg0);  // queueL
  mixerL.gain(1, arg1);  // sineL
  mixerL.gain(2, arg2);  // light_adc
  mixerL.gain(3, arg3);  // mic_adc
  mixerR.gain(0, arg0);  // queueR
  mixerR.gain(1, arg1);  // sineR
  mixerR.gain(2, arg2);  // light_adc
  mixerR.gain(3, arg3);  // mic_adc
  return 0;
}

int callback_synth_set(StringBuilder* text_return, StringBuilder* args) {
  AudioSynthWaveformSine* synth = nullptr;
  int arg0 = args->position_as_int(0);
  switch (arg0) {
    case 0:   synth = &sineL;  break;
    case 1:   synth = &sineR;  break;
    default:  return -1;
  }
  if (1 < args->count()) {
    synth->frequency(args->position_as_int(1));
  }
  if (2 < args->count()) {
    synth->amplitude(args->position_as_double(2));
  }
  if (3 < args->count()) {
    synth->phase(args->position_as_double(3));
  }
  return 0;
}


int callback_active_app(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = args->position_as_int(0);
  switch (arg0) {
    case 0:   active_app = AppID::APP_SELECT;    break;
    case 1:   active_app = AppID::TOUCH_TEST;    break;
    case 2:   active_app = AppID::CONFIGURATOR;  break;
    case 3:   active_app = AppID::DATA_MGMT;     break;
    case 4:   active_app = AppID::SYNTH_BOX;     break;
    case 5:   active_app = AppID::COMMS_TEST;    break;
    case 6:   active_app = AppID::META;          break;
    case 7:   active_app = AppID::I2C_SCANNER;   break;
    case 8:   active_app = AppID::TRICORDER;     break;
    case 9:   active_app = AppID::HOT_STANDBY;   break;
    case 10:  active_app = AppID::SUSPEND;       break;
    default:
      text_return->concatf("Unsupported app: %d\n", arg0);
      return -1;
  }
  return 0;
}


int callback_sensor_info(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = args->position_as_int(0);
  switch ((SensorID) arg0) {
    case SensorID::MAGNETOMETER:   magnetometer.printDebug(text_return);  break;
    //case SensorID::BARO:           baro.printDebug(text_return);          break;
    //case SensorID::LIGHT:          tsl2561.printDebug(text_return);       break;
    //case SensorID::UV:             uv.printDebug(text_return);            break;
    //case SensorID::THERMOPILE:     grideye.printDebug(text_return);       break;
    case SensorID::BATT_VOLTAGE:       break;
    case SensorID::IMU:                break;
    case SensorID::MIC:                break;
    case SensorID::GPS:                break;
    //case SensorID::PSU_TEMP:       tmp102.printDebug(text_return);        break;
    default:
      text_return->concatf("Unsupported sensor: %d\n", arg0);
      return -1;
  }
  return 0;
}


int callback_sensor_init(StringBuilder* text_return, StringBuilder* args) {
  int arg0 = args->position_as_int(0);
  int arg1 = args->position_as_int(1);
  int ret = 0;
  switch ((SensorID) arg0) {
    case SensorID::MAGNETOMETER:   break;
    case SensorID::BARO:           ret = baro.init(&Wire1);       break;
    case SensorID::LIGHT:          ret = tsl2561.init(&Wire1);    break;
    case SensorID::UV:             ret = uv.init(&Wire1);         break;
    case SensorID::THERMOPILE:     ret = grideye.init(&Wire1);    break;
    case SensorID::PSU_TEMP:       ret = tmp102.init(&Wire1);     break;
    case SensorID::BATT_VOLTAGE:       break;
    case SensorID::IMU:                break;
    case SensorID::MIC:                break;
    case SensorID::GPS:                break;
    default:
      text_return->concatf("Unsupported sensor: %d\n", arg0);
      return -1;
  }
  text_return->concatf("Sensor %d init() returned %d\n", arg0, ret);
  return 0;
}


int callback_sensor_enable(StringBuilder* text_return, StringBuilder* args) {
  int arg0  = args->position_as_int(0);
  bool arg1 = (1 < args->count()) ? (1 == args->position_as_int(1)) : true;
  bool en   = false;
  switch ((SensorID) arg0) {
    case SensorID::MAGNETOMETER:  magnetometer.power(arg1);   en = arg1;      break;
    case SensorID::BARO:          en = baro.enabled();            break;
    case SensorID::LIGHT:         tsl2561.enabled(arg1);  en = tsl2561.enabled();   break;
    case SensorID::UV:            uv.enabled(arg1);       en = uv.enabled();        break;
    case SensorID::THERMOPILE:    grideye.enabled(arg1);  en = grideye.enabled();   break;
    case SensorID::BATT_VOLTAGE:  tmp102.enabled(arg1);   en = tmp102.enabled();    break;
    case SensorID::IMU:           break;
    case SensorID::MIC:           break;
    case SensorID::GPS:           break;
    case SensorID::PSU_TEMP:      break;
    default:
      text_return->concatf("Unsupported sensor: %d\n", arg0);
      return -1;
  }
  text_return->concatf("Sensor %d is now %sabled\n", arg0, arg1?"en":"dis");
  return 0;
}


int callback_audio_volume(StringBuilder* text_return, StringBuilder* args) {
  if (1 == args->count()) {
    float arg0 = args->position_as_double(0);
    ampL.gain(arg0);
    ampR.gain(arg0);
    text_return->concatf("Audio volume: %.2f\n", arg0);
  }
  return 0;
}


/*******************************************************************************
* Setup function
*******************************************************************************/
void setup() {
  boot_time = millis();
  float percent_setup = 0.0;
  char* init_step_str = "";
  int cursor_height = 26;
  Serial.begin(115200);   // USB

  pinMode(IMU_IRQ_PIN, INPUT_PULLUP);
  pinMode(DRV425_CS_PIN, INPUT); // Wrong
  pinMode(PSU_SX_IRQ_PIN, INPUT);
  pinMode(ANA_LIGHT_PIN, INPUT);
  pinMode(LED_R_PIN, INPUT);
  pinMode(LED_G_PIN, INPUT);
  pinMode(LED_B_PIN, INPUT);

  SPI.setSCK(SPISCK_PIN);
  SPI.setMISO(SPIMISO_PIN);
  SPI.setMOSI(SPIMOSI_PIN);
  SPI.begin();

  Wire.setSDA(SDA0_PIN);
  Wire.setSCL(SCL0_PIN);
  Wire.setClock(400000);
  Wire.begin();
  Wire1.setSDA(SDA1_PIN);
  Wire1.setSCL(SCL1_PIN);
  Wire.setClock(400000);
  Wire1.begin();

  Serial1.setRX(GPS_TX_PIN);
  Serial1.setTX(GPS_RX_PIN);
  Serial1.begin(9600);    // GPS

  Serial6.setRX(COMM_TX_PIN);
  Serial6.setTX(COMM_RX_PIN);
  Serial6.begin(115200);    // Comm
  AudioMemory(32);

  //SD.begin(BUILTIN_SDCARD);

  sineL.amplitude(1.0);
  sineL.frequency(440);
  sineL.phase(0);
  sineR.amplitude(0.8);
  sineR.frequency(660);
  sineR.phase(0);

  mixerFFT.gain(0, 0.0);  // sineL
  mixerFFT.gain(1, 1.0);  // sineR
  mixerFFT.gain(2, 0.0);  // light_adc
  mixerFFT.gain(3, 0.0);  // mic_adc

  mixerL.gain(0, 0.0);  // queueL
  mixerL.gain(1, 1.0);  // sineL
  mixerL.gain(2, 0.0);  // light_adc
  mixerL.gain(3, 0.0);  // mic_adc

  mixerR.gain(0, 0.0);  // queueR
  mixerR.gain(1, 1.0);  // sineR
  mixerR.gain(2, 0.0);  // light_adc
  mixerR.gain(3, 0.0);  // mic_adc

  pinkNoise.amplitude(1.0);

  ampL.gain(0.4);
  ampR.gain(0.4);

  analogWriteResolution(12);
  graph_array_ana_light.init();

  display.begin();
  display.fillScreen(BLACK);
  display.setCursor(14, 0);
  display.setTextSize(1);
  display.println("Motherflux0r");
  display.setTextSize(0);
  //display.setTextColor(CYAN);
  //display.println(TEST_PROG_VERSION);
  draw_progress_bar(0, 11, 95, 12, GREEN, true, false, percent_setup);

  init_step_str = "Audio           ";
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  delay(10);

  init_step_str = "GridEye         ";
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (0 == grideye.init(&Wire1)) {
    graph_array_therm_mean.init();
  }
  else {
    display.setTextColor(RED);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }
  delay(10);

  init_step_str = "Magnetometer    ";
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (0 == magnetometer.init(&Wire1, &SPI)) {
  }
  else {
    display.setTextColor(RED);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }
  delay(10);

  init_step_str = "Baro            ";
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (0 == baro.init(&Wire1)) {
    graph_array_pressure.init();
    graph_array_humidity.init();
    graph_array_air_temp.init();
  }
  else {
    display.setTextColor(RED);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }
  delay(10);

  init_step_str = "UVI";
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (VEML6075_ERROR_SUCCESS == uv.init(&Wire1)) {
    graph_array_uva.init();
    graph_array_uvb.init();
    graph_array_uvi.init();
  }
  else {
    display.setTextColor(RED, BLACK);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }
  delay(10);

  init_step_str = "Lux ";
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  graph_array_visible.init();
  int8_t lux_ret = tsl2561.init(&Wire1);
  if (0 == lux_ret) {
    tsl2561.integrationTime(TSLIntegrationTime::MS_101);
  }
  else {
    display.setTextColor(RED, BLACK);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    display.print(lux_ret);
    cursor_height += 8;
  }
  delay(10);

  init_step_str = "PSU Temperature";
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  //if (0 == tmp102.init(&Wire)) {
  if (false) {
    graph_array_psu_temp.init();
  }
  else {
    display.setTextColor(RED, BLACK);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }
  delay(10);

  init_step_str = "Inertial        ";
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  //if (0 == imu.init(&Wire)) {
  if (false) {
  }
  else {
    display.setTextColor(RED);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }
  delay(10);

  //imu.begin(IMU_CS_PIN, SPI, 10000000);
  //imu.swReset();
  //imu.sleep(false);
  //imu.lowPower(false);
  //imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm2;         // gpm2, gpm4, gpm8, gpm16
  myFSS.g = dps250;       // dps250, dps500, dps1000, dps2000
  //imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                          // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                          // acc_d111bw4_n136bw
                                          // acc_d50bw4_n68bw8
                                          // acc_d23bw9_n34bw4
                                          // acc_d11bw5_n17bw
                                          // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                          // acc_d473bw_n499bw
  myDLPcfg.g = gyr_d361bw4_n376bw5;       // gyr_d196bw6_n229bw8, gyr_d151bw8_n187bw6, gyr_d119bw5_n154bw3, gyr_d51bw2_n73bw3, gyr_d23bw9_n35bw9, gyr_d11bw6_n17bw8, gyr_d5bw7_n8bw9, gyr_d361bw4_n376bw5
  //imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  // Choose whether or not to use DLPF
  //ICM_20948_Status_e accDLPEnableStat = imu.enableDLPF((ICM_20948_Internal_Gyr | ICM_20948_Internal_Acc), true);
  if (255 != IMU_IRQ_PIN) {
    pinMode(IMU_IRQ_PIN, INPUT);
    //imu.cfgIntActiveLow(true);
    //imu.cfgIntOpenDrain(false);
    //imu.cfgIntLatch(true);          // IRQ is a 50us pulse.
    //imu.intEnableRawDataReady(true);
    //attachInterrupt(digitalPinToInterrupt(IMU_IRQ_PIN), imu_isr_fxn, FALLING);
  }

  init_step_str = "Console         ";
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);

  disp_update_last = millis();
  disp_update_next = disp_update_last + 1800;
  console.defineCommand("help",        '?', arg_list_1_str, "Prints help to console.", "", 0, callback_help);
  console.defineCommand("history",     arg_list_0, "Print command history.", "", 0, callback_print_history);
  console.defineCommand("reboot",      arg_list_0, "Reboot the controller.", "", 0, callback_reboot);
  console.defineCommand("touchreset",  arg_list_0, "Reset SX8634", "", 0, callback_touch_reset);
  console.defineCommand("touchinfo",   arg_list_0, "SX8634 info", "", 0, callback_touch_info);
  console.defineCommand("touchmode",   arg_list_1_uint, "Get/set SX8634 mode", "", 0, callback_touch_mode);
  console.defineCommand("led",   arg_list_3_uint, "LED Test", "", 1, callback_led_test);
  console.defineCommand("vib",   'v', arg_list_2_uint, "Vibrator test", "", 0, callback_vibrator_test);
  console.defineCommand("disp",  'd', arg_list_1_uint, "Display test", "", 1, callback_display_test);
  console.defineCommand("aout",  arg_list_4_float, "Mix volumes for the headphones.", "", 4, callback_aout_mix);
  console.defineCommand("fft",   arg_list_4_float, "Mix volumes for the FFT.", "", 4, callback_fft_mix);
  console.defineCommand("synth", arg_list_4_uuff, "Mix volumes for the FFT.", "", 2, callback_synth_set);
  console.defineCommand("si",    's', arg_list_1_uint, "Sensor information.", "", 1, callback_sensor_info);
  console.defineCommand("sinit", arg_list_2_uint, "Sensor initialize.", "", 1, callback_sensor_init);
  console.defineCommand("se",    arg_list_2_uint, "Sensor enable.", "", 1, callback_sensor_enable);
  console.defineCommand("app",   'a', arg_list_1_uint, "Select active application.", "", 1, callback_active_app);
  console.defineCommand("vol",   arg_list_1_float, "Audio volume.", "", 0, callback_audio_volume);
  console.setTXTerminator(LineTerm::CRLF);
  console.setRXTerminator(LineTerm::CR);
  console.localEcho(true);
  console.init();

  StringBuilder ptc("Motherflux0r ");
  //ptc.concat(TEST_PROG_VERSION);
  console.printToLog(&ptc);
  delay(10);

  init_step_str = "Touchpad        ";
  touch = new SX8634(&_touch_opts);
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  if (0 == touch->init(&Wire)) {
  }
  else {
    display.setTextColor(RED);
    display.setCursor(0, cursor_height);
    display.print(init_step_str);
    cursor_height += 8;
  }
  delay(10);

  disp_update_last = millis();
  disp_update_next = disp_update_last + 3000;

  config_time = millis();
  //display.setTextColor(GREEN);
  //display.print((config_time - boot_time), DEC);
  //display.println("ms");

  init_step_str = "USB        ";
  percent_setup += 0.08;
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, percent_setup);
  display.setTextColor(WHITE);
  display.setCursor(4, 14);
  display.print(init_step_str);
  while (Serial.available()) {
    Serial.read();
  }
  //display.println("Serial");

  while (disp_update_next > millis()) {}
  if (touch->deviceFound()) {
    touch->poll();
    touch->setMode(SX8634OpMode::ACTIVE);
    touch->setLongpress(800, 0);   // 800ms is a long-press. No rep.
    touch->setButtonFxn(cb_button);
    touch->setSliderFxn(cb_slider);
    touch->setLongpressFxn(cb_longpress);
  }
  draw_progress_bar(0, 11, 95, 12, GREEN, false, false, 1.0);
  delay(50);
}



/*******************************************************************************
* Main loop
*******************************************************************************/
void loop() {
  StringBuilder output;

  if (Serial) {
    const uint8_t RX_BUF_LEN = 32;
    char ser_buffer[RX_BUF_LEN];
    uint8_t rx_len = 0;
    bool cr_rxd = false;
    memset(ser_buffer, 0, RX_BUF_LEN);
    while ((RX_BUF_LEN > rx_len) && (0 < Serial.available())) {
      ser_buffer[rx_len++] = Serial.read();
    }
    if (rx_len > 0) {
      switch (console.feed(ser_buffer, rx_len)) {
        case -1:   // console buffered the data, but took no other action.
        default:
          ledOn(LED_B_PIN, 5, 500);
          break;
        case 0:   // A full line came in.
          last_interaction  = millis();
          ledOn(LED_R_PIN, 5, 500);
          break;
        case 1:   // A callback was called.
          last_interaction  = millis();
          ledOn(LED_G_PIN, 5, 500);
          break;
      }
    }
    console.fetchLog(&output);
  }

  int8_t t_res = touch->poll();
  if (0 < t_res) {
    // Something changed in the hardware.
  }
  if (imu_irq_fired) {
    imu_irq_fired = false;
    read_imu();
    //imu.clearInterrupts();
  }

  /* Run our async cleanup stuff. */
  uint32_t millis_now = millis();
  if (millis_now >= off_time_led_r) {   pinMode(LED_R_PIN, INPUT);     }
  if (millis_now >= off_time_led_g) {   pinMode(LED_G_PIN, INPUT);     }
  if (millis_now >= off_time_led_b) {   pinMode(LED_B_PIN, INPUT);     }
  if (millis_now >= off_time_vib) {     pinMode(VIBRATOR_PIN, INPUT);  }

  /* Poll each sensor class. */
  if (0 < baro.poll()) {           read_baro_sensor();                  }
  if (0 < uv.poll()) {             read_uv_sensor();                    }
  if (0 < tsl2561.poll()) {        read_visible_sensor();               }
  if (0 < grideye.poll()) {        read_thermopile_sensor();            }
  //if (0 < tmp102.poll()) {         read_battery_temperature_sensor();   }
  //if (1 == magnetometer.poll()) {}

  if ((last_interaction + 100000) <= millis_now) {
    // After 100 seconds, time-out the display.
    if (AppID::HOT_STANDBY != active_app) {
      active_app = AppID::HOT_STANDBY;
    }
  }

  millis_now = millis();
  if (disp_update_next <= millis_now) {
    updateDisplay();
    disp_update_last = millis_now;
    disp_update_next = (1000 / update_disp_rate) + disp_update_last;
  }

  if ((Serial) && (output.length() > 0)) {
    Serial.write(output.string(), output.length());
  }
}
