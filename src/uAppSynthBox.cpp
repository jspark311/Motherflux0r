#include <CppPotpourri.h>
#include <SensorFilter.h>
#include <Audio.h>
#include "uApp.h"
#include "Motherflux0r.h"


extern SSD13xx display;

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


uAppSynthBox::uAppSynthBox() : uApp("SynthBox", (Image*) &display) {}

uAppSynthBox::~uAppSynthBox() {}



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
int8_t uAppSynthBox::_lc_on_preinit() {
  int8_t ret = 1;
  redraw_app_window();
  return ret;
}


/**
* Called by superclass to perform the first draw. Input will be processed and
*   the display redrawn after this function returns.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppSynthBox::_lc_on_active() {
  int8_t ret = 0;
  return ret;
}


/**
* Called by superclass to warn us of impending destruction. Here, the class
*   ought to dispatch any final I/O, and start to close down.
*
* @return 0 for no lifecycle FSM change, 1 for FSM increment, -1 for halt.
*/
int8_t uAppSynthBox::_lc_on_teardown() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass to perform the final cleanup. Any memory allocated on the
*   heap needs to be cleaned up or accounted for before this function returns 1.
*
* @return 0 for no lifecycle FSM change, 1 for FSM reset to PREINIT, -1 for halt.
*/
int8_t uAppSynthBox::_lc_on_inactive() {
  int8_t ret = 1;
  return ret;
}


/**
* Called by superclass while app is ACTIVE to handle user input that has
*   collected since the last tick.
*
* @return 0 for no change, 1 for display refresh, -1 for application change.
*/
int8_t uAppSynthBox::_process_user_input() {
  int8_t ret = 0;

  if (_slider_current != _slider_pending) {
    FB->setCursor(64, 0);
    FB->setTextColor(YELLOW, BLACK);
    if (_slider_pending <= 7) {
      FB->writeString("Vol  ");
      draw_progress_bar_vertical(0,  11, 27, 52, GREEN, true, true, volume_left_output);
      draw_progress_bar_vertical(29, 11, 27, 52, GREEN, true, true, volume_right_output);
      draw_progress_bar_vertical(58, 11, 27, 52, GREEN, true, true, volume_pink_noise);
    }
    else if (_slider_pending <= 15) {
      FB->writeString("Mix F");
      draw_progress_bar_vertical(0,  11, 22, 52, GREEN, true, true, mix_queueL_to_fft);
      draw_progress_bar_vertical(23, 11, 22, 52, GREEN, true, true, mix_queueR_to_fft);
      draw_progress_bar_vertical(46, 11, 22, 52, GREEN, true, true, mix_noise_to_fft);
      draw_progress_bar_vertical(69, 11, 22, 52, GREEN, true, true, 0.0);
    }
    else if (_slider_pending <= 22) {
      FB->writeString("Mix O");
      draw_progress_bar_vertical(0,  11, 22, 52, GREEN, true, true, mix_synth_to_line);
      draw_progress_bar_vertical(23, 11, 22, 52, GREEN, true, true, mix_queue_to_line);
      draw_progress_bar_vertical(46, 11, 22, 52, GREEN, true, true, mix_noise_to_line);
      draw_progress_bar_vertical(69, 11, 22, 52, GREEN, true, true, 0.0);
    }
    else if (_slider_pending <= 30) {
      FB->writeString("DTMF ");
    }
    else if (_slider_pending <= 37) {
      FB->writeString("Slot0 ");
    }
    else if (_slider_pending <= 45) {
      FB->writeString("Slot1 ");
    }
    else if (_slider_pending <= 52) {
      FB->writeString("Slot2 ");
    }
    else {
      FB->writeString("FFT  ");
    }
    _slider_current = _slider_pending;
    ret = 1;
  }
  if (_buttons_current != _buttons_pending) {
    uint16_t diff = _buttons_current ^ _buttons_pending;

    if (diff & 0x0001) {   // Interpret a cancel press as a return to APP_SELECT.
      uApp::setAppActive(AppID::APP_SELECT);
      ret = -1;
    }
    if (diff & 0x0002) {   // Cluttered display toggle.
      if (_buttons_pending & 0x0002) {
        _cluttered_display(!_cluttered_display());
      }
    }
    if (diff & 0x0008) {   // Text of actual value.
      if (_buttons_pending & 0x0008) {
        _render_text_value(!_render_text_value());
      }
    }
    if (diff & 0x0010) {   // Button 5
      if (_buttons_pending & 0x0010) {
        _render_lock_range(!_render_lock_range());
      }
    }
    _buttons_current = _buttons_pending;
    bool up_pressed   = (_buttons_current & 0x0002);
    bool down_pressed = (_buttons_current & 0x0010);
    _button_pressed_up(!down_pressed && up_pressed);
    _button_pressed_dn(down_pressed && !up_pressed);
    ret = 1;
  }

  if ((0 == ret) && (52 < _slider_current)) {
    // We want the FFT window to refresh always.
    ret = 1;
  }
  return ret;
}



/*
* Draws the synth and sound app.
* sineL.amplitude(1.0);
* sineL.frequency(440);
* sineL.phase(0);
* sineR.amplitude(0.8);
* sineR.frequency(660);
* sineR.phase(0);
*/
void uAppSynthBox::_redraw_window() {
  if (_slider_current <= 7) {
    if (_button_pressed_up() || _button_pressed_dn()) {
      if (_button_pressed_dn()) {
        float temp_float = volume_left_output - 0.05;
        if (temp_float < 0.0) temp_float = 0.0;
        volume_left_output  = temp_float;
        volume_right_output = temp_float;
      }
      else if (_button_pressed_up()) {
        float temp_float = volume_left_output + 0.05;
        if (temp_float > 1.0) temp_float = 1.0;
        volume_left_output  = temp_float;
        volume_right_output = temp_float;
      }
      ampL.gain(volume_left_output);
      ampR.gain(volume_right_output);
      pinkNoise.amplitude(volume_pink_noise);
      draw_progress_bar_vertical(0,  11, 27, 52, GREEN, false, true, volume_left_output);
      draw_progress_bar_vertical(29, 11, 27, 52, GREEN, false, true, volume_right_output);
      draw_progress_bar_vertical(58, 11, 27, 52, GREEN, false, true, volume_pink_noise);
    }
  }
  else if (_slider_current <= 15) {
    if (_button_pressed_up() || _button_pressed_dn()) {
      if (_button_pressed_dn()) {
        float temp_float = mix_noise_to_fft - 0.05;
        if (temp_float < 0.0) temp_float = 0.0;
        mix_noise_to_fft = temp_float;
      }
      else if (_button_pressed_up()) {
        float temp_float = mix_noise_to_fft + 0.05;
        if (temp_float > 1.0) temp_float = 1.0;
        mix_noise_to_fft = temp_float;
      }
      mixerFFT.gain(0, mix_queueL_to_fft);
      mixerFFT.gain(1, mix_queueR_to_fft);
      mixerFFT.gain(2, mix_noise_to_fft);
      mixerFFT.gain(3, 0.0);
      draw_progress_bar_vertical(0,  11, 22, 52, GREEN, false, true, mix_queueL_to_fft);
      draw_progress_bar_vertical(23, 11, 22, 52, GREEN, false, true, mix_queueR_to_fft);
      draw_progress_bar_vertical(46, 11, 22, 52, GREEN, false, true, mix_noise_to_fft);
      draw_progress_bar_vertical(69, 11, 22, 52, GREEN, false, true, 0.0);
    }
  }
  else if (_slider_current <= 22) {
    if (_button_pressed_up() || _button_pressed_dn()) {
      if (_button_pressed_dn()) {
        float temp_float = mix_synth_to_line - 0.05;
        if (temp_float < 0.0) temp_float = 0.0;
        mix_synth_to_line = temp_float;
      }
      else if (_button_pressed_up()) {
        float temp_float = mix_synth_to_line + 0.05;
        if (temp_float > 1.0) temp_float = 1.0;
        mix_synth_to_line = temp_float;
      }
      mixerL.gain(0, mix_queue_to_line);
      mixerL.gain(1, mix_noise_to_line);
      mixerL.gain(2, mix_synth_to_line);
      mixerL.gain(3, 0.0);
      mixerR.gain(0, mix_queue_to_line);
      mixerR.gain(1, mix_noise_to_line);
      mixerR.gain(2, mix_synth_to_line);
      mixerR.gain(3, 0.0);
      draw_progress_bar_vertical(0,  11, 22, 52, GREEN, false, true, mix_synth_to_line);
      draw_progress_bar_vertical(23, 11, 22, 52, GREEN, false, true, mix_queue_to_line);
      draw_progress_bar_vertical(46, 11, 22, 52, GREEN, false, true, mix_noise_to_line);
      draw_progress_bar_vertical(69, 11, 22, 52, GREEN, false, true, 0.0);
    }
  }
  else if (_slider_current <= 30) {
  }
  else if (_slider_current <= 37) {
  }
  else if (_slider_current <= 45) {
  }
  else if (_slider_current <= 52) {
  }
  else {
    const float SCALER_PIX   = 52;
    const int   SHOWN_DECAY  = 1;
    float fft_bins[96];
    for (uint8_t i = 0; i < 96; i++) {
      fft_bins[i] = fft256_1.read(BIN_INDICIES[i << 1], BIN_INDICIES[(i << 1) + 1]);
    }
    for (uint8_t i = 0; i < 96; i++) {
      uint8_t scaled_val = fft_bins[i] * SCALER_PIX;
      uint y_real  = (FB->y()-12) - scaled_val;
      uint h_real  = (FB->y()-12) - scaled_val;
      FB->drawFastVLine(i, 11, FB->y()-12, BLACK);
      if (scaled_val >= fft_bars_shown[i]) {
        fft_bars_shown[i] = scaled_val;
        FB->drawFastVLine(i, y_real, h_real, GREEN);
      }
      else {
        if (fft_bars_shown[i] > 0) fft_bars_shown[i] = fft_bars_shown[i] - SHOWN_DECAY;
        uint y_decay = (FB->y()-12) - fft_bars_shown[i];
        uint h_decay = y_decay - h_real;
        FB->drawFastVLine(i, y_decay, h_decay, GREEN);
        FB->drawFastVLine(i, y_real, h_real, WHITE);
      }
    }
  }
}
