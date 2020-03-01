/*
*                                                  ---J. Ian Lindsay  2020.02.29
*/
#include <Arduino.h>
#include <StopWatch.h>

#ifndef __U_APP_H_
#define __U_APP_H_


/*******************************************************************************
* Types
*******************************************************************************/
enum class AppID : uint8_t {
  APP_SELECT   =  0,  // For choosing the app.
  TOUCH_TEST   =  1,  // For diagnostics of the touch pad.
  CONFIGURATOR =  2,  // For tuning all the things.
  DATA_MGMT    =  3,  // For managing recorded datasets.
  SYNTH_BOX    =  4,  // Sound synthesis from data.
  COMMS_TEST   =  5,  // Connecting to the outside world.
  META         =  6,  // Shutdown/reboot/reflash, profiles.
  I2C_SCANNER  =  7,  // Tool for non-intrusively scanning foreign i2c buses.
  TRICORDER    =  8,  // This is the primary purpose of the device.
  HOT_STANDBY  =  9,  // Full operation with powered-down UI elements.
  SUSPEND      = 10   // Minimal power without an obligatory reboot.
};


/* Struct for tracking application state. */
// TODO: This should evolve into a class if we irradiate millions of copies of it.
typedef struct {
  const char* const title;           // Name of tha application.
  const AppID       id;              // ID of the application.
  uint8_t           page_count;      // Total page count.
  uint8_t           page_top;        // The currently visible page.
  uint8_t           slider_val;      // Cached slider value.
  uint8_t           frame_rate;      // App's frame rate.
  bool              screen_refresh;  // Set to indicate a refresh is needed.
  bool              app_active;      // This app is active.
  bool              locked;          // This app is locked into its current state.
} AppHandle;

/* Struct for defining global hotkeys. */
typedef struct {
  uint8_t id;        // Uniquely IDs this hotkey combo.
  uint8_t buttons;   // To trigger, the button state must equal this...
  uint32_t duration; // ...for at least this many milliseconds.
} KeyCombo;




/* Class flags */
#define UAPP_FLAG_ENABLED          0x00000001  // This application is active.
#define UAPP_FLAG_CLUTTER_DISPLAY  0x00000002  // User wants more information on the display.
#define UAPP_FLAG_TEXT_OF_VALUE    0x00000004  // User wants more information on the display.
#define UAPP_FLAG_LOCK_RANGE       0x00000008  // Lock the output range.

#define UAPP_FLAG_PRESS_UP         0x40000000  //
#define UAPP_FLAG_PRESS_DOWN       0x80000000  //


/*******************************************************************************
*******************************************************************************/
class uApp {
  public:
    inline bool enabled() {          return _uapp_flag(UAPP_FLAG_ENABLED);        };

    virtual int8_t refresh() =0;

    inline void deliverSliderValue(uint16_t val) {  _slider_pending  = val;   };
    inline void deliverButtonValue(uint16_t val) {  _buttons_pending  = val;  };

    inline void printStopwatch(StringBuilder* out) {  _stopwatch.printDebug(_UA_NAME, out);   };
    inline void resetStopwatch() {                    _stopwatch.reset();                     };

    static void  setAppActive(AppID);
    static AppID appActive();
    static AppID drawnApp();
    static AppID previousApp();

    static int8_t redraw();
    static uApp*  getActiveAppPtr();
    static const char* const getAppIDString(AppID);
    static void listAllApplications(StringBuilder*);

    // TODO: Should be protected.
    static void redraw_app_window(const char* title, uint8_t pages, uint8_t active_page);


  protected:
    const char*   _UA_NAME;
    StopWatch     _stopwatch;
    uint16_t      _slider_current  = 0;
    uint16_t      _slider_pending  = 0;
    uint16_t      _buttons_current = 0;
    uint16_t      _buttons_pending = 0;

    uApp(const char* _n) : _UA_NAME(_n) {};
    virtual ~uApp() {};

    inline void redraw_app_window() {         redraw_app_window(_UA_NAME, 0, 0);  };

    inline bool _cluttered_display() {        return _uapp_flag(UAPP_FLAG_CLUTTER_DISPLAY);   };
    inline void _cluttered_display(bool x) {  _uapp_set_flag(UAPP_FLAG_CLUTTER_DISPLAY, x);   };
    inline bool _render_text_value() {        return _uapp_flag(UAPP_FLAG_TEXT_OF_VALUE);     };
    inline void _render_text_value(bool x) {  _uapp_set_flag(UAPP_FLAG_TEXT_OF_VALUE, x);     };
    inline bool _render_lock_range() {        return _uapp_flag(UAPP_FLAG_LOCK_RANGE);        };
    inline void _render_lock_range(bool x) {  _uapp_set_flag(UAPP_FLAG_LOCK_RANGE, x);        };

    inline bool _button_pressed_up() {        return _uapp_flag(UAPP_FLAG_PRESS_UP);        };
    inline void _button_pressed_up(bool x) {  _uapp_set_flag(UAPP_FLAG_PRESS_UP, x);        };
    inline bool _button_pressed_dn() {        return _uapp_flag(UAPP_FLAG_PRESS_DOWN);      };
    inline void _button_pressed_dn(bool x) {  _uapp_set_flag(UAPP_FLAG_PRESS_DOWN, x);      };

    /* Flag manipulation inlines */
    inline uint32_t _uapp_flags() {                return _flags;           };
    inline bool _uapp_flag(uint32_t _flag) {       return (_flags & _flag); };
    inline void _uapp_clear_flag(uint32_t _flag) { _flags &= ~_flag;        };
    inline void _uapp_set_flag(uint32_t _flag) {   _flags |= _flag;         };
    inline void _uapp_set_flag(uint32_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };


  private:
    uint32_t      _flags     = 0;
    uint32_t      _last_draw = 0;
};



class uAppTricorder : public uApp {
  public:
    uAppTricorder();
    ~uAppTricorder();
    int8_t refresh();

  private:
    int8_t _process_user_input();
    void   _redraw_window();
};



class uAppTouchTest : public uApp {
  public:
    uAppTouchTest();
    ~uAppTouchTest();
    int8_t refresh();

  private:
    int8_t _process_user_input();
    void   _redraw_window();
};



class uAppMeta : public uApp {
  public:
    uAppMeta();
    ~uAppMeta();
    int8_t refresh();

  private:
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppRoot : public uApp {
  public:
    uAppRoot();
    ~uAppRoot();
    int8_t refresh();

  private:
    AppID app_page = AppID::TRICORDER;

    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppSynthBox : public uApp {
  public:
    uAppSynthBox();
    ~uAppSynthBox();
    int8_t refresh();

  private:
    uint8_t fft_bars_shown[96];

    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppStandby : public uApp {
  public:
    uAppStandby();
    ~uAppStandby();
    int8_t refresh();

  private:
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppConfigurator : public uApp {
  public:
    uAppConfigurator();
    ~uAppConfigurator();
    int8_t refresh();

  private:
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppComms : public uApp {
  public:
    uAppComms();
    ~uAppComms();
    int8_t refresh();

  private:
    int8_t _process_user_input();
    void   _redraw_window();
};


#endif   // __U_APP_H_
