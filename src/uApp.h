/*
*                                                  ---J. Ian Lindsay  2020.02.29
*/
#include <Arduino.h>
#include <StopWatch.h>
#include <Image/Image.h>
#include <FlagContainer.h>
#include <ManuvrDrivers.h>
#include "Storage/DataRecords.h"

#ifndef __U_APP_H_
#define __U_APP_H_

// TODO: Might make this a config or load-contingent value.
#define APP_POLLING_PERIOD        20    // App runs at ~50fps.
#define UAPP_BOOT_INIT_TIMEOUT  2700    // Number of milliseconds to wait for init.


/*******************************************************************************
* Types, defines, and externs
*******************************************************************************/
enum class AppID : uint8_t {
  APP_BOOT = 0,   // Should run once on startup.
  APP_SELECT,     // For choosing the app.
  TOUCH_TEST,     // For diagnostics of the touch pad.
  CONFIGURATOR,   // For tuning all the things.
  DATA_MGMT,      // For managing recorded datasets.
  SYNTH_BOX,      // Sound synthesis from data.
  COMMS_TEST,     // Connecting to the outside world.
  META,           // Shutdown/reboot/reflash, profiles.
  TRICORDER,      // This is the primary purpose of the device.
  MAGNETOMETER,   // The magnetometer detail uApp.
  LOCATOR,        // GPS and location tools.
  HOT_STANDBY,    // Full operation with powered-down UI elements.
  SUSPEND         // Minimal power without an obligatory reboot.
};

/* Application lifecycle state machine positions. */
enum class AppLifecycle : uint8_t {
  UNINITIALIZED = 0,  //
  PREINIT       = 1,  //
  ACTIVE        = 2,  //
  TEARDOWN      = 3,  //
  INACTIVE      = 4   //
};

extern SSD1331 display;


/* Class flags */
#define UAPP_FLAG_LIFECYC_SM_MASK  0x00000007  //
#define UAPP_FLAG_IS_ACTIVE        0x00000010  // This application is active.
#define UAPP_FLAG_CLUTTER_DISPLAY  0x00000020  // User wants more information on the display.
#define UAPP_FLAG_TEXT_OF_VALUE    0x00000040  // User wants more information on the display.
#define UAPP_FLAG_LOCK_RANGE       0x00000080  // Lock the output range.

#define UAPP_FLAG_PRESS_UP         0x04000000  //
#define UAPP_FLAG_PRESS_DOWN       0x08000000  //
#define UAPP_FLAG_PRESS_LEFT       0x10000000  //
#define UAPP_FLAG_PRESS_RIGHT      0x20000000  //
#define UAPP_FLAG_PRESS_OK         0x40000000  //
#define UAPP_FLAG_PRESS_CANCEL     0x80000000  //


/* Modal IDs for the UI. */
#define UAPP_MODAL_NONE                 0x00   // Zero implies no active modal.
#define UAPP_MODAL_TRICORDER_MAG        0x01
#define UAPP_MODAL_TRICORDER_IMU        0x02
#define UAPP_MODAL_TRICORDER_GPS        0x03
#define UAPP_MODAL_TRICORDER_THERMOPILE 0x04
#define UAPP_MODAL_TRICORDER_LIGHT      0x05
#define UAPP_MODAL_TRICORDER_ATMO       0x06
#define UAPP_MODAL_TRICORDER_RANGING    0x07

#define UAPP_MODAL_COMMS_WIFI           0x01
#define UAPP_MODAL_COMMS_BLUETOOTH      0x02
#define UAPP_MODAL_COMMS_LORA           0x03
#define UAPP_MODAL_COMMS_CONSOLE        0x04
#define UAPP_MODAL_COMMS_MLINK          0x05
#define UAPP_MODAL_COMMS_SERIAL         0x06



/*******************************************************************************
*******************************************************************************/
class uApp {
  public:
    int8_t refresh();  // TODO: Might-should be static?

    inline void deliverSliderValue(uint16_t val) {    _slider_pending   = val;  };
    inline void deliverButtonValue(uint16_t val) {    _buttons_pending  = val;  };
    inline void printStopwatch(StringBuilder* out) {  _stopwatch.printDebug(_UA_NAME, out);   };
    inline void resetStopwatch() {                    _stopwatch.reset();                     };
    inline bool isActive() {     return _uapp_flag(UAPP_FLAG_IS_ACTIVE);  };
    inline const char* getAppIDString() {   return _UA_NAME;    };

    inline void    modal(uint8_t m) {      _modal_id = m;       };
    inline uint8_t modal() {               return _modal_id;    };

    static uApp* appActive();
    static uApp* drawnApp();
    static uApp* previousApp();
    static void setAppActive(AppID, uint8_t modal = 0);


  protected:
    const char*   _UA_NAME;
    Image*        FB;
    StopWatch     _stopwatch;
    uint16_t      _slider_current  = 0;
    uint16_t      _slider_pending  = 0;
    uint16_t      _buttons_current = 0;
    uint16_t      _buttons_pending = 0;
    uint8_t       _modal_id        = UAPP_MODAL_NONE;

    uApp(const char* _n, Image* img) : _UA_NAME(_n), FB(img) {};
    virtual ~uApp() {};

    virtual int8_t _lc_on_preinit()      =0;
    virtual int8_t _lc_on_active()       =0;
    virtual int8_t _lc_on_teardown()     =0;
    virtual int8_t _lc_on_inactive()     =0;
    virtual int8_t _process_user_input() =0;
    virtual void   _redraw_window()      =0;

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
    inline bool _button_pressed_l() {         return _uapp_flag(UAPP_FLAG_PRESS_LEFT);      };
    inline void _button_pressed_l(bool x) {   _uapp_set_flag(UAPP_FLAG_PRESS_LEFT, x);      };
    inline bool _button_pressed_u() {         return _uapp_flag(UAPP_FLAG_PRESS_RIGHT);     };
    inline void _button_pressed_u(bool x) {   _uapp_set_flag(UAPP_FLAG_PRESS_RIGHT, x);     };
    inline bool _button_pressed_ok() {        return _uapp_flag(UAPP_FLAG_PRESS_OK);        };
    inline void _button_pressed_ok(bool x) {  _uapp_set_flag(UAPP_FLAG_PRESS_OK, x);        };
    inline bool _button_pressed_x() {         return _uapp_flag(UAPP_FLAG_PRESS_CANCEL);    };
    inline void _button_pressed_x(bool x) {   _uapp_set_flag(UAPP_FLAG_PRESS_CANCEL, x);    };

    void _lc_increment_position();
    inline AppLifecycle _lc_current_position() {
      return (AppLifecycle) (_flags & UAPP_FLAG_LIFECYC_SM_MASK);
    };

    /* Flag manipulation inlines */
    inline uint32_t _uapp_flags() {                return _flags;           };
    inline bool _uapp_flag(uint32_t _flag) {       return (_flags & _flag); };
    inline void _uapp_clear_flag(uint32_t _flag) { _flags &= ~_flag;        };
    inline void _uapp_set_flag(uint32_t _flag) {   _flags |= _flag;         };
    inline void _uapp_set_flag(uint32_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };

    static void redraw_app_window(const char* title, uint8_t pages, uint8_t active_page);


  private:
    uint32_t _flags     = 0;
    uint32_t _last_draw = 0;
};


/*******************************************************************************
* Specific derived classes
*******************************************************************************/

class uAppBoot : public uApp {
  public:
    uAppBoot();
    ~uAppBoot();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();

  private:
    uint32_t _last_init_sent = 0;
    FlagContainer32  _init_sent_flags;
    FlagContainer32  _init_done_flags;
};


class uAppRoot : public uApp {
  public:
    uAppRoot();
    ~uAppRoot();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};



class uAppTricorder : public uApp {
  public:
    uAppTricorder();
    ~uAppTricorder();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();

  private:
    uint8_t _sub_modal = UAPP_MODAL_NONE;

    int8_t _pui_thermal_field();
    int8_t _pui_imu();
    int8_t _pui_baro();
    int8_t _pui_gps();
    int8_t _pui_photometry();
    int8_t _pui_tof();

    void _render_thermal_field();
    void _render_imu();
    void _render_baro();
    void _render_gps();
    void _render_photometry();
    void _render_tof();
};


class uAppMagnetometer : public uApp {
  public:
    uAppMagnetometer();
    ~uAppMagnetometer();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};



class uAppLocation : public uApp {
  public:
    uAppLocation();
    ~uAppLocation();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppTouchTest : public uApp {
  public:
    uAppTouchTest();
    ~uAppTouchTest();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};



class uAppMeta : public uApp {
  public:
    uAppMeta();
    ~uAppMeta();

  protected:
    uint32_t _last_i2c_scan = 0;

    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppSynthBox : public uApp {
  public:
    uAppSynthBox();
    ~uAppSynthBox();

  protected:
    uint8_t fft_bars_shown[96];

    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppStandby : public uApp {
  public:
    uAppStandby();
    ~uAppStandby();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppConfigurator : public uApp {
  public:
    uAppConfigurator();
    ~uAppConfigurator();

    void setActiveConf(const ConfKey K) {   _active_conf = K;  };


  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();

  private:
    ConfKey _active_conf = ConfKey::INVALID;
};


class uAppComms : public uApp {
  public:
    uAppComms();
    ~uAppComms();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};


class uAppDataMgmt : public uApp {
  public:
    uAppDataMgmt();
    ~uAppDataMgmt();

  protected:
    int8_t _lc_on_preinit();
    int8_t _lc_on_active();
    int8_t _lc_on_teardown();
    int8_t _lc_on_inactive();
    int8_t _process_user_input();
    void   _redraw_window();
};

#endif   // __U_APP_H_
