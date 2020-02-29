/*
*                                                  ---J. Ian Lindsay  2020.02.29
*/
#include <Arduino.h>
#include <StopWatch.h>
#include "Motherflux0r.h"

#ifndef __U_APP_H_
#define __U_APP_H_


/* Class flags */
#define UAPP_FLAG_ENABLED          0x00000001  // This application is active.



/*******************************************************************************
*******************************************************************************/
class uApp {
  public:
    inline bool enabled() {          return _uapp_flag(UAPP_FLAG_ENABLED);        };

    virtual int8_t refresh() =0;

    inline void deliverSliderValue(uint16_t val) {  _slider_pending  = val;   };
    inline void deliverButtonValue(uint16_t val) {  _buttons_pending  = val;  };

    inline void printStopwatch(StringBuilder* out) {  _stopwatch.printDebug(_UA_NAME, out);   };

    static void  setAppActive(AppID);
    static AppID appActive();
    static AppID drawnApp();
    static AppID previousApp();

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

};



#endif   // __U_APP_H_
