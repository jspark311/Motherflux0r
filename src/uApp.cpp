#include <CppPotpourri.h>
#include <Adafruit_SSD1331.h>
#include "uApp.h"


extern Adafruit_SSD1331 display;


static AppID active_app   = AppID::APP_SELECT;
static AppID drawn_app    = AppID::META;
static AppID app_previous = AppID::APP_SELECT;


void  uApp::setAppActive(AppID new_app) {
  active_app = new_app;
}

AppID uApp::appActive() {      return active_app;     }
AppID uApp::drawnApp() {       return drawn_app;      }
AppID uApp::previousApp() {    return app_previous;   }


/*
* Draws the basics of the UI.
* 96x64
*/
void uApp::redraw_app_window(const char* title, uint8_t pages, uint8_t active_page) {
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




uAppTricorder::uAppTricorder() : uApp("Tricorder") {
}


uAppTricorder::~uAppTricorder(){
}



int8_t uAppTricorder::refresh() {
  int8_t ret = 0;
  _stopwatch.markStart();

  ret = _process_user_input();
  _stopwatch.markStop();
  return ret;
}



int8_t uAppTricorder::_process_user_input() {
  int8_t ret = 0;
  if (_slider_current != _slider_pending) {
    ret++;
  }
  if (_buttons_current != _buttons_pending) {
    ret++;
  }
  return ret;
}
