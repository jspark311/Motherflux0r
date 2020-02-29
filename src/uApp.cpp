#include <CppPotpourri.h>
#include <Adafruit_SSD1331.h>
#include "uApp.h"
#include "Motherflux0r.h"

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



int8_t uApp::redraw() {
  return 0;
}


uApp* uApp::getActiveAppPtr() {
  return nullptr;
}



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



const char* const uApp::getAppIDString(AppID e) {
  switch (e) {
    case AppID::APP_SELECT:       return "APP_SELECT";
    case AppID::TOUCH_TEST:       return "TOUCH_TEST";
    case AppID::CONFIGURATOR:     return "CONFIGURATOR";
    case AppID::DATA_MGMT:        return "DATA_MGMT";
    case AppID::SYNTH_BOX:        return "SYNTH_BOX";
    case AppID::COMMS_TEST:       return "COMMS_TEST";
    case AppID::META:             return "META";
    case AppID::I2C_SCANNER:      return "I2C_SCANNER";
    case AppID::TRICORDER:        return "TRICORDER";
    case AppID::HOT_STANDBY:      return "HOT_STANDBY";
    case AppID::SUSPEND:          return "SUSPEND";
  }
  return "UNKNOWN";
}

void uApp::listAllApplications(StringBuilder* output) {
  for (uint8_t i = 0; i < 11; i++) {
    output->concatf("%2u: %s\n", i, getAppIDString((AppID) i));
  }
}
