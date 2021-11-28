#include "StarPointerDisplay.h"

StarPointerDisplay::StarPointerDisplay(Adafruit_ST7735 *screen, Adafruit_miniTFTWing *control)
{
  _screen = screen;
  _control = control;
  _currentScreen = -1;
  _numberOfScreens = 5;

  ScreenDefinition defaultScreen;
  defaultScreen.drawTemplate  = &StarPointerDisplay::overviewScreenTemplate;
  defaultScreen.drawData = &StarPointerDisplay::overviewDrawData;
  _screenDefinitions[0] = defaultScreen;

  ScreenDefinition gpsScreen;
  gpsScreen.drawTemplate  = &StarPointerDisplay::gpsScreenTemplate;
  gpsScreen.drawData = &StarPointerDisplay::gpsDrawData;
  _screenDefinitions[1] = gpsScreen;

  ScreenDefinition directionScreen;
  directionScreen.drawTemplate  = &StarPointerDisplay::directionScreenTemplate;
  directionScreen.drawData = &StarPointerDisplay::directionDrawData;
  _screenDefinitions[2] = directionScreen;

  ScreenDefinition climateScreen;
  climateScreen.drawTemplate  = &StarPointerDisplay::climateScreenTemplate;
  climateScreen.drawData = &StarPointerDisplay::climateDrawData;
  _screenDefinitions[3] = climateScreen;

  ScreenDefinition timeScreen;
  timeScreen.drawTemplate  = &StarPointerDisplay::timeScreenTemplate;
  timeScreen.drawData = &StarPointerDisplay::timeDrawData;
  _screenDefinitions[4] = timeScreen;
}

bool StarPointerDisplay::begin()
{
  if (_control->begin()) {
    _control->tftReset();
    _control->setBacklight(TFTWING_BACKLIGHT_ON); //set the backlight fully on
    isInitialised = true;
  }

  _screen->initR(INITR_MINI160x80);   // initialize a ST7735S chip, mini display
  _screen->setRotation(3);

  if (isInitialised) {
    _screen->fillScreen(ST77XX_BLACK);
  } else {
    _screen->fillScreen(ST77XX_RED);
  }

  return isInitialised;
}

void StarPointerDisplay::drawFrame(uint16_t frameColor) {
  _screen->drawRect(0, 0, 160, 80, frameColor);
  _screen->drawLine(145, 0, 145, 80, frameColor);
}

void StarPointerDisplay::drawControls(bool fillPrevious, bool fillNext, uint16_t controlColor) {
  // Draw back arrow
  if (fillPrevious) {
    _screen->fillTriangle(149, 12, 152, 5, 155, 12, controlColor);
  } else {
    _screen->drawTriangle(149, 12, 152, 5, 155, 12, controlColor);
  }

  // Draw forward arrow
  if (fillNext) {
    _screen->fillTriangle(149, 68, 152, 75, 155, 68, controlColor);
  } else {
    _screen->drawTriangle(149, 68, 152, 75, 155, 68, controlColor);
  }
}

bool StarPointerDisplay::setScreen(int screenId)
{
  if (screenId < 0 || screenId >= _numberOfScreens || screenId == _currentScreen ) {
    return false;
  }

  _screen->fillScreen(_backgroundColor);
  _currentScreen = screenId;

  // _screenDefinitions[this->_currentScreen].drawTemplate();
  (this->*_screenDefinitions[this->_currentScreen].drawTemplate)();
  drawFrame(_screenColor);
  drawControls(_currentScreen > 0, _currentScreen < _numberOfScreens - 1, _screenColor);

  lastScreenUpdate = 0;
  return true;
}

bool StarPointerDisplay::readControls() {
  uint32_t buttons = _control->readButtons();
  drawControls(false, false, _screenColor);
  if (! (buttons & TFTWING_BUTTON_DOWN)) {
    drawControls(true, false, _screenColor);
    return nextScreen();
  }

  if (! (buttons & TFTWING_BUTTON_UP)) {
    drawControls(false, true, _screenColor);
    return previousScreen();
  }
}

bool StarPointerDisplay::nextScreen() {
  return setScreen(_currentScreen + 1);
}

bool StarPointerDisplay::previousScreen() {
  return setScreen(_currentScreen - 1);
}

bool StarPointerDisplay::updateScreen(AstroSensorData* data) {
  (this->*_screenDefinitions[this->_currentScreen].drawData)(data);
  lastScreenUpdate = millis();
}

void StarPointerDisplay::sendToScreen(uint screenX, uint screenY, const char *format, ...) {
  char buffer[20];
  va_list args;
  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end(args);
  _screen->setCursor(screenX, screenY);
  _screen->print(buffer);
}

bool StarPointerDisplay::overviewScreenTemplate() {
  _screenColor = 0x1B51;
  _screen->setTextSize(1);
  _screen->setTextColor(_screenColor, _backgroundColor);

  _screen->drawLine(96, 0, 96, 80, _screenColor);

  _screen->setCursor(5, 5);
  _screen->print("Alt:");
  _screen->setCursor(5, 15);
  _screen->print("Az :");

  _screen->setCursor(5, 25);
  _screen->print("Dec:");
  _screen->setCursor(5, 35);
  _screen->print("Ra :");

  _screen->setCursor(5, 45);
  _screen->print("UTC:");

  _screen->setCursor(5, 55);
  _screen->print("Tmp:");

  _screen->setCursor(5, 65);
  _screen->print("Bar:");

  _screen->setCursor(100, 5);
  _screen->print("Pos:");

  _screen->setCursor(100, 15);
  _screen->print("GPS:");

  _screen->setCursor(100, 25);
  _screen->print("RTC:");

  _screen->setCursor(100, 35);
  _screen->print("Met:");

  _screen->setCursor(100, 45);
  _screen->print("Syn:");
  return true;
}

bool StarPointerDisplay::gpsScreenTemplate() {
  _screenColor = 0x1B51;
  _screen->setTextSize(1);
  _screen->setTextColor( _backgroundColor, _screenColor);
  _screen->fillRect(1, 1, 144, 15, _screenColor);
  _screen->setCursor(5, 5);
  _screen->print("GPS Location");
  _screen->setTextColor(_screenColor, _backgroundColor);
  _screen->setCursor(5, 20);
  _screen->print("Latitude:");
  _screen->setCursor(5, 35);
  _screen->print("Longitude:");

  _screen->setCursor(5, 50);
  _screen->print("Altitude:");

  _screen->setCursor(5, 65);
  _screen->print("Satellites:");

  return true;
}

bool StarPointerDisplay::climateScreenTemplate() {
  _screenColor = 0x1B51;
  _screen->setTextSize(1);
  _screen->setTextColor( _backgroundColor, _screenColor);
  _screen->fillRect(1, 1, 144, 15, _screenColor);
  _screen->setCursor(5, 5);
  _screen->print("Climate");
  _screen->setTextColor(_screenColor, _backgroundColor);
  _screen->setCursor(5, 20);
  _screen->print("Temperature:");
  _screen->setCursor(5, 35);
  _screen->print("Pressure:");
  return true;
}


bool StarPointerDisplay::timeScreenTemplate() {
  _screenColor = 0x1B51;
  _screen->setTextSize(1);
  _screen->setTextColor( _backgroundColor, _screenColor);
  _screen->fillRect(1, 1, 144, 15, _screenColor);
  _screen->setCursor(5, 5);
  _screen->print("Real Time Clock");
  _screen->setTextColor(_screenColor, _backgroundColor);
  _screen->setCursor(5, 20);
  _screen->print("Date:");
  _screen->setCursor(5, 35);
  _screen->print("Time:");
  return true;
}

bool StarPointerDisplay::directionScreenTemplate() {
  _screenColor = 0x1B51;
  // claibration data grid
  _screen->drawLine(1, 48, 145, 48, _screenColor);
  _screen->drawLine(1, 64, 145, 64, _screenColor);
  _screen->drawLine(35, 48, 35, 80, _screenColor);
  _screen->drawLine(70, 48, 70, 80, _screenColor);
  _screen->drawLine(105, 48, 105, 80, _screenColor);

  _screen->setTextSize(1);
  _screen->setTextColor( _backgroundColor, _screenColor);
  _screen->fillRect(1, 1, 144, 15, _screenColor);
  _screen->setCursor(5, 5);
  _screen->print("Direction");
  _screen->setTextColor(_screenColor, _backgroundColor);
  _screen->setCursor(5, 20);
  _screen->print("Altitude:");
  _screen->setCursor(5, 35);
  _screen->print("Azimuth :");
  _screen->setCursor(5, 53);
  _screen->print("Sys.");
  _screen->setCursor(40, 53);
  _screen->print("Gyr.");
  _screen->setCursor(75, 53);
  _screen->print("Acc.");
  _screen->setCursor(110, 53);
  _screen->print("Mag.");
  return true;
}

void StarPointerDisplay::printAltitude(uint screenX,  uint screenY, AstroSensorData *data) {
  DegMinSecs dmsAlt = toDMS(data->direction.altitude);
  sendToScreen( screenX, screenY, "% 03d%c%02d'%02.0f%c", dmsAlt.degrees, 247, dmsAlt.minutes, dmsAlt.seconds, 34);
}

void StarPointerDisplay::printAzimuth(uint screenX, uint screenY, AstroSensorData *data) {
  DegMinSecs dmsAz = toDMS(data->direction.azimuth);
  sendToScreen(screenX, screenY, "%3d%c%02d'%02.0f%c", dmsAz.degrees, 247, dmsAz.minutes, dmsAz.seconds, 34);
}

void StarPointerDisplay::printDeclination(uint screenX,  uint screenY, AstroSensorData *data) {
  DegMinSecs dmsDec = toDMS(data->result.dec);
  sendToScreen(screenX, screenY, "% 03d%c%02d'%02.0f%c", dmsDec.degrees, 247, abs(dmsDec.minutes), dmsDec.seconds, 34);
}

void StarPointerDisplay::printRightAscension( uint screenX, uint screenY, AstroSensorData *data) {
  DegMinSecs dmsRa = toDMS(data->result.ra / hoursToDegrees);
  sendToScreen(screenX, screenY,  " %02d%c%02d'%02.0f%c", dmsRa.degrees, 247, dmsRa.minutes, dmsRa.seconds, 34);
}

void StarPointerDisplay::printCalculationTime(uint screenX, uint screenY, AstroSensorData *data) {
  // Show Time
  sendToScreen(screenX, screenY, "%02d:%02d:%02.0f", data->dateTime.hour, data->dateTime.minutes, data->dateTime.seconds);
}

void StarPointerDisplay::printTime(uint screenX, uint screenY, AstroSensorData *data) {
  // Show Time
  sendToScreen(screenX, screenY, "%02d:%02d:%02.0f", data->realTime.hour, data->realTime.minutes, data->realTime.seconds);
}

void StarPointerDisplay::printDate(uint screenX, uint screenY, AstroSensorData *data) {
  // Show Date
  sendToScreen(screenX, screenY, "%02d/%02d/%4d", data->realTime.day, data->realTime.month, data->realTime.year);
}

void StarPointerDisplay::printTemperature(uint screenX, uint screenY, AstroSensorData* data) {
  sendToScreen(screenX, screenY, "%4.1f%cC", data->climate.temperature, 247);
}

void StarPointerDisplay::printPressure(uint screenX, uint screenY, AstroSensorData* data) {
  sendToScreen( screenX, screenY, "%7.2fhPa", data->climate.pressure);
}

void StarPointerDisplay::printLatitude(uint screenX,  uint screenY, AstroSensorData *data) {
  char latDir = data->location.latitude < 0 ? 'S' : 'N';
  DegMinSecs dmsDec = toDMS(abs(data->location.latitude));
  sendToScreen(screenX, screenY, "% 03d%c%02d'%02.0f%c%c", dmsDec.degrees, 247, abs(dmsDec.minutes), dmsDec.seconds, 34, latDir);
}

void StarPointerDisplay::printLongitude(uint screenX,  uint screenY, AstroSensorData *data) {
  char lonDir = data->location.longitude < 0 ? 'E' : 'W';
  DegMinSecs dmsDec = toDMS(abs(data->location.longitude));
  sendToScreen(screenX, screenY, "% 03d%c%02d'%02.0f%c%c", dmsDec.degrees, 247, abs(dmsDec.minutes), dmsDec.seconds, 34, lonDir);
}

void StarPointerDisplay::printHeightAboveSea(uint screenX, uint screenY, AstroSensorData* data) {
  sendToScreen(screenX, screenY, "%6.2fm", data->location.heightAboveSeaLevel);
}

bool StarPointerDisplay::overviewDrawData(AstroSensorData *data) {
  const uint dataColumn = 30;

  // Show Measured Alt/Az
  printAltitude(dataColumn, 5, data);
  printAzimuth(dataColumn, 15, data);

  // Show calculated Dec/Ra
  printDeclination(dataColumn, 25, data);
  printRightAscension(dataColumn, 35, data);

  // Show time hh:mm:ss
  printCalculationTime(dataColumn, 45, data);

  // Show Temperature
  printTemperature(dataColumn, 55, data);

  // Show Pressure
  printPressure( dataColumn, 65, data);

  // Show calibration status
  _screen->setCursor(125, 5);
  _screen->print(data->direction.sensorState.calibration.system, DEC);

  // Show Time Init
  if (data->realTime.sensorState.active) {
    _screen->fillCircle(127, 29, 2, _screenColor);
  } else  {
    _screen->fillCircle(127, 29, 2, _backgroundColor);
  }

  // Show GPS Init
  if (data->location.sensorState.current) {
    _screen->fillCircle(127, 19, 2, _screenColor);
  } else {
    _screen->fillCircle(127, 19, 2, _backgroundColor);
  }

  // Show Climate Init
  if (data->climate.sensorState.current) {
    _screen->fillCircle(127, 39, 2, _screenColor);
  } else {
    _screen->fillCircle(127, 39, 2, _backgroundColor);
  }

  // Show Telescope Display Sync'd
  if (data->telescopeState.isSynchronised || data->telescopeState.isNewPositionData ) {
    if (data->telescopeState.isNewPositionData == true) {
      _screen->fillCircle(127, 49, 2, _screenColor);
    } else {
      _screen->drawCircle(127, 49, 2, _screenColor);
    }
  } else  {
    _screen->fillCircle(127, 49, 2, _backgroundColor);
  }
  return true;
}

bool StarPointerDisplay::gpsDrawData(AstroSensorData *data) {
  const uint dataColumn = 75;

  printLatitude(dataColumn,  20, data);
  printLongitude(dataColumn, 35, data);
  printHeightAboveSea(dataColumn, 50, data);
  _screen->setCursor(dataColumn, 65);
  _screen->print(data->location.sensorState.satellites, DEC);
}

bool StarPointerDisplay::climateDrawData(AstroSensorData *data) {
  printTemperature(80,  20, data);
  printPressure(70, 35, data);
  return true;
}

bool StarPointerDisplay::timeDrawData(AstroSensorData *data) {
  const uint dataColumn = 35;
  printDate(dataColumn, 20, data);
  printTime(dataColumn, 35, data);
  return true;
}

bool StarPointerDisplay::directionDrawData(AstroSensorData *data) {
  const uint dataColumn = 60;
  const uint calibRow = 68;
  printAltitude(dataColumn, 20, data);
  printAzimuth(dataColumn, 35, data);
  _screen->setCursor(15, calibRow);
  _screen->print(data->direction.sensorState.calibration.system);
  _screen->setCursor(50, calibRow);
  _screen->print(data->direction.sensorState.calibration.gyro);
  _screen->setCursor(85, calibRow);
  _screen->print(data->direction.sensorState.calibration.accel);
  _screen->setCursor(120, calibRow);
  _screen->print(data->direction.sensorState.calibration.mag);
  return true;
}
