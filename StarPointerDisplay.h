#ifndef __ANTELOPEIT_STARPOINTERDISPLAY_H__
#define __ANTELOPEIT_STARPOINTERDISPLAY_H__

#include <stdarg.h>

// Adafruit miniTFT Display Libraries
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "Adafruit_miniTFTWing.h"

// StarPointer specific Libraries
#include <AntelopeIT_Astrolib.h>

typedef struct SensorMetaData {
  bool active = false;
  bool current = false;
  u_int32_t lastRead = 0;
  u_int32_t lastUpdate = 0;
  u_int32_t updateFrequency;
} SensorMetaData;

typedef struct DateTime {
  SensorMetaData sensorState;
  int year;
  int month;
  int day;
  int hour;
  int minutes;
  double seconds;
} DateTime;

typedef struct LocationSensorMetaData : SensorMetaData {
  int satellites;
} LocationSensorMetaData;

typedef struct Location {
  LocationSensorMetaData sensorState;
  double heightAboveSeaLevel;
  double latitude;
  double longitude;
} Location;

typedef struct Climatic {
  SensorMetaData sensorState;
  double temperature;
  double pressure;
} Climatic;

typedef struct DirectionCalibration {
  uint8_t system;
  uint8_t gyro;
  uint8_t accel;
  uint8_t mag;
} DirectionCalibration;

typedef struct DirectionSensorMetaData : SensorMetaData {
  DirectionCalibration calibration;
} DirectionSensorMetaData;

typedef struct Direction {
  DirectionSensorMetaData sensorState;
  double azimuth;
  double altitude;
} Direction;

typedef struct TelescopeMetaData : SensorMetaData {
  bool isSynchronised;
  bool isNewPositionData;
} TelescopeMetaData;

typedef struct AstroSensorData {
  TelescopeMetaData telescopeState;
  DateTime dateTime;
  DateTime realTime;
  Location location;
  Climatic climate;
  Direction direction;
  Equatorial result;
} AstroSensorData;

class StarPointerDisplay
{
    typedef bool (StarPointerDisplay::*drawTemplate_t)();
    typedef bool (StarPointerDisplay::*drawData_t)(AstroSensorData *data);
    typedef struct ScreenDefinition {
      drawTemplate_t drawTemplate;
      drawData_t drawData;
    } ScreenDefinition;

  public:
    StarPointerDisplay(Adafruit_ST7735 *screen, Adafruit_miniTFTWing *control);
    bool begin();
    bool setScreen(int screenId);
    bool updateScreen(AstroSensorData *data);
    bool isInitialised = false;
    bool readControls();
    uint32_t lastScreenUpdate = 0;
  private:
    int _currentScreen;
    int _numberOfScreens;
    uint16_t _backgroundColor = ST77XX_BLACK;
    uint16_t _screenColor = ST77XX_BLUE;
    Adafruit_ST7735 *_screen;
    Adafruit_miniTFTWing *_control;
    ScreenDefinition _screenDefinitions[5];
    bool nextScreen();
    bool previousScreen();
    void drawControls(bool fillPrevious, bool fillNext, uint16_t controlColor);
    void drawFrame(uint16_t frameColor);
    // General screen writing utility
    void sendToScreen(uint screenX, uint screenY, const char *format, ...);
    // Overview Screen
    bool overviewDrawData(AstroSensorData *data);
    bool overviewScreenTemplate();
    // GPS Screen
    bool gpsDrawData(AstroSensorData *data);
    bool gpsScreenTemplate();
    // Climate Screen
    bool climateDrawData(AstroSensorData *data);
    bool climateScreenTemplate();
    // Time Screen
    bool timeDrawData(AstroSensorData *data);
    bool timeScreenTemplate();
    // Direction Screen
    bool directionDrawData(AstroSensorData *data);
    bool directionScreenTemplate();
    // Data Values
    void printAltitude(uint screenX,  uint screenY, AstroSensorData *data);
    void printAzimuth(uint screenX, uint screenY, AstroSensorData *data);
    void printDeclination(uint screenX,  uint screenY, AstroSensorData *data);
    void printRightAscension( uint screenX,  uint screenY, AstroSensorData *data);
    void printCalculationTime(uint screenX, uint screenY, AstroSensorData *data);
    void printTime(uint screenX, uint screenY, AstroSensorData *data);
    void printDate(uint screenX, uint screenY, AstroSensorData *data);
    void printTemperature(uint screenX,  uint screenY, AstroSensorData* data);
    void printPressure(uint screenX,  uint screenY, AstroSensorData* data);
    void printHeightAboveSea(uint screenX, uint screenY, AstroSensorData* data);
    void printLatitude(uint screenX,  uint screenY, AstroSensorData *data);
    void printLongitude(uint screenX,  uint screenY, AstroSensorData *data);
};

#endif
