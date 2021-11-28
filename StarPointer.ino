// BMP390 Temperature and Pressure (Climatic)
#include <bmp3.h>
#include <Adafruit_BMP3XX.h>
#include <bmp3_defs.h>

// BNO055 Libraries Direction (IMU)
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Mini GPS Libraries Location
#include <Adafruit_GPS.h>

// RTC
#include <RTCZero.h>

// StarPointer specific Libraries
#include <AntelopeIT_Astrolib.h>

// StarPointer Display Library including
// Adafruit minTFT display libraries
#include "StarPointerDisplay.h"

const char ACK = 0x06;
const char NAK = 0x15;
const char DEG = 0xF8;
const char CMD_TERM = '#';
const char CMD_INIT = ':';
const int  CMD_COUNT = 2;

enum COMMANDS
{
  Get_RA = 0,
  Get_DEC,
};

const String command_str[] = {
  [Get_RA] = ":GR",
  [Get_DEC] = ":GD"
};

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

/* configuration constants for the miniTFT Wing display */
#define TFT_RST   -1    // we use the seesaw for resetting to save a pin
#define TFT_CS     5
#define TFT_DC     4

// Adafruit miniTFT SeeSaw Display user control
Adafruit_miniTFTWing displayControl;

// Adafruit miniTFT Display Screen
Adafruit_ST7735 displayScreen = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Display Manager
StarPointerDisplay display = StarPointerDisplay(&displayScreen, &displayControl);

// Connect to the GPS on the hardware I2C port
Adafruit_GPS locationSensor(&Wire);

// Connect to the BNO055 on the hardware I2C port
Adafruit_BNO055 directionSensor = Adafruit_BNO055(55, 0x28);

// Temperature and Pressure Sensor
Adafruit_BMP3XX climateSensor;

// Current Data Set
AstroSensorData currentData;

// Real Time Clock
RTCZero rtc;

void setup() {
  currentData.climate.sensorState.updateFrequency = 60000;
  currentData.location.sensorState.updateFrequency = 60000;

  // Setup GPS
  currentData.location.sensorState.active =  locationSensor.begin(0x10);  // The I2C address to use is 0x10
  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude - sufficient for parsing
  locationSensor.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  locationSensor.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate - we're not moving that fast

  // Request updates on antenna status, comment out to keep quiet
  locationSensor.sendCommand(PGCMD_ANTENNA);

  // Setup Position Sensor (BNO055)
  currentData.direction.sensorState.active = directionSensor.begin();
  directionSensor.setExtCrystalUse(true);

  // Setup Climate Sensor (BMP390)
  currentData.climate.sensorState.active = climateSensor.begin_I2C();

  // Set up oversampling and filter initialization
  if (currentData.climate.sensorState.active) {
    climateSensor.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    climateSensor.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    climateSensor.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    climateSensor.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // Initialise Real Time Clock
  rtc.begin();

  // Initialise the display
  if (display.begin()) {
    display.setScreen(0);
  }

  //Initialise serial and wait for port to open:
  Serial.begin(9600);

  // Wait until we have a GPS fix 
  // and get a valid time / date before
  // starting the main loop
  while (currentData.realTime.sensorState.active != true) {
    char c = locationSensor.read();
    // Get the GPS data
    if (locationSensor.newNMEAreceived()) {
      if (locationSensor.parse(locationSensor.lastNMEA())) {
        if (locationSensor.fix && locationSensor.year > 0) {
          rtc.setTime(locationSensor.hour, locationSensor.minute, locationSensor.seconds);
          rtc.setDate(locationSensor.day, locationSensor.month, locationSensor.year);
          currentData.realTime.sensorState.active = true;
        }
      }
    }
  }
}

void loop() {
  currentData.telescopeState.lastUpdate = millis();
  // Need to pump this as fast as we can to avoid overflows.
  char c = locationSensor.read();
  // Get the GPS data
  if (locationSensor.newNMEAreceived()) {
    if (locationSensor.parse(locationSensor.lastNMEA())) {
      currentData.location.sensorState.current = locationSensor.fix;
      currentData.location.sensorState.satellites = (int)locationSensor.satellites;
      if (locationSensor.fix) {
        getLocation(currentData.location);
      }
    }
  }

  if (currentData.realTime.sensorState.active == true) {
    // Get the current real time values
    getDateTime(currentData.realTime);

    // Get the position data
    getDirection(currentData);
  }

  // Get the Climate data
  getClimate(currentData.climate);

  if (currentData.realTime.sensorState.active && currentData.telescopeState.lastUpdate < currentData.direction.sensorState.lastUpdate) {
    // Calculate the star position
    calculateStarPos(currentData);
    currentData.telescopeState.isNewPositionData = true;
  }

  if (Serial && Serial.available() > 0) {
    const String cmd = Serial.readStringUntil(CMD_TERM);
    if (cmd && currentData.direction.sensorState.calibration.system == 3) {
      if (processResponse(cmd, currentData)) {
        currentData.telescopeState.isSynchronised = true;
        currentData.telescopeState.isNewPositionData = false;
      }
    } else {
      Serial.print(NAK);
      currentData.telescopeState.isSynchronised = false;
    }
  }

  display.readControls();
  if (display.lastScreenUpdate == 0 || display.lastScreenUpdate + 1000 < millis()) {
    display.updateScreen(&currentData);
  }
}

// LX200 Communication methods
bool processResponse(String cmd, AstroSensorData& asd) {
  const int cmd_id = interpretCommand(cmd);
  switch (cmd_id)
  {
    case 0:
      Serial.print(toRAString(asd.result.ra));
      asd.telescopeState.lastRead = millis();
      return true;
    case 1:
      Serial.print(toDecString(asd.result.dec));
      asd.telescopeState.lastRead = millis();
      return true;
    default:
      Serial.print(NAK);
      return false;
  }
}

int interpretCommand(String cmd) {
  const String cmd_code = extractCode(cmd);
  int pos = -1;
  for (int i = 0; i < CMD_COUNT; i++)
  {
    if (command_str[i] == cmd_code)
    {
      pos = i;
      break;
    }
  }

  return pos;
}

String extractCode(String cmd) {
  int pos = cmd.indexOf(CMD_INIT);

  if (pos == -1) {
    return "";
  }

  return cmd.substring(pos);
}

/**
   @brief Given a declination value returns a string of the Form "sDD*MM’SS#"
          Example: +56*32'45#

   @param value
   @return String
*/
String toDecString(const double& dec) {
  DegMinSecs dmsDec = toDMS(dec);
  char buffer [10];
  sprintf(buffer, "%+03d*%02d:%02.0f#", dmsDec.degrees, abs(dmsDec.minutes), dmsDec.seconds);
  return buffer;
}

/**
   @brief Given a right ascension value returns a string of the Form "HH:MM:SS#"
          Example: 05:45:30#

   @param value
   @return String
*/
String toRAString(const double& ra) {
  DegMinSecs dmsRa = toDMS(ra / hoursToDegrees);
  char buffer [9];
  sprintf(buffer, "%02d:%02d:%02.0f#", dmsRa.degrees, dmsRa.minutes, dmsRa.seconds);
  return buffer;
}

Equatorial calculateStarPos(AstroSensorData &asd) {
  JulianDate jd = convertToJulianDate2000(asd.dateTime.year, asd.dateTime.month, asd.dateTime.day, asd.dateTime.hour, asd.dateTime.minutes, asd.dateTime.seconds);
  double temperatureKelvin = 0.0;
  double pressure = -1.0;

  if (asd.climate.sensorState.current) {
    temperatureKelvin = convertToKelvin(asd.climate.temperature);
    pressure = asd.climate.pressure;
    asd.climate.sensorState.lastRead = millis();
  }

  Equatorial eq = convertToEquatorial(asd.direction.altitude, asd.direction.azimuth, jd, asd.location.latitude, asd.location.longitude, asd.location.heightAboveSeaLevel, temperatureKelvin, pressure, false, true, true, true, true);
  asd.result = eq;
  asd.telescopeState.lastUpdate = millis();
  return eq;
}

// Direction Sensor (BNO055)  Methods
uint8_t getDirectionSensorCalibration(Direction &direction) {
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  directionSensor.getCalibration(&system, &gyro, &accel, &mag);
  direction.sensorState.calibration.system = system;
  direction.sensorState.calibration.gyro = gyro;
  direction.sensorState.calibration.accel = accel;
  direction.sensorState.calibration.mag = mag;
  return system;
}

bool getDirection(AstroSensorData &asd) {
  sensors_event_t event;
  asd.direction.sensorState.current = directionSensor.getEvent(&event);
  uint8_t returnedSys =  getDirectionSensorCalibration(asd.direction);
  if (asd.direction.sensorState.current) {
    // Get time snapshot
    getDateTime(asd.dateTime);

    // correct for sensor orientation - clockwise rotation = positive
    // Northern hemisphere anticlockwise = increase alt
    asd.direction.altitude = -1 * event.orientation.z;  // 0 - 90

    if (asd.direction.altitude > 90.0 ) {
      asd.direction.altitude  = 180.0 - asd.direction.altitude ;
    } else if (asd.direction.altitude < -90.0 ) {
      asd.direction.altitude  = -180.0 - asd.direction.altitude ;
    }

    asd.direction.azimuth = event.orientation.x;   // 0 - 360
    asd.direction.sensorState.lastUpdate = millis();
  }

  return asd.direction.sensorState.current;
}


// GPS Sensor Methods
bool getLocation(Location &location) {
  if (location.sensorState.lastUpdate == 0 || location.sensorState.lastUpdate + location.sensorState.updateFrequency < millis()) {
    location.sensorState.lastUpdate = millis();
    location.heightAboveSeaLevel = locationSensor.altitude;
    uint latDegrees = static_cast<int>(locationSensor.latitude / 100);
    float latDec = fmod(locationSensor.latitude, 100) / 60 + latDegrees;
    location.latitude = locationSensor.lat == 'W' ? -1 * latDec : latDec;
    uint lonDegrees = static_cast<int>(locationSensor.longitude / 100);
    float lonDec = fmod(locationSensor.longitude, 100) / 60 + lonDegrees;
    location.longitude = locationSensor.lon == 'S' ? -1 * lonDec : lonDec;
  }
}

// Climatic Sesnsor (BMP3x) Methods
bool getClimate(Climatic &climate) {
  // If there is an active climate sensor
  if (climate.sensorState.active && (climate.sensorState.lastUpdate == 0 || climate.sensorState.lastUpdate + climate.sensorState.updateFrequency < millis())) {
    // Get Temperature and Pressure
    climate.sensorState.current = climateSensor.performReading();

    // If New readings retrieved then update readings;
    if (climate.sensorState.current) {
      climate.sensorState.lastUpdate = millis();
      climate.pressure = climateSensor.pressure / 100.0;
      climate.temperature = climateSensor.temperature;
      return true;
    }
  }
  return false;
}

/**
   @brief Get the Date Time object

   @param asd AstroSensorData.dateTime to pupulate with date time snapshot
*/
void getDateTime(DateTime &dateTime) {
  dateTime.sensorState.lastUpdate = millis();
  dateTime.year = 2000 + rtc.getYear();
  dateTime.month = rtc.getMonth();
  dateTime.day = rtc.getDay();
  dateTime.hour = rtc.getHours();
  dateTime.minutes = rtc.getMinutes();
  dateTime.seconds = rtc.getSeconds();
}

/**
   @brief Simple utility function Converts a temperature expressed in degrees celsius to degrees Kelvin

   @param celsius
   @return double temperature in degrees Kelvin
*/
double convertToKelvin(const double celsius) {
  return celsius + 273.15;
}
