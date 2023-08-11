#include <LibAPRS.h>        //Modified version of https://github.com/markqvist/LibAPRS
#include <SoftwareSerial.h>
#include <TinyGPS++.h>      //https://github.com/mikalhart/TinyGPSPlus
#include <LowPower.h>       //https://github.com/rocketscream/Low-Power
#include <Wire.h>
#include <Adafruit_BMP085.h>//https://github.com/adafruit/Adafruit-BMP085-Library
#include <avr/wdt.h>

#define RfPDPin     19
#define GpsVccPin   18
#define RfPwrHLPin  21
#define RfPttPin    20
#define BattPin     A2
#define PIN_DRA_RX  22
#define PIN_DRA_TX  23

// Definitions for RRC3 altimeter
#define RRC3_UART_RX 6 // labeled MISO on PCB
#define RRC3_UART_TX 5 // labeled MOSI on PCB
#define RRC3_buff_max_size 17 // 16 telemetry data bits plus carriage return 0x0D at end
#define RRC3_altitude_max_size 6 // number of characters in altitude per RRC3 user manual
#define RRC3_speed_max_size 5 // number of characters in speed per RRC3 user manual
#define ASCII_CR 0x0D // Carriage Return

// Number of speed values to save and compare when determining landing event.
// This number multiplied by RRC3 telemetry rate determines how long to check speed values when
// determining if landing event occured.
#define recentSpeedCount 10

#define landingSpeedThreshold 2 // value in mph.  (3fps ~= 2mph) 
#define launchAltitudeThreshold 300 // feet

// Preset transmit intervals to dynamically chose from depending on flight status.
// These do not include preamble or PTT delay.
#define preLaunchXmitInterval   15000 // 15 seconds
#define postLaunchXmitInterval  1500  // 1.5 seconds
#define postApogeeXmitInterval  5000  // 5 seconds
#define postLandingXmitInterval 60000 // 1 minute
#define defaultXmitInterval  5000  // 5 seconds

// PTT delay.  Needs to be longer when transmissions are farther apart, but can
// be shortened for frequent transmissions.
#define PTTDelayLong  1000  // 1 seconds
#define PTTDelayShort 350   // 0.35 seconds

#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH false

#define GpsON         digitalWrite(GpsVccPin, LOW)//PNP
#define GpsOFF        digitalWrite(GpsVccPin, HIGH)
#define RfON          digitalWrite(RfPDPin, HIGH)
#define RfOFF         digitalWrite(RfPDPin, LOW)
#define RfPwrHigh     pinMode(RfPwrHLPin, INPUT)
#define RfPwrLow      pinMode(RfPwrHLPin, OUTPUT);digitalWrite(RfPwrHLPin, LOW)
#define RfPttON       digitalWrite(RfPttPin, HIGH)//NPN
#define RfPttOFF      digitalWrite(RfPttPin, LOW)
#define AprsPinInput  pinMode(12,INPUT);pinMode(13,INPUT);pinMode(14,INPUT);pinMode(15,INPUT)
#define AprsPinOutput pinMode(12,OUTPUT);pinMode(13,OUTPUT);pinMode(14,OUTPUT);pinMode(15,OUTPUT)

//#define DEVMODE // Development mode. Uncomment to enable for debugging.
bool firstTransmit = true;

// APRS callsign & frequency information
char  CallSign[7]="N0CALL";
int   CallNumber=11;
char  Symbol='O';
bool  alternateSymbolTable = true ; //false = '/' , true = '\'
char Frequency[9]="144.3900"; //default frequency. 144.3900 for US, 144.8000 for Europe

// Timer variables
unsigned int transmitInterval = defaultXmitInterval;    // Default transmit interval at startup
unsigned int lastTransmitStartTime = 0;
unsigned int gpsInterval = 5000;        // Time to wait in milliseconds between reading GPS location
unsigned int lastGpsStartTime = 0;

// APRS path configuration
byte  Wide1=1; // 1 for WIDE1-1 path
byte  Wide2=1; // 1 for WIDE2-1 path
int pathSize=0; // 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N

// Telemetry buffer used to build telemetry string before transmission
static char telemetry_buff[23];

// Setup for connected devices
TinyGPSPlus gps;
Adafruit_BMP085 bmp;
String serialCommand;
SoftwareSerial RRC3_UART(RRC3_UART_RX, RRC3_UART_TX);

// Status and variables for telemetry information
bool drogueDeployed = false;
bool drogueContinuity = false;
bool mainDeployed = false;
bool mainContinuity = false;
bool altimeterUartConnected = false;
bool launchDetected = false;
bool landingDetected = false;
bool gpsValid = false;
float altitude = 0;
float recentSpeed[recentSpeedCount];

void setup() {
  wdt_enable(WDTO_8S);
  analogReference(INTERNAL2V56);
  pinMode(RfPDPin, OUTPUT);
  pinMode(GpsVccPin, OUTPUT);
  pinMode(RfPwrHLPin, OUTPUT);
  pinMode(RfPttPin, OUTPUT);
  pinMode(BattPin, INPUT);
  pinMode(PIN_DRA_TX,INPUT);

  GpsON;
  RfPwrLow;   // change to RfPwrHigh for 1.0 watt operation
  RfPttOFF;
  RfON;       // turn radio module on
  
  Serial.begin(57600);  // begin serial comms on serial <--> USB port
  Serial1.begin(9600);  // begin serial comms with GPS module
#if defined(DEVMODE)
  Serial.println(F("Start"));
#endif

  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(CallSign,CallNumber);
  APRS_setDestination("APLIGA", 0);
  APRS_setMessageDestination("APLIGA", 0);
  APRS_setPath1("WIDE1", Wide1);
  APRS_setPath2("WIDE2", Wide2);
  APRS_useAlternateSymbolTable(alternateSymbolTable); 
  APRS_setSymbol(Symbol);
  //increase following value (for example to 500UL) if you experience packet loss/decode issues. 
  APRS_setPreamble(350UL); // was 350  
  APRS_setPathSize(pathSize);
  AprsPinInput;

  setDefaultPosition();

  delay(2000);  // delay for RF module to turn on
  configDra818(Frequency);

  bmp.begin();
  
  RRC3_UART.begin(9600);
  RRC3_UART.listen();
}

void loop()
  {
      wdt_reset();
      unsigned int currentMillis = millis();

      updateRRC3Data();

      // Update GPS location on specified intervals
      if(currentMillis - lastGpsStartTime >= gpsInterval)
      {
        lastGpsStartTime = millis();

        updateGpsData(1000);
        gpsDebug();

        gpsValid = gps.location.isValid();

        if (gpsValid)
        {
          updatePosition();
        }
        else
        {
#if defined(DEVMODE)
          Serial.println(F("GPS Signal Invalid"));
#endif
        } 
      }

      // Send APRS packet on specified transmit intervals.
      // Transmit one more time at faster interval before slowing transmit interval down to transmit launch/deployment event changes faster.
      if(getXmitInterval() < transmitInterval)
      {
        transmitInterval = getXmitInterval();
      }
      if((currentMillis - lastTransmitStartTime >= transmitInterval) || firstTransmit)
      {
#if defined(DEVMODE)
        Serial.print("xmit interval: ");
        Serial.println(transmitInterval);
#endif
        lastTransmitStartTime = millis();
        
        // Stopping RRC3 serial communications is required for radio module to function properly.
        // Without stopping RRC3 serial communications, radio module will still transmit as observed by SDR.
        // However, transmission is incorrect and is not decoded by DireWolf.
        RRC3_UART.end();

        updateTelemetry();
        sendLocation();

        // Reset the altimeter UART connection status to false.  If it stays connected until next
        // APRS transmission, the status flag will have been modified to true.
        altimeterUartConnected = false; 

        // Restart RRC3 serial communications
        RRC3_UART.begin(9600);
        RRC3_UART.listen();
        
        transmitInterval = getXmitInterval();

        firstTransmit = false;
      }

      Serial.flush();      
}

unsigned int getXmitInterval()
{
  if(landingDetected)
  {
    return postLandingXmitInterval; 
  }
  else if(launchDetected && (drogueDeployed || mainDeployed))
  {
    return postApogeeXmitInterval;
  }
  else if(launchDetected && !drogueDeployed)
  {
    return postLaunchXmitInterval;
  }
  else if(!launchDetected)
  {
    return preLaunchXmitInterval;
  }
  else
  {
#if defined(DEVMODE)
    Serial.println("Failsafe transmit rate set");    
#endif
    return defaultXmitInterval;  // Failsafe.  This line should never be executed.
  }
}

void aprs_msg_callback(struct AX25Msg *msg) {
  //do not remove this function, necessary for LibAPRS
}

byte configDra818(char *freq)
{
  SoftwareSerial Serial_dra(PIN_DRA_RX, PIN_DRA_TX);
  Serial_dra.begin(9600);
  char cmd[50];
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  Serial_dra.println(cmd);
  return 0;
}

void updateRRC3Data()
{
#if defined(DEVMODE)
  if (RRC3_UART.overflow())
  {
    Serial.println("RRC3 software serial buffer overflow");    
  }
#endif

  bool RRC3DataReceived = false;
  bool RRC3DataValid = true;
  
  char RRC3DrogueBit = '?';
  char RRC3MainBit = '?';

  // +1 in buffer sizes leaves room for terminating character
  char RRC3_buff[RRC3_buff_max_size+1];  
  char RRC3AltitudeBytes[RRC3_altitude_max_size+1];
  char RRC3SpeedBytes[RRC3_speed_max_size+1];

  // set temp buffers to known values (NULL)
  for (int i=0; i<=RRC3_buff_max_size; i++)
  {
    RRC3_buff[i] = NULL;
  }
  for (int i=0; i<=RRC3_altitude_max_size; i++)
  {
    RRC3AltitudeBytes[i] = NULL;
  }
  for (int i=0; i<=RRC3_speed_max_size; i++)
  {
    RRC3SpeedBytes[i] = NULL;
  }

  // Read UART.  Packet end is Carriage Return per RRC3 user manual
  int pos = 0;
  while (RRC3_UART.available() > 0 && pos < RRC3_buff_max_size)
  {
    RRC3_buff[pos] = RRC3_UART.read();
    
    if (RRC3_buff[pos] == ASCII_CR)
    {
      RRC3DataReceived = true;
#if defined(DEVMODE)
      Serial.println("CR received");
#endif
      break; // carriage return signals end of packet
    }      
    pos++;
  }

  if (RRC3DataReceived)
  {
    altimeterUartConnected = true;

#if defined(DEVMODE)
    Serial.print("RRC3 Data: ");
    printStr(RRC3_buff, RRC3_buff_max_size);  
    Serial.println();
#endif

    // Validate correct number of comma separators in RRC3 data.
    // Should be total of 2 when only altitude, speed, and events are selected in RRC3 config.
    int commaCount = 0;
    for (int i=0; i<RRC3_buff_max_size; i++)
    {
      if (RRC3_buff[i] == ',')
      {
        commaCount++;
      }
    }
    if (commaCount != 2)
    {
        RRC3DataValid = false;
    }
#if defined(DEVMODE)
    Serial.print("comma count: ");
    Serial.println(commaCount);
#endif

    // RRC3_buff format is altitude, speed, dma
    char* RRC3data = strtok(RRC3_buff, ",");
    if (strlen(RRC3data) > RRC3_altitude_max_size)
    {
      RRC3DataValid = false;
    }
    else
    {
      strcpy(RRC3AltitudeBytes, RRC3data);
    }
    RRC3data = strtok(0, ",");  
    if (strlen(RRC3data) > RRC3_speed_max_size)
    {
      RRC3DataValid = false;
    }    
    else
    {
      strcpy(RRC3SpeedBytes, RRC3data);
    }
    RRC3data = strtok(0, ",");  

#if defined(DEVMODE)
    Serial.print("RRC3AltitudeBytes: ");
    printStr(RRC3AltitudeBytes, RRC3_altitude_max_size);  
    Serial.println();
    Serial.print("RRC3SpeedBytes: ");
    printStr(RRC3SpeedBytes, RRC3_speed_max_size);  
    Serial.println();
#endif

    // Validate RRC3 altitude and speed
    char * e_altitude;
    char * e_speed;
    int errno = 0;    
    double tempAltitude = strtod(RRC3AltitudeBytes, &e_altitude);
    double tempSpeed = strtod(RRC3SpeedBytes, &e_speed);
    if (*e_altitude != '\0' ||  errno != 0)
    {
      RRC3DataValid = false;
    }    
    else
    {
      altitude = tempAltitude;  
      if(altitude > launchAltitudeThreshold)   
      {
        launchDetected = true;
      } 
    }
    if (*e_speed != '\0' ||  errno != 0)
    {
      RRC3DataValid = false;
    }    
    else
    {
      pushSpeed(tempSpeed);
      if(launchDetected && !landingDetected)
      {
        landingDetected = checkIfLanded();
      }
    }

    // Validate RRC3 drogue and main status if earlier data was valid.
    // Reliance on earlier data being valid is because if wrong number of commas (incorrect packet format)
    // or data is otherwise corrupt, drogue and main status data is likely corrupt.
    if (RRC3DataValid)
    { 
      RRC3DrogueBit = RRC3data[0];
      RRC3MainBit = RRC3data[1];
      
      // determine drogue status
      if(RRC3DrogueBit == '?')
      {
        drogueContinuity = false;    
      }
      else if(RRC3DrogueBit == 'd')
      {
        drogueContinuity = true;
        drogueDeployed = false;    
      }
      else if(RRC3DrogueBit == 'D')
      {
        drogueContinuity = false;
        drogueDeployed = true;    
      }
      else
      {
        RRC3DataValid = false;
      }
      
      // determine main parachute status
      if(RRC3MainBit == '?')
      {
        mainContinuity = false;    
      }
      else if(RRC3MainBit == 'm')
      {
        mainContinuity = true;
        mainDeployed = false;    
      }
      else if(RRC3MainBit == 'M')
      {
        mainContinuity = false;
        mainDeployed = true;    
      }
      else
      {
        RRC3DataValid = false;
      }
    }
  
#if defined(DEVMODE)
    if (RRC3DataValid)
    {
      Serial.println("RRC3 data valid");  
    }
    else
    {
      Serial.println("RRC3 data NOT valid");        
    }
    Serial.write("Altitude: ");
    Serial.println(altitude);  
    Serial.write("Speed: ");
    Serial.println(recentSpeed[recentSpeedCount-1]);  
    Serial.write("Drogue: ");
    Serial.println(RRC3DrogueBit);
    Serial.write("Main: ");
    Serial.println(RRC3MainBit);
#endif
  }

  wdt_reset();
}

void pushSpeed(float speed)
{
  for(int i=0; i<recentSpeedCount-1; i++)
  {
    recentSpeed[i] = recentSpeed[i+1];
  }
  recentSpeed[recentSpeedCount-1] = speed; 
}

bool checkIfLanded()
{
  bool landed = true;
  for(int i=0; i<recentSpeedCount-1; i++)
  {
    if(recentSpeed[i] > landingSpeedThreshold)
    {
      landed = false;
    }
  }
  return landed;
}

bool isValidAltitudeOrSpeedDigit(char digit)
{
  return digit == '-' || digit == '0' || digit == '1' || digit == '2' || digit == '3' || digit == '4' || digit == '5' || digit == '6' || digit == '7' || digit == '8' || digit == '9';
}

void updatePosition() {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  int temp = 0;

  double d_lat = gps.location.lat();
  double dm_lat = 0.0;

  if (d_lat < 0.0) {
    temp = -(int)d_lat;
    dm_lat = temp * 100.0 - (d_lat + temp) * 60.0;
  } else {
    temp = (int)d_lat;
    dm_lat = temp * 100 + (d_lat - temp) * 60.0;
  }

  dtostrf(dm_lat, 7, 2, latStr);

  if (dm_lat < 1000) {
    latStr[0] = '0';
  }

  if (d_lat >= 0.0) {
    latStr[7] = 'N';
  } else {
    latStr[7] = 'S';
  }

  APRS_setLat(latStr);

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
  double d_lon = gps.location.lng();
  double dm_lon = 0.0;

  if (d_lon < 0.0) {
    temp = -(int)d_lon;
    dm_lon = temp * 100.0 - (d_lon + temp) * 60.0;
  } else {
    temp = (int)d_lon;
    dm_lon = temp * 100 + (d_lon - temp) * 60.0;
  }

  dtostrf(dm_lon, 8, 2, lonStr);

  if (dm_lon < 10000) {
    lonStr[0] = '0';
  }
  if (dm_lon < 1000) {
    lonStr[1] = '0';
  }

  if (d_lon >= 0.0) {
    lonStr[8] = 'E';
  } else {
    lonStr[8] = 'W';
  }

  APRS_setLon(lonStr);
}

void setDefaultPosition()
{
  // DDMM.MM[N/S]
  char latStr[10];
  latStr[0] = '0';
  latStr[1] = '0';
  latStr[2] = '0';
  latStr[3] = '0';
  latStr[4] = '.';
  latStr[5] = '0';
  latStr[6] = '0';
  latStr[7] = 'N';
  APRS_setLat(latStr);

  // DDDMM.MM[E/W]
  char lonStr[10];
  lonStr[0] = '0';
  lonStr[1] = '0';
  lonStr[2] = '0';
  lonStr[3] = '0';
  lonStr[4] = '0';
  lonStr[5] = '.';
  lonStr[6] = '0';
  lonStr[7] = '0';
  lonStr[8] = 'W';
  APRS_setLon(lonStr);
}

char boolToChar(bool value)
{
  if(value)
  {
    return '1';
  }
  else
  {
    return '0';
  }
}

void updateTelemetry() {
 
  // Altitude [0-4]
  dtostrf(altitude, 5, 0, telemetry_buff);

  // Speed [5-8]
  dtostrf(recentSpeed[recentSpeedCount-1], 4, 0, telemetry_buff + 5);

  // Battery Voltage [9-12]
  float battV = readBatt() * 100;
  dtostrf(battV, 4, 0, telemetry_buff + 9);

  // Temperature [13-16]
  float tempC = bmp.readTemperature() * 100;
  dtostrf(tempC, 4, 0, telemetry_buff + 13);

  // Drogue Deployed [17]
  if(drogueContinuity && !drogueDeployed)
  {
    telemetry_buff[17] = 'd';
  }
  else if(!drogueContinuity && drogueDeployed)
  {
    telemetry_buff[17] = 'D';
  }
  else
  {
    telemetry_buff[17] = '?';    
  }

  // Main Deployed [18]
  if(mainContinuity && !mainDeployed)
  {
    telemetry_buff[18] = 'm';
  }
  else if(!mainContinuity && mainDeployed)
  {
    telemetry_buff[18] = 'M';
  }
  else
  {
    telemetry_buff[18] = '?';
  }

  // GPS Signal Valid [19]
  telemetry_buff[19] = boolToChar(gpsValid);

  // Altimeter UART [20]
  telemetry_buff[20] = boolToChar(altimeterUartConnected);

  // Launch Detected [21]
  telemetry_buff[21] = boolToChar(launchDetected);

  // Landing Detected [22]
  telemetry_buff[22] = boolToChar(landingDetected); 

#if defined(DEVMODE)
  Serial.println(telemetry_buff);
#endif

}

void sendLocation() {

#if defined(DEVMODE)
      Serial.println(F("Location sending with comment"));
#endif

  int PTTDelay = PTTDelayShort;
  if (transmitInterval >= preLaunchXmitInterval)
  {
    PTTDelay = PTTDelayLong;
  }

  AprsPinOutput;
  RfPttON;
  delay(PTTDelay);
  APRS_sendLoc(telemetry_buff, strlen(telemetry_buff)); // beacon without timestamp
  delay(50);
  while(digitalRead(1)){;} // LibAprs TX Led pin PB1
  delay(50);  
  RfPttOFF;
  AprsPinInput;

#if defined(DEVMODE)
  Serial.println(F("Location sent with comment"));
#endif

  wdt_reset();
}

static void updateGpsData(int ms)
{
  while (!Serial1) {
    delayMicroseconds(1); // wait for serial port to connect.
  }
    unsigned long start = millis();
    unsigned long bekle=0;
    do
    {
      while (Serial1.available()>0) {
        gps.encode(Serial1.read());
        bekle= millis();
      }
      if (bekle!=0 && bekle+10<millis())break;
    } while (millis() - start < ms);

  wdt_reset();
}

float readBatt() {
  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0;
  do { 
    value =analogRead(BattPin);
    delay(5);
    value =analogRead(BattPin);
    value=value-8;
    value = (value * 2.56) / 1024.0;
    value = value / (R2/(R1+R2));
  } while (value > 16.0);
  return value ;
}

void freeMem() {
#if defined(DEVMODE)
  Serial.print(F("Free RAM: ")); Serial.print(freeMemory()); Serial.println(F(" byte"));
#endif
}

void gpsDebug() {
#if defined(DEVMODE)
  Serial.println();
  Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card Chars Sentences Checksum"));
  Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  RX    RX        Fail"));
  Serial.println(F("-----------------------------------------------------------------------------------------------------------------"));

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();

#endif
}

static void printFloat(float val, bool valid, int len, int prec)
{
#if defined(DEVMODE)
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
#endif
}

static void printInt(unsigned long val, bool valid, int len)
{
#if defined(DEVMODE)
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
#endif
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
#if defined(DEVMODE)
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
#endif
}

static void printStr(const char *str, int len)
{
#if defined(DEVMODE)
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
#endif
}


