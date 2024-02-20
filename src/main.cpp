#include <Arduino.h>
#include <time.h>
#include <SPIFFS.h>
#include <nvs_flash.h>
#include <Preferences.h>

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_MCP23X17.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <Adafruit_ADS1X15.h>


#include <WiFi.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

#include <ESP32Ping.h>

#include <Firebase_ESP_Client.h>
// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

//Single-point calibration Mode=0
//Two-point calibration Mode=1
#define TWO_POINT_CALIBRATION 1

#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL_VAL_V1 (1075) //mv
#define CAL_VAL_T1 (25)   //℃
//Two-point calibration needs to be filled CAL2_V and CAL2_T
//CAL1 High temperature point, CAL2 Low temperature point
#define CAL_VAL_V2 (787)  //mv
#define CAL_VAL_T2 (15)   //℃


/***********STRUCTS***********/
struct Button {
  const uint8_t pin;
  bool pressed;
  unsigned long lastPressTime;
};

/**********CONSTANTS**********/
const char* DEFAULT_WIFI_SSID = "HUAWEI-2.4G-88uF"; // = "Globe"; //  = "Moggy"; //           //insert default ssid for dev purposes
const char* DEFAULT_WIFI_PASSWORD = "wFJKvAksmLtec5Eb"; // = "dikoalam"; // = "calculus"; //        //insert default password for dev purposes

const char* USER_EMAIL = "smartAquaponicsSystem@gmail.com";
const char* USER_PASSWORD = "smartAquaponics";

const char* API_KEY = "AIzaSyALUCmJy89DL04ZAWQGoUNat5SGwKFuLq4";
const char* DATABASE_URL = "https://esp32-smart-aquaponics-system-default-rtdb.asia-southeast1.firebasedatabase.app/";

const char* NTP_SERVER_1 = "pool.ntp.org";        //ntp server 1
const char* NTP_SERVER_2 = "time.google.com";     //ntp server 2
const char* NTP_SERVER_3 = "time.cloudflare.com"; //ntp server 3

const char* REMOTE_HOST = "www.google.com";       //ping this remote host to confirm if we have internet connection

//for searching for parameter in the HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";

//SPIFFS filepaths
const char* SSID_PATH = "/ssid.txt";
const char* PASS_PATH = "/pass.txt";

//Firebase filepaths
// Database main path (to be updated in setup with the user UID)
String databasePath;
// Database child nodes
// Variables to save database paths

String tempPath = "/temperature";
String phPath = "/ph";
String turbPath = "/turbidity";
String oxyPath = "/oxygen";
String timePath = "/timestamp";

// Parent Node (to be updated in every loop)
String parentPath;
String listenerPath;
String schedulePath;
String scheduleTimePath;
String scheduleStatePath;
//timer constants
const unsigned long ONE_SECOND_MS = 1000;
const unsigned long TWO_SECOND_MS = 2000;
const unsigned long TEN_SECOND_MS = 10000;
const unsigned long FIFTEEN_SECOND_MS = 15000;
const unsigned long THIRTY_SECOND_MS = 30000;
const unsigned long SIXTY_SECOND_MS = 60000;
const unsigned long THREE_MINUTE_MS = 180000;
const unsigned long FIVE_MINUTE_MS = 300000;
const unsigned long SETTLING_TIME_S = 60; //= 3600;
const unsigned long TRANSFER_TIME_S = 60; //= 600;
const unsigned long SCHEDULE_TIME_S = 30; //= 604800;
// Define the restart interval in microseconds (24 hours = 86400000000 microseconds)
const unsigned long long RESTART_INTERVAL = 86400000000UL;

//pin constants
//relay mcp pins, LOW SIGNAL connects COMMON to NORMALLY OPEN terminal, HIGH SIGNAL connects COMMON to NORMALLY CLOSED terminal (unpowered relay also means that the COMMON is always connected to the CLOSED terminal)
const uint8_t WATER_PUMP_1 = 0;   //fish tank water pump, normally closed 
const uint8_t WATER_PUMP_2 = 1;   //sump tank water pump, normally closed
const uint8_t AIR_PUMP_1 = 2;     //fishtank air pump, normally closed
const uint8_t AIR_PUMP_2 = 3;   // mineralization tank air pump, normally closed
const uint8_t AIR_PUMP_3 = 4;   // backup airpump, normally open
const uint8_t M_VALVE_1 = 5;    // backup filter to sump motorized vale, normally open
const uint8_t M_VALVE_2 = 6;    // separator to mineralization motorized valve, normally open, connect open wire to normally open terminal, connect close wire to normally closed terminal
const uint8_t M_VALVE_3 = 7;    // mineralization to growbed motorized valve, normally open, connect open wire to normally open terminal, connect close wire to normally closed terminal
const uint8_t S_VALVE_1 = 8;    // sump to fishtank solenoid valve, normally closed
const uint8_t S_VALVE_2 = 9;    // fishtank to separator solenoid valve, normally closed
const uint8_t S_VALVE_3 = 10;   // fishtank to backup filter solenoid valve, normally open
const uint8_t S_VALVE_4 = 11;   // sump to growbed solenoid valve, normally open
const uint8_t S_VALVE_5= 12;    // backup filter to growbed solenoid valve, normally open 
const uint8_t P_PUMP_1 = 13;   //peristaltic pump up, normally open
const uint8_t P_PUMP_2  = 14;    //peristaltic pump down, normally open
const uint8_t RELAY16 = 15; 


//sensor pins
const uint8_t ONE_WIRE_BUS = 32;    //temperature sensor
const uint8_t PH_PIN = 0;          //ph sensor pin, on the ADS1115
const uint8_t TURBIDITY_PIN = 1;   //turbidity sensor pin
const uint8_t OXYGEN_PIN = 2;      //dissolved oxygen sensor pin

//sensor values
const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

/******PREVIOUS TIME VARIABLES******/
unsigned long previousCleanupMillis = 0;//last time clients were cleaned up
unsigned long previousModeMillis = 0;

unsigned long previousWiFiMillis = 0;
unsigned long previousCheckMillis = 0;  //stores when wifi connection was last checked

unsigned long previousUIDMillis = 0; // for the timeout code of getting the UID

unsigned long previousReadingTime = 0;  //store when readings was last read
unsigned long previousSendingTime = 0;  //store when sensor reading was last sent to thingspeak

unsigned long previousCycleTime = 0;    //when lcd display was last changed

unsigned long settlingStartTime = 0;    //stores epoch timestamp of when settling process was started
unsigned long transferStartTime = 0;    //stores epoch timestamp of when transfer process was started
unsigned long lastModeSwitch = 0;
unsigned long previousTokenCheck = 0;
unsigned long previousTransferCheck = 0; 
unsigned long valveClosingStartTime = 0;
unsigned long previousScheduleCheckTime = 0;
unsigned long scheduleStartTime = 0;
unsigned long scheduledTime = 0;
unsigned long previousButtonPressTime = 0;

unsigned long previousOxygenCheck = 0;

const int32_t timeZoneAdj = -28800; // GMT +08 OFFSET

/*LIVE VARIABLES*/
int operationMode = 0;
int lcdDisplayCycle = 0;
float temperatureLevel = 0;
float phLevel = 0;
float turbidityLevel = 0;
float oxygenLevel = 0;
float tempSum = 0, phSum = 0, turbSum = 0, oxySum = 0;
int readCount = 0;
int buttonPressed = 0;
// Variable to save USER UID
String uid;
int timestamp;
FirebaseJson json;
// Variable to store wifi creds from spiffs
String ssid;
String pass;

/*STORED VARIABLES*/
int lastOperationMode = 0; //store the value from preferences here
int displayCount = 0;
const int displayTotal = 10; //1: Month date and time, 2: Operation mode, 3: Temp and PH value, 4: turbidity and oxygen, 5: wifi connection with IP, 6: Scheduler state, 7: Sensor Connection, 8: Transfer State, 9: Operation mode and time



//DateTime formats
const char* timeFormat1 = "%b %d, %Y - %I:%M:%S %p"; // Month Day, Year - Hour, Minute, Seconds AM/PM
const char* timeFormat2 = "%b %d, %Y";               // Month Day, Year 
const char* timeFormat3 = "%I:%M:%S %p";             // Hour, Minute, Seconds AM/PM
const char* timeFormat4 = "%b %d, %Y";               // Month Day, Year
const char* timeFormat5 = "%m/%d-%H:%M:%S";          //  mm/dd - hh:mm:ss

/*FLAGS*/
bool settlingStarted = false;
bool transferStarted = false;
bool modeSwitched = false;
bool schedulerState = false;
bool ntpServerConnected = false;
bool preferencesStarted = false;
bool spiffsMounted = false;
bool timeUpdated = false;
bool wifiConnected = false;
bool webSocketSuccess = false;
bool mcpConnected = false;
bool rtcConnected = false;
bool lcdConnected = false;
bool adsConnected = false;
bool sensorsDisconnected = false;
bool dataReady = false;
bool firebaseConnected = false;
bool localSwitch = false;
bool valveClosingStarted = false;
bool scheduleStarted = false;
bool updateFromDB = false;
bool scheduleTriggered = false;
bool backupAirPumpState = false;
bool internetConnected = false;

/***********OBJECTS***********/
//button objects
Button aquaponicsButton = {26, false, 0};
Button aquacultureButton = {27, false, 0};
Button hydroponicsButton = {14, false, 0};
Button transferButton = {13, false, 0};
Button relayButton = {4, false, 0};
Button sensorButton = {16, false, 0};
Button wifiButton = {17, false, 0};
Button scheduleButton = {18, false, 0};

Preferences preferences;            //instantiate preference object

Adafruit_MCP23X17 mcp;              //instantiate mcp object
LiquidCrystal_I2C lcd(0x27, 16, 2); //instantiate lcd object
RTC_DS3231 rtc;                     //instantiate rtc object
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

OneWire oneWire(ONE_WIRE_BUS);      //instantiate onewire object
DallasTemperature sensors(&oneWire);//pass the oneWire object to DallasTemp object


hw_timer_t *My_timer = NULL;

// Define Firebase objects
FirebaseData fbdo;
FirebaseData stream;
FirebaseAuth auth;
FirebaseConfig config;

//webserver objects
AsyncWebServer server(80); //Create an AsyncWebServer object on port 80.
// AsyncWebSocket ws("/ws");  //Create an AsyncWebSocket object called ws to handle the connections on the /ws path.
// AsyncEventSource events("/events"); // Create an Event Source on /events

//debugging 
unsigned long previousStateCheckTime = 0;
String boolState = "";
int buttonCount = 0;
int buttonTotal = 8;


/*FUNCTION DECLARATIONS*/
//functions in setup()

void scanI2C(); //scans for I2C Devices and prints their addresses in serial, mostly for debugging
void beginI2C(); //initializes all the I2C devices
void setPinMode(); // sets the pinmode for all the devices, must first check if mcp was initialized for the relay pins
void attachISR(); //attaches interrupts to the buttons

bool initSPIFFS(); //attempts to mount SPIFFS, returns false if it fails
void loadSPIFFsFiles(); //reads the values from the given filepaths and passes them to the appropriate variables
void startSoftAP();
bool initWiFi(); //attempts to connect to wifi using the wifi credentials from SPIFFS storage, if it dit not manage to connect after ten seconds it would return false
bool updateSystemTime(); //attempts to update the esp32 internal rtc time from ntp time(must check if there is an internet connection), and also the external RTC time, returns false if it failed.
bool internetCheck();
bool initFirebase();
bool initializePreferences(); //attempts to initialize preferences, returns false if namespace was not opened
void checkLastMode(); //must check if preferences namespace was opened, fetches the operation mode and returns the value into lastOperationMode

void streamTimeoutCallback(bool timeout);
// Callback function that runs on database changes
void streamCallback(FirebaseStream data);

//functions in loop()

  bool checkWiFi(); //periodically checks if WiFi is still connected, returns false
  void readSensorData();
  void sendSensorData();
  void reconfigureWifi(bool wifiButtonPressed, int buttonPressed); // deletes the stored wifi credentials and forces the esp32 to restart
  void checkInput();
  void selectOperationMode();

//functions that will be reused by other stuff

void aquaponicsMode();
void aquacultureMode();
void hydroponicsMode();
void transferMode();
void turnEverythingOff();
void sensorDisconnect(bool buttonPress, int buttonPressed);
void startScheduler(bool buttonPress, int buttonPressed);
void checkSchedule();

void displayLCD(); //displays various details every 2 seconds
void backupPump(); //checks every minute if dissolved oxygen is below 5mg/L

unsigned long getTime();

float temp(); //returns the temperature from the sensor
float ph(); //outputs the converted ph value from voltage
float turbidity(); //outputs the converted turnodity value from voltage
int16_t readDO(); //outputs the converted dissolved oxygen value micrograms/L from millivolts

String readFile(fs::FS &fs, const char * path); //reads the file from a given filepath and returns the value within.
void writeFile(fs::FS &fs, const char * path, const char * message); //writes the given value into the given file path

String UnixToString(time_t unixTime, const char* format);

String processor(const String& var); //responsible for searching for placeholders on the HTML text and replace them with whatever we want before sending the web page to the browser

String dbSwitch(int gpio, int state); //decides with button name would be given for debug purposes

String dbButtons(int buttonPressed); //decides with path the button value would go

int dbPressed(int gpio); //decides with button was pressed based on the gpio, ran only during setup

void toggleSwitch (int buttonPressed); //decides how the button value would be handled, some buttons would always insist on only giving 1 while others would toggle between 0 and 1

int readIntDB(String buttonPath); //reads the int value from the path, here it is mostly used for the button values

void whichButtonPressed(); // just displays what was previously pressed before checkInput;

//INTERRUPTS
void IRAM_ATTR onAquaponicPress() {
	unsigned long button_time = millis();
  if (button_time - previousButtonPressTime /*aquaponicsButton.lastPressTime*/>= 250){
    aquaponicsButton.pressed = true;
    previousButtonPressTime = button_time;
    //aquaponicsButton.lastPressTime = button_time;
    operationMode = 1;
    buttonPressed = 1;
    localSwitch = true;
    settlingStarted = false;
    transferStarted = false;
  }
}

void IRAM_ATTR onAquaculturePress() {
	unsigned long button_time = millis();
  if (button_time - previousButtonPressTime /*aquacultureButton.lastPressTime*/ > 250){
    aquacultureButton.pressed = true;
    previousButtonPressTime = button_time;
    //aquacultureButton.lastPressTime = button_time;
    operationMode = 2;
    buttonPressed = 2;
    localSwitch = true;
    settlingStarted = false;
    transferStarted = false;
  }
}

void IRAM_ATTR onHydroponicsPress() {
	unsigned long button_time = millis();
  if (button_time - previousButtonPressTime /*hydroponicsButton.lastPressTime*/ > 250){
    hydroponicsButton.pressed = true;
    previousButtonPressTime = button_time;
    //hydroponicsButton.lastPressTime = button_time;
    operationMode = 3;
    buttonPressed = 3;
    localSwitch = true;
    settlingStarted = false;
    transferStarted = false;
  }
}

void IRAM_ATTR onSludgePress() {
	unsigned long button_time = millis();
  if (button_time - previousButtonPressTime /*transferButton.lastPressTime*/ > 250){
    transferButton.pressed = true;
    previousButtonPressTime = button_time;
    //transferButton.lastPressTime = button_time;
    operationMode = 4;
    buttonPressed = 4;
    localSwitch = true;
    settlingStarted = false;
    transferStarted = false;
  }
}

void IRAM_ATTR onRelayPress() {
	unsigned long button_time = millis();
  if (button_time - previousButtonPressTime /*relayButton.lastPressTime*/ > 250){
    relayButton.pressed = true;
    previousButtonPressTime = button_time;
    //relayButton.lastPressTime = button_time;
    operationMode = 5;
    buttonPressed = 5;
    localSwitch = true;
    settlingStarted = false;
    transferStarted = false;
  }
}

void IRAM_ATTR onSensorPress() {
	unsigned long button_time = millis();
  if (button_time - previousButtonPressTime /*sensorButton.lastPressTime*/ > 250){
    sensorButton.pressed = true;
    previousButtonPressTime = button_time;
    //sensorButton.lastPressTime = button_time;
    buttonPressed = 6;
    localSwitch = true;
  }
}

void IRAM_ATTR onWiFiPress() {
	unsigned long button_time = millis();
  if (button_time - previousButtonPressTime /*wifiButton.lastPressTime*/ > 250){
    wifiButton.pressed = true;
    previousButtonPressTime = button_time;
    //wifiButton.lastPressTime = button_time;
    buttonPressed = 7;
    localSwitch = true;
    
  }
}

void IRAM_ATTR onSchedulePress() {
	unsigned long button_time = millis();
  if (button_time - previousButtonPressTime /*scheduleButton.lastPressTime*/ > 250){
    scheduleButton.pressed = true;
    previousButtonPressTime = button_time;
    //scheduleButton.lastPressTime = button_time;
    buttonPressed = 8;
    localSwitch = true;
    // settlingStarted = false;
    // transferStarted = false;
    
  }
}

void IRAM_ATTR restartESP(){
  Serial.println("DAILY RESTART: Starting cleanup tasks...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DAILY RESTART");
  lcd.setCursor(0, 1);
  lcd.print("Starting cleanup");

  delay(1000);

  Serial.println("DAILY RESTART: Storing the current operation mode...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DAILY RESTART");
  lcd.setCursor(0, 1);
  lcd.print("Saving Last Mode");
  // save operation state to preferences
  preferences.putInt("lastOperation", operationMode);

  delay(1000);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DAILY RESTART");
  lcd.setCursor(0, 1);
  lcd.print("Restarting ESP32");

  delay(1000);

  ESP.restart();
}

void setup() {
  //initialize serial
  Serial.begin(115200);
  delay(100);
  Serial.println("BOOT: Initializing Serial");

  Wire.begin();
  scanI2C();
  beginI2C();
  
  
  //set adc gain
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  setPinMode();
  Serial.println("Mode Switching: turning off all relays...");
    //220V
    mcp.digitalWrite(WATER_PUMP_1, LOW);
    mcp.digitalWrite(WATER_PUMP_2, LOW);

    mcp.digitalWrite(AIR_PUMP_1, LOW);
    mcp.digitalWrite(AIR_PUMP_2, LOW);
    mcp.digitalWrite(AIR_PUMP_3, HIGH);

    mcp.digitalWrite(M_VALVE_1, HIGH);
    mcp.digitalWrite(M_VALVE_2, HIGH);
    mcp.digitalWrite(M_VALVE_3, HIGH);

    //12V
    mcp.digitalWrite(S_VALVE_1, HIGH);
    mcp.digitalWrite(S_VALVE_2, HIGH);
    mcp.digitalWrite(S_VALVE_3, LOW);
    mcp.digitalWrite(S_VALVE_4, LOW);
    mcp.digitalWrite(S_VALVE_5, HIGH);
    mcp.digitalWrite(P_PUMP_1, HIGH);
    mcp.digitalWrite(P_PUMP_2, HIGH);
    mcp.digitalWrite(RELAY16, HIGH);

  
  attachISR();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SMART AQUAPONICS");
  lcd.setCursor(0, 1);
  lcd.print("BOOTING SYSTEM..");
  
  preferencesStarted = initializePreferences();
  
  spiffsMounted = initSPIFFS();
  loadSPIFFsFiles();

  wifiConnected = initWiFi();
  checkLastMode();
  internetConnected = internetCheck();
  ntpServerConnected = updateSystemTime();
  firebaseConnected = initFirebase();

  startSoftAP();
  //initialize the timer that will restart the esp32 every 24 hours
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &restartESP, true);
  timerAlarmWrite(My_timer, RESTART_INTERVAL, false);
  timerAlarmEnable(My_timer);

}

void loop() {
  // put your main code here, to run repeatedly:

  //refresh the firebase token...
  if (millis() - previousTokenCheck >= 5000) {
    if (firebaseConnected) {
      if (Firebase.isTokenExpired()){
      Firebase.refreshToken(&config);
      Serial.println("Refresh token");
      }
    } else {
      Serial.println("Firebase not Connected");
    }
    previousTokenCheck = millis();
  }

  readSensorData();
  sendSensorData();

  
  whichButtonPressed();
  checkInput();
  checkSchedule();
  selectOperationMode();
  
  displayLCD();
  backupPump();

}



void startSoftAP() {
  if (wifiConnected == false || WiFi.status() != WL_CONNECTED) {
  /***************************************
  START WIFI MANAGER AP TO GET CREDENTIALS
  ****************************************/
  // Connect to Wi-Fi network with SSID and password
  Serial.println("BOOT - WIFI AP: Setting AP (Access Point)");
  // NULL sets an open Access Point
  WiFi.softAP("SMART-AQUAPONICS-AP", NULL);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("BOOT - WIFI AP: AP IP address: ");
  Serial.println(IP); 

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/wifimanager.html", "text/html");
  });
  
  server.serveStatic("/", SPIFFS, "/");
  
  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
    int params = request->params();
    for(int i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isPost()){
        // HTTP POST ssid value
        if (p->name() == PARAM_INPUT_1) {
          ssid = p->value().c_str();
          Serial.print("SSID set to: ");
          Serial.println(ssid);
          // Write file to save value
          writeFile(SPIFFS, SSID_PATH, ssid.c_str());
        }
        // HTTP POST pass value
        if (p->name() == PARAM_INPUT_2) {
          pass = p->value().c_str();
          Serial.print("Password set to: ");
          Serial.println(pass);
          // Write file to save value
          writeFile(SPIFFS, PASS_PATH, pass.c_str());
        }
        //Serial.printf("\nPOST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }
  request->send(200, "text/plain", "Done. ESP will restart, please head to the webapp to access the dashboard.");
  delay(3000);
  ESP.restart();
  });
  }
  server.begin();
}

// put function definitions here:

void scanI2C() {
  byte error, address;
  int nDevices;
  Serial.println("BOOT - I2C SCAN: Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("BOOT - I2C SCAN: I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("BOOT - I2C SCAN: Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("BOOT - I2C SCAN: No I2C devices found\n");
  }
  else {
    Serial.println("BOOT - I2C SCAN: done\n");
  }
}

bool initWiFi() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WIFI CONNECTION");
  lcd.setCursor(0, 1);
  lcd.print("INITIALIZING...");
  delay(500);
  // Serial.println("BOOT - WIFI INIT: Attempting to connect to WiFi.");
  // if(ssid==""){
  // Serial.println("BOOT - WIFI INIT: Undefined SSID or IP address.");
  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("WIFI CONNECTION");
  // lcd.setCursor(0, 1);
  // lcd.print("NO CREDENTIALS..");
  // delay(500);
  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("WIFI CONNECTION");
  // lcd.setCursor(0, 1);
  // lcd.print("CANNOT CONNECT..");
  // return false;
  // }

  WiFi.mode(WIFI_STA);
  //WiFi.begin(ssid.c_str(), pass.c_str());
  WiFi.begin(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PASSWORD);
  Serial.println("BOOT - WIFI INIT: Connecting to WiFi...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WIFI CONNECTION");
  lcd.setCursor(0, 1);
  lcd.print("CONNECTING..");

  unsigned long currentMillis = millis();
  previousWiFiMillis = currentMillis;

  while(WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - previousWiFiMillis >= TEN_SECOND_MS) {
      Serial.println("BOOT - WIFI INIT: Failed to connect.");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WIFI CONNECTION");
      lcd.setCursor(0, 1);
      lcd.print("FAILED...");

      return false;
    }
  }

  Serial.println(WiFi.localIP());

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WIFI CONNECTED..");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP().toString());

  return true;
}

bool updateSystemTime() {
  bool success;
  unsigned long time;
  unsigned long adjustedTime;
  Serial.println("BOOT - SYSTEM TIME UPDATE: Attempting to connect to ntp servers.");
  if (WiFi.status() == WL_CONNECTED && wifiConnected == true && internetConnected == true) {
    configTime(28800, 0, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
    delay(2000);
    Serial.print("Time from ntp: ");
    time = getTime();
    adjustedTime = time + 28800;
    Serial.println(UnixToString(time, timeFormat1));
    Serial.println(UnixToString(adjustedTime, timeFormat1));

    Serial.println("Updating time stored in the RTC...");
    rtc.adjust(DateTime(getTime()));

    Serial.print("Time from rtc: ");
    time = rtc.now().unixtime();
    adjustedTime = time + 28800;
    Serial.println(UnixToString(time, timeFormat1));
    Serial.println(UnixToString(adjustedTime, timeFormat1));
    return true;

  } else if (WiFi.status() != WL_CONNECTED || wifiConnected == false || internetConnected == false) {
    Serial.println("BOOT - SYSTEM TIME UPDATE: No internet connection, cannot update epoch time through ntp server.");
    return false;
  }
}

// Function that gets current epoch time
unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

String UnixToString(time_t unixTime, const char* format) {
  struct tm* timeinfo;
  char buffer[80];
  timeinfo = localtime(&unixTime);
  strftime(buffer, sizeof(buffer), format, timeinfo);
  //Serial.print(unixTime);
  //Serial.print(" converted to readable string: ");
  //Serial.println(buffer);
  return buffer;
}

float temp() {
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  return temperatureC;
}

bool internetCheck() {
  bool success = false;
  if (WiFi.status() != WL_CONNECTED && wifiConnected != true) {

    Serial.println("CONNECTION: Not connected to WiFi ");

  } else {

    Serial.println("CONNECTION: Pinging \"www.google.com\" ");
    success = Ping.ping(REMOTE_HOST, 3); //ping esp32 using google
    if(!success) {
      Serial.println("CONNECTION: Ping to \"www.google.com\" failed...");
    } else {
      Serial.println("CONNECTION: Ping to \"www.google.com\" succeeded...");
    }

  }

  
  return success;
}

bool initFirebase() {
  if (WiFi.status() == WL_CONNECTED && wifiConnected == true && internetConnected == true)  {
    bool success = false;
    int count = 0;
    // Assign the api key (required)
    config.api_key = API_KEY;

    // Assign the user sign in credentials
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    // Assign the RTDB URL (required)
    config.database_url = DATABASE_URL;

    Firebase.reconnectWiFi(true);
    fbdo.setResponseSize(4096);

    if (!internetCheck()) {
      Serial.println("BOOT: Firebase: Lost connection to internet...");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("FIREBASE INIT..");
      lcd.setCursor(0, 1);
      lcd.print("NO INTERNET...");

      return false;
    }
    delay(500);

    // Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

    // Assign the maximum retry of token generation
    config.max_token_generation_retry = 5;

    // Initialize the library with the Firebase authen and config
    Firebase.begin(&config, &auth);
    // Getting the user UID might take a few seconds
    //TODO: Add a bit for the LCD to display this

    Serial.println("Getting User UID");

    previousUIDMillis = millis();

    while ((auth.token.uid) == "") {

      Serial.print('.');

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("FIREBASE INIT..");
      lcd.setCursor(0, 1);
      lcd.print("Getting UID...");

      //if there is still no UID after 15 seconds, that means there was something wrong with the token generation, and it will just set the flag that the firebase connection has failed.
      if (millis() - previousUIDMillis >= FIFTEEN_SECOND_MS) {
  
        Serial.println("BOOT: Firebase: Timeout - Cannot fetch the UID...");
        Serial.println("BOOT: Firebase: Timeout - Firebase connection failed...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("FIREBASE INIT..");
        lcd.setCursor(0, 1);
        lcd.print("UID FETCH FAIL");

        delay(1000);
        return false;
      }

      delay(1000);
    }

    // Print user UID
    uid = auth.token.uid.c_str();
    Serial.print("User UID: ");
    Serial.println(uid);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FIREBASE INIT..");
    lcd.setCursor(0, 1);
    lcd.print("UID FETCHED...");

    // Update database path
    databasePath = "/UsersData/" + uid + "/readings";
    listenerPath = "/UsersData/" + uid + "/buttons";
    schedulePath = "/UsersData/" + uid + "/schedule";
    scheduleTimePath = schedulePath + "/timestamp";
    scheduleStatePath = schedulePath + "/state";
    if (!Firebase.RTDB.beginStream(&stream, listenerPath.c_str()))
      Serial.printf("\nstream begin error, %s\n\n", stream.errorReason().c_str());
    // Assign a calback function to run when it detects changes on the database
    Firebase.RTDB.setStreamCallback(&stream, streamCallback, streamTimeoutCallback);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FIREBASE INIT..");
    lcd.setCursor(1, 0);
    lcd.print("CONNECTED...");

    firebaseConnected = true;
    
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FIREBASE INIT..");
    lcd.setCursor(0, 1);
    lcd.print("FAILED: NO WIFI");
    firebaseConnected = false;
  } 

  return firebaseConnected;
}

void streamTimeoutCallback(bool timeout){
  if (timeout)
    Serial.println("stream timeout, resuming...\n");
  if (!stream.httpConnected())
    Serial.printf("\nerror code: %d, reason: %s\n\n", stream.httpCode(), stream.errorReason().c_str());
}

// Callback function that runs on database changes
void streamCallback(FirebaseStream data){
  Serial.printf("\nstream path, %s\nevent path, %s\ndata type, %s\nevent type, %s\n\n",
                data.streamPath().c_str(),
                data.dataPath().c_str(),
                data.dataType().c_str(),
                data.eventType().c_str());
  printResult(data); //see addons/RTDBHelper.h
  Serial.println();

  // Get the path that triggered the function
  String streamPath = String(data.dataPath());

  // if the data returned is an integer, there was a change on the GPIO state on the following path /{gpio_number}
  if (localSwitch == true) {
    Serial.println("Button states updated from esp32...");
    Serial.println("Skipping till next loop...");
    localSwitch = false;
  } else if (localSwitch == false){
    if (scheduleTriggered != true) {
      Serial.println("Button states updated from database...");
      if (data.dataTypeEnum() == fb_esp_rtdb_data_type_integer){
        Serial.println("FIRST BIT");
        String buttonName;
        String gpio = streamPath.substring(1);
        int state = data.intData();
        buttonName = dbSwitch(atoi(gpio.c_str()), state);
        Serial.print("BUTTON GPIO: ");
        Serial.print(gpio);
        Serial.print(": ");
        Serial.println(buttonName);
        Serial.print("BUTTON STATE: ");
        Serial.println(state);
      }
    } else if (scheduleTriggered == true){
      Serial.println("Recent button state update was because of scheduler...");
      Serial.println("Skipping till next loop...");
      scheduleTriggered = false;
    }
  }

  /* When it first runs, it is triggered on the root (/) path and returns a JSON with all keys
  and values of that path. So, we can get all values from the database and updated the GPIO states*/
  if (data.dataTypeEnum() == fb_esp_rtdb_data_type_json){
    FirebaseJson json = data.to<FirebaseJson>();
    Serial.println("SECOND BIT");
    // To iterate all values in Json object
    size_t count = json.iteratorBegin();
    Serial.println("\n---------");
    for (size_t i = 0; i < count; i++){
        FirebaseJson::IteratorValue value = json.valueAt(i);
        String buttonName;
        int gpio = value.key.toInt();
        int state = value.value.toInt();
        //this selects the operation mode and which button was pressed depending on the state read from the database
        if (state == 1) {
          switch (gpio) {
            case 26:
              operationMode = 1;
              aquaponicsButton.pressed = true;
              break;
            case 27:
              operationMode = 2;
              aquacultureButton.pressed = true;
              break;
            case 14:
              operationMode = 3;
              hydroponicsButton.pressed = true;
              break;
            case 13:
              operationMode = 4;
              transferButton.pressed = true;
              break;
            case 4:
              operationMode = 5;
              relayButton.pressed = true;
              break;
            default:
              break;
          }
          localSwitch = true; // makes it so that the state change from here would not loop
        }
        Serial.print("BUTTON GPIO: ");
        Serial.println(gpio);
        Serial.print("BUTTON STATE: ");
        Serial.println(state);
        Serial.printf("Name: %s, Value: %s, Type: %s\n", value.key.c_str(), value.value.c_str(), value.type == FirebaseJson::JSON_OBJECT ? "object" : "array");
    }
    Serial.println();
    json.iteratorEnd(); // required for free the used memory in iteration (node data collection)
    Serial.println("BOOT: Firebase: Retrieving scheduler state from database...");
    
    if (Firebase.RTDB.getBool(&fbdo, scheduleStatePath.c_str())) {
      if (fbdo.dataType() == "bool") {
        scheduleStarted = fbdo.boolData();
      }
    }

    Serial.print("BOOT: Firebase: Stored scheduleState:");
    Serial.println(scheduleStarted);
    Serial.println("BOOT: Firebase: Retrieving schedule start time from database...");

    if (Firebase.RTDB.get(&fbdo, scheduleTimePath.c_str())) {
      if (fbdo.dataType() == "int") {
        String timestampStr = fbdo.stringData();
        int64_t timestamp = atoll(timestampStr.c_str());
        // Use the timestamp value as needed
        scheduleStartTime = timestamp;
      }
    }

    Serial.print("BOOT: Firebase: Stored scheduleTime:");
    Serial.println(scheduleStartTime);
    
    Serial.print("Saved operation mode from database is: ");
    
    Serial.println(operationMode);
  }
  
  //This is the size of stream payload received (current and max value)
  //Max payload size is the payload size under the stream path since the stream connected
  //and read once and will not update until stream reconnection takes place.
  //This max value will be zero as no payload received in case of ESP8266 which
  //BearSSL reserved Rx buffer size is less than the actual stream payload.
  Serial.printf("\nReceived stream payload size: %d (Max. %d)\n\n", data.payloadLength(), data.maxPayloadLength());
}

//outputs the corresponding number for each button
int dbPressed(int gpio){
  int buttonPressed;
  switch(gpio){
    case 26: 
      buttonPressed = 1;
      break;
    case 27: 
      buttonPressed = 2;
      break;
    case 14: 
      buttonPressed = 3;
      break;
    case 13: 
      buttonPressed = 4;
      break;
    case 4: 
      buttonPressed = 5;
      break;
    case 16: 
      buttonPressed = 6;
      break;
    case 17: 
      buttonPressed = 7;
      break;
    case 18: 
      buttonPressed = 8;
      break;
    default:
      break;
  }
  return buttonPressed;
}

//outputs the corresponding name string for each button while also emulating a physical button press
String dbSwitch(int gpio, int state){
  String buttonName;
  switch(gpio){
    case 26: 
      if (millis() - previousButtonPressTime >= 250) {
        if(state != 0) {
          aquaponicsButton.pressed = true;
          operationMode = 1;
          buttonPressed = 1;
          previousButtonPressTime = millis();
          settlingStarted = false;
          transferStarted = false;
        }
      }
      buttonName = "Aquaponics Button";
      break;
    case 27: 
      if (millis() - previousButtonPressTime >= 250) {
        if (state != 0){
          aquacultureButton.pressed = true;
          operationMode = 2;
          buttonPressed = 2;
          previousButtonPressTime = millis();
          settlingStarted = false;
          transferStarted = false;
        }
      }
      buttonName = "Aquaculture Button";
      break;
    case 14: 
      if (millis() - previousButtonPressTime >= 250) {
        if (state != 0){
          hydroponicsButton.pressed = true;
          operationMode = 3;
          buttonPressed = 3;
          hydroponicsButton.lastPressTime = millis();
          settlingStarted = false;
          transferStarted = false;
        }
      }
      buttonName = "Hydroponics Button";
      break;
    case 13: 
      if (millis() - previousButtonPressTime >= 250) {
        if (state != 0){
          transferButton.pressed = true;
          operationMode = 4;
          buttonPressed = 4;
          previousButtonPressTime = millis();
          settlingStarted = false;
          transferStarted = false;
        }
      }
      buttonName = "Transfer Button";
      break;
    case 4: 
      if (millis() - previousButtonPressTime >= 250) {
        if (state != 0){
          relayButton.pressed = true;
          operationMode = 5;
          buttonPressed = 5;
          previousButtonPressTime = millis();
          settlingStarted = false;
          transferStarted = false;
        }
      }
      buttonName = "Relay Button";
      break;
    case 16: 
      if (millis() - previousButtonPressTime >= 250) {
        sensorButton.pressed = true;
        buttonPressed = 6;
        previousButtonPressTime = millis();
      }
      buttonName = "Sensor Button";
      break;
    case 17: 
      if (millis() - previousButtonPressTime >= 250) {
        wifiButton.pressed = true;
        buttonPressed = 7;
        previousButtonPressTime = millis();
      }
      buttonName = "WiFi Button";
      break;
    case 18: 
      if (millis() - previousButtonPressTime >= 250) {
        scheduleButton.pressed = true;
        buttonPressed = 8;
        previousButtonPressTime = millis();
      }
      buttonName = "Schedule Button";
      break;
    default:
      break;
  }
  localSwitch = false;
  return buttonName;
}

//outputs the corresponding database child path for the button
String dbButtons(int buttonPressed) {
  String buttonPath;
  switch(buttonPressed) {
    case 1: 
      buttonPath = "/26";
      break;
    case 2: 
      buttonPath = "/27";
      break;
    case 3: 
      buttonPath = "/14";
      break;
    case 4: 
      buttonPath = "/13";
      break;
    case 5: 
      buttonPath = "/4";
      break;
    case 6: 
      buttonPath = "/16";
      break;
    case 7: 
      buttonPath = "/17";
      break;
    case 8: 
      buttonPath = "/18";
      break;
    default:
      break;
    }
  return buttonPath;
}

void setPinMode(){
  //relays
  if (mcpConnected == true) {
    //220V AC
    mcp.pinMode(WATER_PUMP_1, OUTPUT);
    mcp.pinMode(WATER_PUMP_2, OUTPUT);
    mcp.pinMode(AIR_PUMP_1, OUTPUT);
    mcp.pinMode(AIR_PUMP_2, OUTPUT);
    mcp.pinMode(AIR_PUMP_3, OUTPUT);
    mcp.pinMode(M_VALVE_1, OUTPUT);
    mcp.pinMode(M_VALVE_2, OUTPUT);
    mcp.pinMode(M_VALVE_3, OUTPUT);

    //12V DC
    mcp.pinMode(S_VALVE_1, OUTPUT);
    mcp.pinMode(S_VALVE_2, OUTPUT);
    mcp.pinMode(S_VALVE_3, OUTPUT);
    mcp.pinMode(S_VALVE_4, OUTPUT);
    mcp.pinMode(P_PUMP_1, OUTPUT);
    mcp.pinMode(P_PUMP_2, OUTPUT);
    mcp.pinMode(S_VALVE_5, OUTPUT);
    mcp.pinMode(RELAY16, OUTPUT);

  }


  //buttons
  pinMode(aquaponicsButton.pin, INPUT_PULLUP);
  pinMode(aquacultureButton.pin, INPUT_PULLUP);
  pinMode(hydroponicsButton.pin, INPUT_PULLUP);
  pinMode(transferButton.pin, INPUT_PULLUP);
  pinMode(relayButton.pin, INPUT_PULLUP);
  pinMode(sensorButton.pin, INPUT_PULLUP);
  pinMode(wifiButton.pin, INPUT_PULLUP);
  pinMode(scheduleButton.pin, INPUT_PULLUP);
  
}
void attachISR(){
  attachInterrupt(aquaponicsButton.pin, onAquaponicPress, FALLING);
  attachInterrupt(transferButton.pin, onSludgePress, FALLING);
  attachInterrupt(aquacultureButton.pin, onAquaculturePress, FALLING);
  attachInterrupt(hydroponicsButton.pin, onHydroponicsPress, FALLING);
  attachInterrupt(relayButton.pin, onRelayPress, FALLING);
  attachInterrupt(sensorButton.pin, onSensorPress, FALLING);
  attachInterrupt(wifiButton.pin, onWiFiPress, FALLING);
  attachInterrupt(scheduleButton.pin, onSchedulePress, FALLING);
}

void checkInput() {
 if(((aquaponicsButton.pressed == true) || (transferButton.pressed == true) ||
     (aquacultureButton.pressed == true) || (hydroponicsButton.pressed == true) || 
     (relayButton.pressed == true)  || (sensorButton.pressed == true) || (wifiButton.pressed == true) || (scheduleButton.pressed == true)) && modeSwitched == false) {
      toggleSwitch(buttonPressed);
      if (sensorButton.pressed == true || wifiButton.pressed == true || scheduleButton.pressed == true) {
        sensorDisconnect(sensorButton.pressed, buttonPressed);
        reconfigureWifi(wifiButton.pressed, buttonPressed);
        startScheduler(scheduleButton.pressed, buttonPressed);
        modeSwitched = false;
      } else {
        modeSwitched = true;
      }
      aquaponicsButton.pressed = false;
      transferButton.pressed = false;
      aquacultureButton.pressed = false;
      hydroponicsButton.pressed = false;
      relayButton.pressed = false;
      wifiButton.pressed = false;
      sensorButton.pressed = false;
      scheduleButton.pressed = false;
     }
}



void toggleSwitch (int buttonPressed) {
  FirebaseJson json;
  String buttonPath = listenerPath.c_str() + dbButtons(buttonPressed);
  if (localSwitch == true) {
    Serial.println("Attempting to update buttonstate from esp32...");
    Serial.print("Updating button state for");
    Serial.println(buttonPressed);
    if(Firebase.ready()) {
      switch (buttonPressed) {
        case 1:
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 1) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(2)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(3)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(4)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(5)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());

          break;
        case 2:
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 1) ? "ok" : fbdo.errorReason().c_str());

          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(1)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(3)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(4)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(5)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          break;  
        case 3:
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 1) ? "ok" : fbdo.errorReason().c_str());

          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(1)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(2)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(4)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(5)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          break;
        case 4:
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 1) ? "ok" : fbdo.errorReason().c_str());

          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(1)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(2)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(3)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(5)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          break;
        case 5:
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 1) ? "ok" : fbdo.errorReason().c_str());

          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(2)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(3)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(4)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, (listenerPath.c_str() + dbButtons(1)).c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          break;
        case 6:
          if (readIntDB(buttonPath) != 1) {
            Serial.print("Current state of Button - ");
            Serial.print(buttonPressed);
            Serial.print(": ");
            Serial.println(readIntDB(buttonPath));

            Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 1) ? "ok" : fbdo.errorReason().c_str());
          } else {
            Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          }
          break; 
        case 7:
          if (readIntDB(buttonPath) != 1) {
            Serial.print("Current state of Button - ");
            Serial.print(buttonPressed);
            Serial.print(": ");
            Serial.println(readIntDB(buttonPath));
            
            Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 1) ? "ok" : fbdo.errorReason().c_str());
          } else {
            Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          }
          break; 
        case 8:
          if (readIntDB(buttonPath) != 1) {
            Serial.print("Current state of Button - ");
            Serial.print(buttonPressed);
            Serial.print(": ");
            Serial.println(readIntDB(buttonPath));
            
            Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 1) ? "ok" : fbdo.errorReason().c_str());
          } else {
            Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
          }
          break; 
        default:
          break;
      }
    }
  }

}

// Read a value from the database
int readIntDB(String buttonPath) {
  int value;
  if (Firebase.RTDB.getInt(&fbdo, buttonPath.c_str())) {
    value = fbdo.intData();
    Serial.print("Value: ");
    Serial.println(value);
  } else {
    Serial.println("Failed to read value from database");
    Serial.println(fbdo.errorReason());
    value = 0;
  }
  return value;
}

// deletes the stored wifi credentials and forces the esp32 to restart
void reconfigureWifi(bool wifiButtonPressed, int buttonPressed) { 
	
  if(wifiButton.pressed == true && buttonPressed == 7) {
    if (SPIFFS.remove(SSID_PATH)) {
      Serial.println("WIFI SSID deleted successfully");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WIFI SSID...");
      lcd.setCursor(0, 1);
      lcd.print("SSID DELETED");

    } else {
      Serial.println("Failed to delete file");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WIFI SSID...");
      lcd.setCursor(0, 1);
      lcd.print("DELETE FAILED");

    }
    delay(100);
    if (SPIFFS.remove(PASS_PATH)) {
      Serial.println("WIFI PASSWORD deleted successfully");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WIFI PASS...");
      lcd.setCursor(0, 1);
      lcd.print("PASS DELETED");

    } else {
      Serial.println("Failed to delete file");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("WIFI PASS...");
      lcd.setCursor(0, 1);
      lcd.print("DELETE FAILED");

    }
    delay(100);
    wifiButton.pressed = false;
    FirebaseJson json;
    String buttonPath = listenerPath.c_str() + dbButtons(buttonPressed);
    if (Firebase.ready()) {
      Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
    }
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CONFIGS DELETED");
    lcd.setCursor(0, 1);
    lcd.print("RESTARTING ESP");

    delay(3000);
    ESP.restart();
  }
}

void selectOperationMode(){
  if ((millis() - previousModeMillis) >= ONE_SECOND_MS) {
    if (modeSwitched == true) {
      switch (operationMode) {
        // Aquaponics Mode
        case 1: {
          // run function
          modeSwitched = false;
          Serial.println("Mode Select: Switching to Aquaponics Mode...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("SWITCHING TO: ");
          lcd.setCursor(0, 1);
          lcd.print("AQUAPONICS");
          delay(100);

          aquaponicsMode();
          // save operation state to preferences
          preferences.putInt("lastOperation", operationMode);
          
          break;
        }

        // Aquaculture Mode
        case 2: {
          // run function
          modeSwitched = false;
          Serial.println("Mode Select: Switching to Aquaculture Mode...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("SWITCHING TO: ");
          lcd.setCursor(0, 1);
          lcd.print("AQUACULTURE");
          delay(100);

          aquacultureMode();
          // save operation state to preferences
          preferences.putInt("lastOperation", operationMode);
          
          break;
        }

        // Hydroponics Mode
        case 3: {
          // run function
          modeSwitched = false;
          Serial.println("Mode Select: Switching to Hydroponics Mode...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("SWITCHING TO: ");
          lcd.setCursor(0, 1);
          lcd.print("HYDROPONICS");
          delay(100);

          hydroponicsMode();
          // save operation state to preferences
          preferences.putInt("lastOperation", operationMode);
          
          break;
        }

        // Sludge Transfer Mode
        case 4: {
          // run function
          Serial.println("Mode Select: Switching to Sludge Transfer Mode...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("SWITCHING TO: ");
          lcd.setCursor(0, 1);
          lcd.print("TRANSFER MODE");
          delay(100);

          transferMode();
          // save operation state to preferences
          //preferences.putInt("lastOperation", operationMode);
          // put modeSwitched = false; inside sludgeTransferMode() because it needs to keep executing it 
          break;
        }

        case 5: {
          Serial.println("Mode Select: Switching off all relays...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("SWITCHING TO: ");
          lcd.setCursor(0, 1);
          lcd.print("DEVICES OFF");
          delay(100);

          turnEverythingOff();
          preferences.putInt("lastOperation", operationMode);
          modeSwitched = false;
          break;
        }

        default:
          Serial.println("Mode Select: No operation mode selected...");
          Serial.println("Mode Select: Defaulting to Aquaponics Mode");
          operationMode = 1;
          preferences.putInt("lastOperation", operationMode);
          modeSwitched = false;
          break;
        }
    } else {
      Serial.println("Mode Select: No state changes detected...");
    }
    previousModeMillis = millis();
  }
} 

void readSensorData() {
  const int totalReadCount = 60;
  unsigned long currentMillis = millis();
  float tempReading, phReading, turbReading, oxyReading;
  //get sensor reading and add to the total every second for 180 seconds.
  if ((readCount < totalReadCount) && ((currentMillis - previousReadingTime)>= ONE_SECOND_MS)) {

    if (sensorsDisconnected != true) {

      tempReading = temp();
      phReading = ph();
      turbReading = turbidity();
      oxyReading = readDO(); // this is in micrograms/Liter
      if ((!isnan(tempReading))&& (!isnan(phReading)) && (!isnan(turbReading)) && (!isnan(oxygenLevel))) {
        tempSum += tempReading;
        phSum += phReading;
        turbSum += turbReading;
        oxySum += oxyReading;
        readCount++;

        Serial.print("SENSOR READING: Current temperature in C: ");
        Serial.println(tempReading);

        Serial.print("SENSOR READING: Current ph level: ");
        Serial.println(phReading);

        Serial.print("SENSOR READING: Current turbidity level in NTU: ");
        Serial.println(turbReading);

        Serial.print("SENSOR READING: Current dissolved oxygen level in in mg/L: ");
        Serial.println(oxyReading/1000);

        Serial.print("Read Count: ");
        Serial.println(readCount);

      } 
    } else {

      Serial.println("SENSORS: Sensors are disconnected... cannot read sensor data...");

    }

    previousReadingTime = millis();
  } else if ((readCount >= totalReadCount) && ((millis() - previousReadingTime >= ONE_SECOND_MS))) {
    if (sensorsDisconnected != true) {

      Serial.print("SENSOR READING: Sum of readings: ");

      Serial.println(tempSum);
      temperatureLevel = tempSum / totalReadCount;
      Serial.print("SENSOR READING: Average Temp in C for 60 Seconds: ");
      Serial.println(temperatureLevel);

      phLevel = phSum / totalReadCount;
      Serial.print("Sensor Reading: Average pH for 60 Seconds: ");
      Serial.println(phLevel);

      turbidityLevel = turbSum / totalReadCount;
      Serial.print("Sensor Reading: Average turbidity in NTUs for 60 Seconds: ");
      Serial.println(turbidityLevel);

      oxygenLevel = oxySum / totalReadCount;
      Serial.print("Sensor Reading: Average Dissolved Oxygen Level in mg/L for 60 Seconds: ");
      Serial.println(oxygenLevel/1000);

      dataReady = true;

      //reset the sum values and the counter
      tempSum = 0;                                                               
      phSum = 0;
      turbSum = 0;   
      oxySum = 0;
      previousSendingTime = 0;
      readCount = 0;
    } else {
      Serial.println("SENSORS: Sensors are disconnected... cannot get average of sensor data...");
    }
    
  }

  

}

void sendSensorData() {
  unsigned long currentMillis = millis();
  if (((currentMillis - previousSendingTime) >= SIXTY_SECOND_MS)) {
    if ((Firebase.ready()) && (dataReady == true)) {
      // Send new readings to database
      //Get current timestamp
      timestamp = getTime();
      Serial.print ("time: ");

      Serial.println(UnixToString(timestamp, timeFormat1));
      //Serial.println(UnixToString(timestamp, timeFormat1));
      parentPath = databasePath + "/" + String(timestamp);

      json.set(tempPath.c_str(), String(temperatureLevel));

      json.set(phPath.c_str(), String(phLevel));

      json.set(turbPath.c_str(), String(turbidityLevel));

      json.set(oxyPath.c_str(), String(oxygenLevel));

      json.set(timePath, String(timestamp));

    
      Serial.printf("\nSet json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
      
      dataReady = false;
    } else {
      Serial.println("WiFi or Firebase not connected, not sending data to database...");
    }
    previousSendingTime = currentMillis;
  }

}

// Initialize SPIFFS
bool initSPIFFS() {
  Serial.println("BOOT - SPIFFS INIT: Attempting to mount SPIFFS");
  if (!SPIFFS.begin(true)) {
    Serial.println("BOOT - SPIFFS INIT: An error has occurred while mounting SPIFFS");
    return false;
  }
  Serial.println("BOOT - SPIFFS INIT: mounted successfully");
  return true;
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char * path){
  Serial.printf("\nReading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return String();
  }
  
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;     
  }
  return fileContent;
}

// Write file to SPIFFS
void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("\nWriting file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
}
// Load values saved in SPIFFS
void loadSPIFFsFiles() {
  Serial.println("BOOT - LOADING SPIFFS FILES: ");
  ssid = readFile(SPIFFS, SSID_PATH);
  pass = readFile(SPIFFS, PASS_PATH);
  Serial.println(ssid);
  Serial.println(pass);
}

void beginI2C() {


  //Attempt to initialize MCP23017
  // uncomment appropriate mcp.begin
  Serial.println("BOOT - I2C DEVICE INIT: Attempting to connect MCP23017");
  if (!mcp.begin_I2C()) {
  //if (!mcp.begin_SPI(CS_PIN)) {
    Serial.println("BOOT - I2C DEVICE INIT: MCP23017: Error.");
    mcpConnected = false;
    while (1);
  }  else {
    Serial.println("BOOT - I2C DEVICE INIT: MCP23017: Connected.");
    mcpConnected = true;
  }


  //Attempt to initialize RTC
  Serial.println("BOOT - I2C DEVICE INIT: Attempting to connect RTC.");
  if (! rtc.begin()) {
    Serial.println("BOOT - I2C DEVICE INIT: Could not find RTC! Check circuit.");
    rtcConnected = false;
    while (1);
  } else {
    Serial.println("BOOT - I2C DEVICE INIT: RTC Successfully Connected.");
    rtcConnected = true;
  }

  //Attempt to initialize ADS
  if (!ads.begin()) {
    Serial.println("BOOT - I2C DEVICE INIT: Failed to initialize ADS.");
    adsConnected = false;
    while (1);
  } else {
    Serial.println("BOOT - I2C DEVICE INIT: RTC Successfully Connected.");
    adsConnected = true;
  }



  //Attempt to initialize LCD
  Serial.println("BOOT - I2C DEVICE INIT: Attempting to connect 16x2 LCD");

  lcd.begin(16, 2);
  lcdConnected = false; 
  lcdConnected = true;
  lcd.backlight(); //open the backlight 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("I2C Devices");
  lcd.setCursor(0, 1);
  lcd.print("STARTED");
  
  
}

bool initializePreferences() {
  bool success = preferences.begin("aquaponics", false);
  Serial.println("BOOT - PREFERENCES INIT: Initializing Preferences...");
  if (success == true) {
    Serial.println("BOOT - PREFERENCES INIT: Namespace opened sucessfully...");
  }
  else {
    Serial.println("BOOT - PREFERENCES INIT: Namespace was not opened");
  }
  return success;
}

void checkLastMode() {

  Serial.println("BOOT - LAST STATES: Fetching last states from ");
  lastOperationMode = preferences.getInt("lastOperation", 1);
  Serial.print("BOOT - LAST OPERATION: The last operation mode stored from preferences is: ");
  Serial.println(lastOperationMode);
  operationMode = lastOperationMode;

  scheduleStarted = preferences.getBool("scheduleStart", false);
  Serial.print("BOOT - LAST SCHEDULE STATE: Retrieving schedulerstate from preferences: ");
  Serial.println(scheduleStarted);

  scheduleStartTime = preferences.getULong("scheduleTime", 0);
  Serial.print("BOOT - LAST SCHEDULE START TIME: Retrieving schedule start time from preferences: ");
  Serial.println(scheduleStartTime);

  modeSwitched = true;
  
}
//would turn every device connected to the relays off, 
//if this was called by another mode function, switch flag would still be true
void turnEverythingOff(){ 
	
  Serial.println("Mode Switching: turning off all relays...");
    //220V
    //normally closed devices
    mcp.digitalWrite(WATER_PUMP_1, LOW);
    mcp.digitalWrite(WATER_PUMP_2, LOW);
    mcp.digitalWrite(AIR_PUMP_1, LOW);
    mcp.digitalWrite(AIR_PUMP_2, LOW);

    //normally open devices
    mcp.digitalWrite(AIR_PUMP_3, HIGH);
    mcp.digitalWrite(M_VALVE_1, HIGH);
    mcp.digitalWrite(M_VALVE_2, HIGH);
    mcp.digitalWrite(M_VALVE_3, HIGH);

    //12V
    //normally closed devices
    mcp.digitalWrite(S_VALVE_1, LOW);
    mcp.digitalWrite(S_VALVE_2, LOW);

     //normally open devices
    mcp.digitalWrite(S_VALVE_3, HIGH);
    mcp.digitalWrite(S_VALVE_4, HIGH);
    mcp.digitalWrite(S_VALVE_5, HIGH);
    mcp.digitalWrite(P_PUMP_1, HIGH);
    mcp.digitalWrite(P_PUMP_2, HIGH);
    mcp.digitalWrite(RELAY16, HIGH);
  if (operationMode == 5) {
    modeSwitched = false;
  } else{
    Serial.println("Mode Switching: proceeding with sequence...");
  }
}

void aquaponicsMode() {
  turnEverythingOff();
  delay(1000);
  Serial.println("MODE SWITCHING: Running in aquaponics mode...");

  Serial.println("AQUAPONICS MODE: Opening valves...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUAPONICS ");
  lcd.setCursor(0, 1);
  lcd.print("OPENING SVALVES");

  mcp.digitalWrite(S_VALVE_1, HIGH);       //normally closed
  mcp.digitalWrite(S_VALVE_2, HIGH);       //normally closed
  delay(1000);

  

  Serial.println("AQUAPONICS MODE: Turning on air pumps...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUAPONICS ");
  lcd.setCursor(0, 1);
  lcd.print("AIRPUMPS ON");

  mcp.digitalWrite(AIR_PUMP_1, HIGH);      //normally closed
  mcp.digitalWrite(AIR_PUMP_2, HIGH);      //normally closed
  delay(1000);
  
  Serial.println("AQUAPONICS MODE: Turning on water pumps...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUAPONICS ");
  lcd.setCursor(0, 1);
  lcd.print("WATERPUMPS ON");

  mcp.digitalWrite(WATER_PUMP_1, HIGH);    //normally closed
  mcp.digitalWrite(WATER_PUMP_2, HIGH);    //normally closed
  
  
  modeSwitched = false;
}

void aquacultureMode() {
  turnEverythingOff();
  delay(1000);
  Serial.println("MODE SWITCHING: Running in aquaculture mode...");
  
  Serial.println("AQUACULTURE MODE: Opening solenoid valves...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUACULTURE ");
  lcd.setCursor(0, 1);
  lcd.print("OPENING SVALVES");

  mcp.digitalWrite(S_VALVE_1, HIGH); //normally closed
  mcp.digitalWrite(S_VALVE_3, LOW); //normally open
  delay(1000);

  Serial.println("AQUACULTURE MODE: Turning on airpumps...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUACULTURE ");
  lcd.setCursor(0, 1);
  lcd.print("AIRPUMPS ON");

  mcp.digitalWrite(AIR_PUMP_1, HIGH); //normally closed
  mcp.digitalWrite(AIR_PUMP_2, HIGH); //normally closed
  delay(1000);

  Serial.println("AQUACULTURE MODE: Opening motorized valves...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUACULTURE ");
  lcd.setCursor(0, 1);
  lcd.print("OPENING MVALVES");
  mcp.digitalWrite(M_VALVE_1, LOW); //normally open
  delay(15000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUACULTURE ");
  lcd.setCursor(0, 1);
  lcd.print("MVALVE OPEN 4s");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUACULTURE ");
  lcd.setCursor(0, 1);
  lcd.print("MVALVE OPEN 3s");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUACULTURE ");
  lcd.setCursor(0, 1);
  lcd.print("MVALVE OPEN 2s");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUACULTURE ");
  lcd.setCursor(0, 1);
  lcd.print("MVALVE OPEN 1s");
  delay(1000);

  Serial.println("AQUACULTURE MODE: Turning on pumps...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AQUACULTURE ");
  lcd.setCursor(0, 1);
  lcd.print("WATERPUMPS ON");

  mcp.digitalWrite(WATER_PUMP_1, HIGH); //normally closed
  mcp.digitalWrite(WATER_PUMP_2, HIGH); //normally closed


  modeSwitched = false;
}

void hydroponicsMode() {
  turnEverythingOff();
  delay(1000);
  Serial.println("Mode Switching: Running in hydroponics mode...");

  Serial.println("HYDROPONICS MODE: Opening solenoid valves...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HYDROPONICS ");
  lcd.setCursor(0, 1);
  lcd.print("OPENING SVALVE4");
  //make sure S_VALVE_1 is closed for good
  mcp.digitalWrite(S_VALVE_1, LOW); //normally closed
  mcp.digitalWrite(S_VALVE_4, LOW); //normally open
  delay(1000);

  Serial.println("HYDROPONICS MODE: Turning on airpumps...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HYDROPONICS ");
  lcd.setCursor(0, 1);
  lcd.print("AIRPUMPS ON");

  mcp.digitalWrite(AIR_PUMP_1, HIGH); //normally closed
  mcp.digitalWrite(AIR_PUMP_2, HIGH); //normally closed
  delay(1000);

  Serial.println("HYDROPONICS MODE: Turning on sump pump...");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HYDROPONICS ");
  lcd.setCursor(0, 1);
  lcd.print("WATERPUMP_2 ON");

  mcp.digitalWrite(WATER_PUMP_2, HIGH); //normally closed

  modeSwitched = false;
}

void transferMode() {
  FirebaseJson json;
  String buttonPath = listenerPath.c_str() + dbButtons(buttonPressed);
  unsigned long currentTime;
  if (millis() - previousTransferCheck >= ONE_SECOND_MS) {

    if (ntpServerConnected == true) {

      currentTime = getTime();

    } else if (rtcConnected == true) {

      currentTime = rtc.now().unixtime();

    }

    if((settlingStarted == false) && (transferStarted == false)){ //check if settling was started, and if transfer is not started

      turnEverythingOff();

      Serial.println("TRANSFER MODE:  Initializing Sludge Transfer mode...");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TRANSFER MODE");
      lcd.setCursor(0, 1);
      lcd.print("Initializing");

      delay(500);

      Serial.println("TRANSFER MODE: Opening solenoid valves...");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TRANSFER MODE");
      lcd.setCursor(0, 1);
      lcd.print("OPENING SVALVES");
      
      mcp.digitalWrite(S_VALVE_1, HIGH); //normally closed

      //divert fishtank water to backup filter
      mcp.digitalWrite(S_VALVE_3, LOW); //normally open
      mcp.digitalWrite(S_VALVE_5, LOW); //normally open

      delay(1000);

      Serial.println("TRANSFER MODE: Turning on airpumps...");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TRANSFER MODE");
      lcd.setCursor(0, 1);
      lcd.print("AIRPUMPS ON");

      mcp.digitalWrite(AIR_PUMP_1, HIGH); //normally closed
      mcp.digitalWrite(AIR_PUMP_2, HIGH); //normally closed
      
      delay(1000);

      Serial.println("TRANSFER MODE: Turning on water pumps...");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TRANSFER MODE");
      lcd.setCursor(0, 1);
      lcd.print("WATERPUMPS ON");
      
      mcp.digitalWrite(WATER_PUMP_1, HIGH); //normally closed
      mcp.digitalWrite(WATER_PUMP_2, HIGH); //normally closed

      Serial.println("TRANSFER MODE:  Water flow diverted...");
      Serial.println("TRANSFER MODE:  Settling Started...");
      settlingStarted = true;
      settlingStartTime = currentTime;
      Serial.print("TRANSFER MODE: Settling started at: ");
      Serial.println(UnixToString(settlingStartTime, timeFormat1));
      
    }

    //wait 30-60 minutes
    if (((currentTime - settlingStartTime) >= SETTLING_TIME_S) && settlingStarted == true && transferStarted == false) {
      Serial.println("TRANSFER MODE:  Settling done");
      settlingStarted = false;

      Serial.println("TRANSFER MODE: Transfer started...");
      transferStarted = true;
      Serial.println("TRANSFER MODE: Opening motorized valves...");


      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TRANSFER MODE ");
      lcd.setCursor(0, 1);
      lcd.print("OPENING MVALVES");

      mcp.digitalWrite(M_VALVE_1, LOW); //normally open
      mcp.digitalWrite(M_VALVE_2, LOW); //normally open

      transferStartTime = currentTime;
      Serial.print("TRANSFER MODE: Transfer started at: ");
      Serial.println(UnixToString(transferStartTime, timeFormat1));

    } else if (settlingStarted == true && transferStarted == false){
      
      unsigned long timeLeft;
      timeLeft = (settlingStartTime + SETTLING_TIME_S) - currentTime;
      Serial.println("TRANSFER MODE: Settling in progress...");
      Serial.print("TRANSFER MODE: Time left is: ");
      Serial.print(timeLeft);
      Serial.println("s");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("SettleTime Left");
      lcd.setCursor(0, 1);
      lcd.print(timeLeft);

    } 

    //wait 5-10 minutes
    if ((transferStarted == true) && ((currentTime - transferStartTime) >= TRANSFER_TIME_S) && settlingStarted == false) {
      
      //back to aquaponics mode
      Serial.println("TRANSFER MODE: Transfer Done");

      Serial.println("TRANSFER MODE: Closing motorized valves: 30s");

      Serial.println("TRANSFER MODE: Opening motorized valves...");


      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TRANSFER MODE ");
      lcd.setCursor(0, 1);
      lcd.print("CLOSING MVALVES");

      mcp.digitalWrite(M_VALVE_1, HIGH); //normally open
      mcp.digitalWrite(M_VALVE_2, HIGH); //normally open

      if (valveClosingStarted == false) {

        valveClosingStarted = true;
        valveClosingStartTime = millis();
      }

      if (millis() - valveClosingStartTime >= THIRTY_SECOND_MS && valveClosingStarted == true) {

        Serial.println("TRANSFER MODE: Ending transfer mode....");
        Serial.println("TRANSFER MODE: Reverting back to aquaponics mode....");
        valveClosingStarted = false;
        transferStarted = false;

        aquaponicsButton.pressed = true;
        operationMode = 1;
        buttonPressed = 1;
        localSwitch = true;
        preferences.putInt("lastOperation", operationMode);

        

      } else if (millis() - valveClosingStartTime < THIRTY_SECOND_MS && valveClosingStarted == true && transferStarted == true) {
        unsigned long timeLeft;
        timeLeft = (valveClosingStartTime + THIRTY_SECOND_MS) - millis();
        Serial.println("TRANSFER MODE: Currently closing motorized valves valves...");
        Serial.print("TRANSFER MODE: Time left is: ");
        Serial.print(timeLeft/1000);
        Serial.println("s");
      }
      
    } else if (settlingStarted == false && transferStarted == true) {
      unsigned long timeLeft;
      timeLeft = (transferStartTime + TRANSFER_TIME_S) - currentTime;
      Serial.println("TRANSFER MODE: Transfer in progress...");
      Serial.print("TRANSFER MODE: Time left is: ");
      Serial.print(timeLeft);
      Serial.println("s");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TransfTime Left");
      lcd.setCursor(0, 1);
      lcd.print(timeLeft);
    } 

  }
  
}

void sensorDisconnect(bool buttonPress, int buttonPressed) {
  FirebaseJson json;
  String buttonPath = listenerPath.c_str() + dbButtons(buttonPressed);
  if (buttonPress == true && buttonPressed == 6) {
    if (sensorsDisconnected != true) {
      Serial.println("SENSORS TOGGLE: Disconnecting sensors from the system...");
      sensorsDisconnected = true;
      if(Firebase.ready()) {
        Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 1) ? "ok" : fbdo.errorReason().c_str());
      }
      localSwitch = true;
    } else {
      Serial.println("SENSORS TOGGLE: Connecting sensors to the system...");
      sensorsDisconnected = false;
      if(Firebase.ready()) {
        Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 0) ? "ok" : fbdo.errorReason().c_str());
      }
      localSwitch = true;
    }
  }
  sensorButton.pressed = false;
}

void startScheduler(bool buttonPress, int buttonPressed) {
  FirebaseJson json;
  String buttonPath = listenerPath.c_str() + dbButtons(buttonPressed);
  if (buttonPress == true && buttonPressed == 8){
    if (schedulerState != true) {
      Serial.println("SCHEDULE TOGGLE: Starting scheduler...");
      schedulerState = true;
      scheduleStarted = true;
      if(Firebase.ready()) {
        //update the stored schedule state
        Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, scheduleStatePath.c_str(), scheduleStarted) ? " scheduleStarted update ok" : fbdo.errorReason().c_str()); 
      }
      
    } else if (schedulerState == true && scheduleStarted == true){
      Serial.println("SCHEDULE TOGGLE: Stopping scheduler...");
      schedulerState = false;
      scheduleStarted = false;
      if(Firebase.ready()) {
        //update the stored schedule state
        Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, scheduleStatePath.c_str(), scheduleStarted) ? " scheduleStarted update ok" : fbdo.errorReason().c_str()); 
      }
    }
    //resets timer so checkSchedule() executes the code..
    previousScheduleCheckTime = 0;
    //make sure that this won't get triggered again without the button being actually pressed... probably not needed
    scheduleButton.pressed = false;
  }
}

void checkSchedule() {
  FirebaseJson json;
  //stores the database path for the scheduleButton entry
  String buttonPath = listenerPath.c_str() + dbButtons(8);
  unsigned long currentTime;
  if ((millis() - previousScheduleCheckTime) >= TEN_SECOND_MS) {
    //updates current time
    if(ntpServerConnected != true) {
    currentTime = rtc.now().unixtime();
    }else {
      currentTime = getTime();
    }

    //for debugging
    Serial.print("SCHEDULER: Is scheduler triggered? ");
    if (schedulerState != true) {
      Serial.println("FALSE");
    } else {
      Serial.println("TRUE");
    }
    Serial.println(schedulerState);
    Serial.print("SCHEDULER: Is schedule started? ");
    if (scheduleStarted != true) {
      Serial.println("FALSE");
    } else {
      Serial.println("TRUE");
    }

    /*Checks the state of the two flags:

    if both are true that means, scheduler has been started,
    schedulerState would be changed to false, and scheduleStarted would be kept to true,
    scheduleStartTime would be updated from currentTime
    and scheduledTime will be calculated by adding SCHEDULE_TIME_S to the scheduleStartTime
    scheduledStartTime will sent to the online database for storage and also stored in preferences.
    This will also be fetched from the database or preferences upon the boot of the system.

    if schedulerState is false, and scheduledStarted is true, it means that the scheduler is running
    it would check if currentTime is greater than or equal to the scheduledTime,
    if true, it means that the schedule has been met, and will switch the system into transferMode, 
      it will make it so that the transferButton has been pressed, 
      triggering checkInput() which would also update the buttonstate in the database
      allowing selectOperationMode() to be triggered which would call the transferMode() function
      it will then change scheduleStarted to false, and update the database entry, thus stopping the scheduler from running. 
    if false, it means that the scheduledTime has not been met yet 
      and would instead display the current time and the scheduled time and the remaining seconds.
    
    if schedulerState and scheduleStared are false, that means the scheduler has not been started yet.
    */
    if (schedulerState == true && scheduleStarted == true) {
      Serial.print("SCHEDULER: Scheduler has been started...");
      Serial.println("updating scheduler values...");
      /**/
      schedulerState = false;
      //this resets the button value in the database to false.
      //makes sure that the database would only receive this and not force the esp32 to read it again
      localSwitch = true;
      scheduleStarted = true; 
      //updates scheduleStartedState again just in case
      if(Firebase.ready()) {
      Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, scheduleStatePath.c_str(), scheduleStarted) ? " scheduleState update ok" : fbdo.errorReason().c_str());
      }
      preferences.putBool("scheduleStart", scheduleStarted);
      
      scheduleStartTime = currentTime;
      Serial.println("SCHEDULER: Scheduler Start time is: "); 
      Serial.println(UnixToString(scheduleStartTime, timeFormat1));
      if(Firebase.ready()) {
       Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, scheduleTimePath.c_str(), scheduleStartTime) ? " scheduleStartTime update ok" : fbdo.errorReason().c_str());
      }
      preferences.putULong("scheduleTime", scheduleStartTime);

      scheduledTime = scheduleStartTime + SCHEDULE_TIME_S;
      Serial.println("SCHEDULER: Scheduler end time is: "); 
      Serial.println(UnixToString(scheduledTime, timeFormat1));
      //forces checkSchedule to execute again immediately at the next iteration of loop
      previousScheduleCheckTime = 0;
    } else if (schedulerState == false) {
      if (scheduleStarted == true) {
        
        //updates scheduledTime again just in case.
        scheduledTime = scheduleStartTime + SCHEDULE_TIME_S;

        Serial.print("SCHEDULER: Scheduler is currently running..."); 
        Serial.print("SCHEDULER: Scheduler was started at: "); 
        Serial.println(UnixToString(scheduleStartTime, timeFormat1));
        Serial.print("SCHEDULER: Scheduler end time at: "); 
        Serial.println(UnixToString(scheduledTime, timeFormat1));

        
        //check if schedule has been met
        if(currentTime >= scheduledTime) {
          //trigger transferMode, this will reset any transferMode values
          Serial.println("SCHEDULER: Starting scheduled sludge transfer...");
          transferButton.pressed = true;
          buttonPressed = 4;
          operationMode = 4;
          settlingStarted = false;
          transferStarted = false;
          
          Serial.println("SCHEDULER: Finished at: ");
          Serial.print(UnixToString(currentTime, timeFormat1));
          scheduleStarted = false;
          schedulerState = false;
          scheduleTriggered = true;
          if(Firebase.ready()) {
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, buttonPath.c_str(), 0) ? " scheduleButtonState update ok" : fbdo.errorReason().c_str());
          Serial.printf("\nSet json... %s\n", Firebase.RTDB.set(&fbdo, scheduleStatePath.c_str(), scheduleStarted) ? " scheduleState update ok" : fbdo.errorReason().c_str());
          }
          preferences.putBool("scheduleStart", scheduleStarted);
          //makes sure that the update is only from the esp32
          localSwitch = true;
        } else {
          unsigned long timeLeft;
          timeLeft = scheduledTime - currentTime;
          Serial.print("SCHEDULER: Time until schedule completion is:..."); 
          Serial.println(UnixToString(scheduledTime, timeFormat1));
        }

      }
    } else if (schedulerState == false) {
       if (scheduleStarted == false) {
          Serial.print("SCHEDULER: Scheduler is not running..."); 
       }
    }


    //reset timer
    previousScheduleCheckTime = millis();
  }

}

int16_t readDO(){
  uint16_t raw;
  uint32_t voltage_mv; 
  uint8_t temperature_c;

  raw = ads.readADC_SingleEnded(OXYGEN_PIN);
  Serial.print("Dissolved Oxygen Raw value: ");
  Serial.println(raw);

  voltage_mv = raw * 0.125;
  Serial.print("Dissolved Oxygen Raw voltage: ");
  Serial.print(voltage_mv);
  Serial.println(" mV");

  sensors.requestTemperatures();
  temperature_c = sensors.getTempCByIndex(0);
  Serial.print("Dissolved Oxygen Detected temp: ");
  Serial.print(temperature_c);
  Serial.println(" C");

  if (TWO_POINT_CALIBRATION == 0) {
    uint16_t V_saturation = (uint32_t)CAL_VAL_V1 + (uint32_t)35 * temperature_c - (uint32_t)CAL_VAL_T1 * 35;
    // Serial.print("Dissolved Oxygen Raw voltage: ");
    // Serial.print((voltage_mv * DO_Table[temperature_c] / V_saturation))/1000;
    // Serial.println(" mg/L");
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
  } else {
    uint16_t V_saturation = (int16_t)((int8_t)temperature_c - CAL_VAL_T2) * ((uint16_t)CAL_VAL_V1 - CAL_VAL_V2) / ((uint8_t)CAL_VAL_T1 - CAL_VAL_T2) + CAL_VAL_V2;
    // Serial.print("Dissolved Oxygen Raw voltage: ");
    // Serial.print((voltage_mv * DO_Table[temperature_c] / V_saturation))/1000;
    // Serial.println(" mg/L");
    return (voltage_mv * DO_Table[temperature_c] / V_saturation);
  }
}

float ph(){
  int16_t raw;
  float voltage;
  float phVal;

  raw = ads.readADC_SingleEnded(PH_PIN);
  Serial.print("PH Raw value: ");
  Serial.println(raw);
  
  voltage = ads.computeVolts(raw);
  Serial.print("PH Raw voltage: ");
  Serial.print(voltage);
  Serial.println(" V");
  
  phVal = -14.374 * voltage + 31.789; //the calibration equation for getting the PH Value
  // Serial.print("PH Level: ");
  // Serial.print(phVal);
  // Serial.println(" pH");

  return phVal;
}

float turbidity() {
  int16_t raw;
  float voltage;
  float turbVal;

  raw = ads.readADC_SingleEnded(TURBIDITY_PIN);
  Serial.print("TURBIDITY Raw value: ");
  Serial.println(raw);

  voltage = ads.computeVolts(raw);
  Serial.print("TURBIDITY Raw voltage: ");
  Serial.print(voltage);
  Serial.println(" V");
  
  turbVal = (366.44*(voltage*voltage)) - (2105.9*voltage) + 3021;
  // Serial.print("Turbidity Level: ");
  // Serial.print(turbVal);
  // Serial.println(" NTU");

  return turbVal;
}

void displayLCD() {
  unsigned long currentTime;

  float tempC, phVal, turb, oxy;

  String currentMode;

  if (millis() - previousCycleTime >= TWO_SECOND_MS) {
    switch (operationMode) {
      // Aquaponics Mode
      case 1: {
        currentMode = "Aquaponics";
        break;
      }

      // Aquaculture Mode
      case 2: {
        currentMode = "Aquaculture";
        break;
      }

      // Hydroponics Mode
      case 3: {
        currentMode = "Hydroponics";
        break;
      }

      // Sludge Transfer Mode
      case 4: {
        currentMode = "Transfer Mode";
        break;
      }

      // Relay Devices OFF Mode
      case 5: {
        currentMode = "Devices OFF";
        break;
      }

      default:
        currentMode = "Aquaponics";
        break;
    }
    
    //updates current time
      if(ntpServerConnected != true) {
      currentTime = rtc.now().unixtime();
      }else {
      currentTime = getTime();
      }

    if (displayCount >= displayTotal) {
      //reset the display counter
      displayCount = 0;
    } else {
      switch(displayCount) {
        case 1: //display date and time in LCD
          
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(UnixToString(currentTime, timeFormat4));
          lcd.setCursor(0, 1);
          lcd.print(UnixToString(currentTime, timeFormat3));

          break;

        case 2: //display current operation mode in LCD

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Current Mode: ");
          lcd.setCursor(0, 1);
          lcd.print(currentMode);

          break;

        case 3: //displays temp and and ph level
          tempC = temp();
          phVal = ph();

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Temp: ");
          lcd.setCursor(6, 0);
          lcd.print(String(tempC));
          lcd.setCursor(0, 1);
          lcd.print("PH: ");
          lcd.setCursor(4, 1);
          lcd.print(String(phVal));

          break;

        case 4: //displays turbidity and dissolved oxygen
          turb = turbidity();
          oxy = (readDO())/1000;

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Turb: ");
          lcd.setCursor(6, 0);
          lcd.print(String(turb));
          lcd.setCursor(0, 1);
          lcd.print("Oxy: ");
          lcd.setCursor(5, 1);
          lcd.print(String(oxy));

          break;

        case 5: //displays wifi state
          
          lcd.clear();
          lcd.setCursor(0, 0);
          if (WiFi.status() == WL_DISCONNECTED) {
            lcd.print("NO WIFI");
          } else {
            lcd.print("WIFI CONNECTED");
          }
          lcd.setCursor(0, 1);
          if (WiFi.status() == WL_DISCONNECTED) {
            lcd.print("NO WIFI");
          } else {
            lcd.print(WiFi.localIP());
          }

          break;

        case 6: //displays scheduler state

          lcd.clear();
          lcd.setCursor(0, 0);
          if (scheduleStarted != true) {
            lcd.print("SCHEDULER OFF");
          } else {
            lcd.print("SCHEDULER ON");
          }
          lcd.setCursor(0, 1);
          if (scheduleStarted != true) {
            lcd.print("SCHEDULER OFF");
          } else {
            lcd.print(UnixToString(scheduledTime, timeFormat5));
          }

          break;

        case 7: //displays sensors state

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Sensors: ");
          lcd.setCursor(0, 1);
          if(sensorsDisconnected != true) {
            lcd.print("CONNECTED");
          } else {
            lcd.print("DISCONNECTED");
          }
          

          break;

        case 8: //displays transfer mode state

          lcd.clear();
          lcd.setCursor(0, 0);
          if (operationMode != 4) {
            lcd.print("TRANSFER OFF");
          } else {
            lcd.print("TRANSFER ON");
          }
          lcd.setCursor(0, 1);
          if (operationMode != 4) {
            lcd.print("TRANSFER OFF");
          } else {
            if(settlingStarted == true) {
              lcd.print("SETTLING...");
            }
            if(transferStarted == true) {
              lcd.print("TRANSFERING...");
            }
          }

          break;

        case 9:

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(UnixToString(currentTime, timeFormat3));
          lcd.setCursor(0, 1);
          lcd.print(currentMode);

          break;
        
        default: 
          break;
      }
      displayCount++;
    }

    previousCycleTime = millis();
  }
  
}

void backupPump() {
  float oxygenLevel;
 
  if (millis() - previousOxygenCheck >= SIXTY_SECOND_MS) {
    oxygenLevel = (readDO())/1000;

    if(oxygenLevel < 5.5) {
      Serial.println("Oxygen level lower than 5.5mg/L");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("BACKUP AIRPUMP");
      lcd.setCursor(0, 1);
      lcd.print("TURNING ON");
      
      if (backupAirPumpState == false) {
        Serial.println("Turning on backup airpump");
        mcp.digitalWrite(AIR_PUMP_3, HIGH);
      } else {
        Serial.println("Keeping backup airpump on");
      }
      
      backupAirPumpState = true;
    } else{
      Serial.println("Oxygen level higher than 5.5 mg/L");

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("BACKUP AIRPUMP");
      lcd.setCursor(0, 1);
      lcd.print("TURNING OFF");

      if (backupAirPumpState == true) {
        Serial.println("Turning off backup airpump");
        mcp.digitalWrite(AIR_PUMP_3, LOW);
        
      } else {
        Serial.println("Keeping backup airpump of");
      }

      backupAirPumpState = false;
    }

    previousOxygenCheck = millis();
  }
  
}

void whichButtonPressed () {
  if (millis() - previousStateCheckTime >= 1000) {

    Serial.print("Aquaponics button: ");
    Serial.println(aquaponicsButton.pressed);
    Serial.print("Aquaculture button: ");
    Serial.println(aquacultureButton.pressed);
    Serial.print("Hydroponics button: ");
    Serial.println(hydroponicsButton.pressed);
    Serial.print("transfer button: ");
    Serial.println(transferButton.pressed);
    Serial.print("Relay button: ");
    Serial.println(relayButton.pressed);
    
    if (buttonCount >= buttonTotal) {
      buttonCount = 0;
    } else {
      switch (buttonCount)
      {
      case 1:
        if (aquaponicsButton.pressed == true) {
          Serial.println("Aquaponics button was pressed...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("AQUAPONICS");
          lcd.setCursor(0, 1);
          lcd.print("BUTTON PRESSED");

        }
        break;
      case 2:
        if (aquacultureButton.pressed == true) {
          Serial.println("Aquaculture button was pressed...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("AQUACULTURE");
          lcd.setCursor(0, 1);
          lcd.print("BUTTON PRESSED");
          
        }
        break;
      
      case 3:
        if (hydroponicsButton.pressed == true) {
          Serial.println("Hydroponics button was pressed...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("HYDROPONICS");
          lcd.setCursor(0, 1);
          lcd.print("BUTTON PRESSED");
          
        }
        break;
      
      case 4:
        if (transferButton.pressed == true) {
          Serial.println("Transfer button was pressed...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("TRANSFER");
          lcd.setCursor(0, 1);
          lcd.print("BUTTON PRESSED");
          
        }
        break;

      case 5:
        if (transferButton.pressed == true) {
          Serial.println("Transfer button was pressed...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("TRANSFER");
          lcd.setCursor(0, 1);
          lcd.print("BUTTON PRESSED");
          
        }
        break;

      case 6:
        if (sensorButton.pressed == true) {
          Serial.println("Sensor button was pressed...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("SENSOR");
          lcd.setCursor(0, 1);
          lcd.print("BUTTON PRESSED");
          
        }
        break;

      case 7:
        if (wifiButton.pressed == true) {
          Serial.println("WiFi button was pressed...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("WIFI");
          lcd.setCursor(0, 1);
          lcd.print("BUTTON PRESSED");
          
        }
        break;

      case 8:
        if (scheduleButton.pressed == true) {
          Serial.println("Schedule button was pressed...");

          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("SCHEDULE");
          lcd.setCursor(0, 1);
          lcd.print("BUTTON PRESSED");
          
        }
        break;
      
      default:
        break;
      }
      buttonCount++;
    }

    previousStateCheckTime = millis();
  }
  
}
