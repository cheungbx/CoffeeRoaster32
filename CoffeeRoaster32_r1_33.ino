// ESP32 Auto Coffee Bean Roaster 

// 1. Core Arduino and ESP32 Network includes MUST go first
#include <Arduino.h>
#include <WiFi.h>          
#include <EEPROM.h> 
#include <ESP32Servo.h>    // ESP32 hardware PWM timer-compatible servo library
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_mac.h"       // Native ESP-IDF hardware register library for eFuse MAC extraction
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MAX31865.h> //for pt100 temperature sensor

const String RELEASE_VERSION = "1.33"; // INCREMENTED: Release version bump


/*
Important (RTD Jumper Settings) on MAX31865 to connect to a 3-wire Temperature Sensor pt100 
Ensure the solder jumpers on your MAX31865 breakout board match the physical PT100 probe you are using:
For a 3-wire PT100 probe (most common for roasters): Cut the 24 Wire trace, solder the 3 Wire jumper closed, and solder the 2/3 Wire jumper closed.
For a 2-wire / 4-wire PT100 probe: Configure the jumpers according to your manufacturer's breakout schematic.

Screw Terminal  Connection
F+              Matched Wire 1 (e.g., Blue)
RTD+            Matched Wire 2 (e.g., Blue)
RTD-            Unmatched Wire (e.g., Red)
F-              Leave empty (The on-board jumpers handle bridging this)

Wiring Reference Table
Peripheral    Pin  ESP32   Target Pin Pin Function / Notes

MAX31865      VCC/VIN 3.3V    Power (3.3V logic)
MAX31865      GND     GND     Ground Reference
MAX31865      CLK     GPIO 14 Software SPI Clock (CLK)
MAX31865      SDO     GPIO 25 Software SPI Master In Slave Out (MISO)
MAX31865      SDI     GPIO 23 Software SPI Master Out Slave In (MOSI)
MAX31865      CS      GPIO 5  Software SPI Chip Select (CS)
MAX31865      RDY     NC      No need to connect, it pulls low when temperature data is ready for collectioon. Adafruit library handles polling directly over SPI.
OLED SSD1306
SSD1306       VCC     3.3V    Power (3.3V)
SSD1306       GND     GND     Ground Reference
SSD1306       SDA     GPIO 26 Custom I2C Data Line
SSD1306       SCL     GPIO 22 Custom I2C Clock Line
Servo's       Brown   GND
servo's       Red     3.3V
Temperature Servo     GPIO16
Time   Servo          GPIO17
OnOff Servo           GPIO18
Fan Servo             GPIO19
*/

// 2. Add explicit type mapping for legacy libraries to avoid core breakages
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int32_t s32_t;

// 3. Temporarily silence unused parameter warnings inside the third-party stack
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <AsyncTCP.h>      
#include <ESPAsyncWebServer.h>
#pragma GCC diagnostic pop 

// --- Configuration Flags ---
const int DebugLevel = 2; // 1 - summary, 2 - detail. 3 - show wifi password.

// --- ESP32 Hardware Pins ---
const int TemperaturePin = 16; 
const int TimePin        = 17; 
const int OnOffPin       = 18; 
const int FanPin         = 19; 
const int FanPressDegree = 45;
const int OnOffPressDegree = 50;
const int pressTime = 300;
const int afterPressTime = 300;

// --- MAX31865 PT100 Hardware Configuration (Release 1.26) ---
const int MAX31865_CS   = 5;
const int MAX31865_SDI  = 23; // MOSI
const int MAX31865_SDO  = 25; // MISO
const int MAX31865_CLK  = 14;

#define RREF      430.0
#define RNOMINAL  100.0

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_SDI, MAX31865_SDO, MAX31865_CLK);

// --- SSD1306 I2C OLED Configuration (Release 1.26) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
const int OLED_SDA = 26; // Avoiding GPIO 21 as requested
const int OLED_SCL = 22;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- Servo Instances ---
Servo ServoTemperature;
Servo ServoTime;
Servo ServoOnOff;
Servo ServoFan;

// --- Kill Switch Configuration (Release 1.33) ---
const int KillSwitchPin = 4;
volatile bool killSwitchPressed = false;
bool awaitingCoolDownConfirm = false;
unsigned long killSwitchPromptTime = 0; // Added for 20-second timeout tracking

// --- AP Default Fallback Configuration ---
const char* defaultPassword = "99819872"; 
String apSsid = "";

String clientSsid = "";
String clientPassword = "";

bool clientLocked = false;
IPAddress allowedClientIP;

AsyncWebServer server(80);

// --- Engine States ---
enum EngineState { WIFI_CONFIG_AP, MENU_SELECTION, Roasting, Cooldown, Done };
EngineState currentState = MENU_SELECTION;

enum OpModeOption { MODE_NONE = 0, MODE_STANDALONE = 1, MODE_WIFI = 2 };
OpModeOption operationMode = MODE_WIFI; 

bool isPaused = false;

// --- Roasting Data Structs ---
struct RoastingInstruction {
  unsigned long timeInSeconds;
  int temperature;
  int fanSpeed;
};

String beanName = "Default Arabica";
RoastingInstruction instructions[10];
int instructionCount = 0;
String rawProfileInput = "";

String syntaxErrorMsg = "";
bool hasSyntaxError = false;
bool isConfirmed = false;

// --- Automation State Tracking ---
int currentInstructionIdx = -1;
unsigned long stepTimer = 0;
unsigned long stepDuration = 0;
int currentRemainingTimeSec = 0;
int totalRemainingTimeSec = 0;

int lastTemperature = 230; 
int lastFanSpeed = 3;      

EngineState lastLoggedState = MENU_SELECTION;
int lastInstructionLogged = -1;

// --- EEPROM Layout Address Mapping ---
const int EEPROM_SIZE = 1024;
const int ADDR_OP_MODE      = 0;  
const int ADDR_WIFI_MARKER  = 1;
const int ADDR_WIFI_SSID    = 2;   
const int ADDR_WIFI_PASS    = 34;
const int ADDR_PROFILE_MARKER = 100;
const int ADDR_PROFILE_TEXT = 101;

const uint8_t VALID_WIFI_MAGIC   = 0xBB;
const uint8_t VALID_PROFILE_MAGIC = 0xCC;

// --- Asynchronous Web Event Action Flags ---
volatile bool triggerRun = false;
volatile bool triggerPause = false;
volatile bool triggerReset = false;
volatile bool triggerEraseAll = false;
volatile bool triggerSaveConfig = false;
volatile bool triggerSwitchMode = false;
OpModeOption targetSwitchMode = MODE_WIFI;

// FIXED: Added non-blocking asynchronous state synchronization to protect the Watchdog from thread starvation
volatile bool triggerHardwareSync = false;
volatile bool syncTimeFlag = false;
volatile bool syncTempFlag = false;
volatile bool syncFanFlag = false;
int targetSyncTemp = 230;
int targetSyncFan = 3;

String pendingSsid = "";
String pendingPassword = "";
OpModeOption pendingOpMode = MODE_WIFI;

// Variable tracking last OLED update timer
unsigned long lastOledUpdate = 0;

// Flag to capture if the network connection process is currently running
bool isConnectingNetwork = false;

// --- Forward Declarations ---
String getFormattedTime(int totalSeconds);
void executeServoPress(Servo &srv, int degree);
void adjustTemperature(int targetTemp);
void adjustFanSpeed(int targetSpeed);
void setMaximumRoastingTime();
void setMiniimumRoastingTime();
void parseProfile(String input);
void saveProfileToEEPROM();
void updateOledDisplay();

void IRAM_ATTR handleKillSwitch() {
  killSwitchPressed = true;
}

String getFormattedTime(int totalSeconds) {
  int minutes = totalSeconds / 60;
  int seconds = totalSeconds % 60;
  char buf[16];
  sprintf(buf, "%02d:%02d", minutes, seconds);
  return String(buf);
}

void computeDynamicAPProperties() {
  uint8_t mac[6];
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
    char macStr[7];
    snprintf(macStr, sizeof(macStr), "%02X%02X%02X", mac[3], mac[4], mac[5]);
    apSsid = "Bean" + String(macStr);
  } else {
    apSsid = "BeanXXXXXX";
  }
}

// --- Servo Manipulations ---
void executeServoPress(Servo &srv, int degree) {
  srv.write(degree);
  delay(pressTime);
  srv.write(0);
  delay(afterPressTime);
}

void setMaximumRoastingTime() {
  if (DebugLevel >= 1) {
    Serial.printf("[%s] [Hardware] Turning Time Servo Clockwise to Max (20 mins)\n", getFormattedTime(totalRemainingTimeSec).c_str());
  }
  ServoTime.write(180); 
  delay(1200); 
  ServoTime.write(90); 
  delay(300);
}

void setMinimumRoastingTime() {
  if (DebugLevel >= 1) {
    Serial.printf("[%s] [Hardware] Turning Time Servo Anti-clockwise to Min (1 min)\n", getFormattedTime(totalRemainingTimeSec).c_str());
  }
  ServoTime.write(0); 
  delay(6000); 
  ServoTime.write(90); 
  delay(300);
}

void adjustTemperature(int targetTemp) {
  float ratio = 43;
  float ratioR = 41;
  float baseRatio = 45;
  int baseTemperature;
  long duration;
  int diff;

  if (targetTemp < 150) targetTemp = 150;
  if (targetTemp > 240) targetTemp = 240;
 
  // First adjust to base Temperature from last temperature
  baseTemperature = (targetTemp < (150+240)/2 ) ? 150 : 240;
  diff = baseTemperature - lastTemperature;
  duration = abs(diff) * baseRatio; // use a bigger ratio to ensure turning to 150 or 240

  if (diff != 0) {
     if (DebugLevel >= 1) {
      Serial.printf("[%s] [Hardware] Adjust to baseTemp %dC from lastTemp %dC. Diff: %d. Running servo for %ld ms\n", getFormattedTime(totalRemainingTimeSec).c_str(), baseTemperature,  lastTemperature, diff, duration);
      }

    if (diff > 0) {
      ServoTemperature.write(180); // half speed 180 - 90/2.
    } else {
      ServoTemperature.write(0);  // half speed 90/2.
    }
    delay(duration);
    ServoTemperature.write(90);    
    delay(300);
  }
  // then adjust from base Temperature to target Temperature to have greater accuracy.
  diff = targetTemp - baseTemperature;

  if (diff != 0) {
    if (diff > 0) { 
      duration = abs(diff) * ratio; // ratio degree to servo ms 
    } else {
      duration = abs(diff) * ratioR;
    }

  if (DebugLevel >= 1) {
    Serial.printf("[%s] [Hardware] Adjust Temp to %dC from baseTemp %dC. Diff: %d. Running servo for %ld ms\n", getFormattedTime(totalRemainingTimeSec).c_str(), targetTemp, baseTemperature,  diff, duration);
  }

  if (diff > 0) {
    ServoTemperature.write(180);
  } else {
    ServoTemperature.write(0);
  }
  
  delay(duration);
  ServoTemperature.write(90);    
  }  
  lastTemperature = targetTemp;

}

void adjustFanSpeed(int targetSpeed) {
  if (targetSpeed < 1) targetSpeed = 1;
  if (targetSpeed > 3) targetSpeed = 3;
  
  int presses = 0;
  if (targetSpeed > lastFanSpeed) {
    presses = targetSpeed - lastFanSpeed;
  } else if (targetSpeed < lastFanSpeed) {
    presses = (3 - lastFanSpeed) + targetSpeed;
  }
  
  if (presses > 0) {
    if (DebugLevel >= 1) {
      Serial.printf("[%s] [Hardware] Adjusting Fan from %d to %d requiring %d presses\n", getFormattedTime(totalRemainingTimeSec).c_str(), lastFanSpeed, targetSpeed, presses);
    }
    for (int i = 0; i < presses; i++) {
      executeServoPress(ServoFan, FanPressDegree);
    }
  }
  lastFanSpeed = targetSpeed;
}

// --- Profile Parsing Utilities ---
void parseProfile(String input) {
  syntaxErrorMsg = "";
  hasSyntaxError = false;
  instructionCount = 0;
  
  input.replace("\r", "");
  int lineStart = 0;
  int lineIdx = 0;
  while (lineStart < (int)input.length() && instructionCount < 10) {
    int lineEnd = input.indexOf('\n', lineStart);
    if (lineEnd == -1) lineEnd = input.length();
    
    String line = input.substring(lineStart, lineEnd);
    line.trim();
    if (line.length() > 0) {
      if (lineIdx == 0) {
        beanName = line;
        if (beanName.length() > 50) {
          beanName = beanName.substring(0, 50);
        }
      } else {
        int firstSpace = line.indexOf(' ');
        if (firstSpace == -1) {
          hasSyntaxError = true;
          syntaxErrorMsg = "Syntax error on instruction line " + String(lineIdx) + ": Missing space delimiter.";
          return;
        }
        
        String timePart = line.substring(0, firstSpace);
        timePart.trim();
        int colon = timePart.indexOf(':');
        if (colon == -1) {
          hasSyntaxError = true;
          syntaxErrorMsg = "Syntax error on line " + String(lineIdx) + ": Time must use mm:ss format.";
          return;
        }
        
        int mins = timePart.substring(0, colon).toInt();
        int secs = timePart.substring(colon + 1).toInt();
        unsigned long totalSecs = (mins * 60) + secs;
        String remainingPart = line.substring(firstSpace + 1);
        remainingPart.trim();
        
        int cPos = remainingPart.indexOf('C');
        if (cPos == -1) {
          hasSyntaxError = true;
          syntaxErrorMsg = "Syntax error on line " + String(lineIdx) + ": Missing 'C' marker for Temperature.";
          return;
        }
        
        int tempVal = remainingPart.substring(0, cPos).toInt();
        if (tempVal < 150 || tempVal > 240) {
          hasSyntaxError = true;
          syntaxErrorMsg = "Error line " + String(lineIdx) + ": Temperature out of range (150C-240C).";
          return;
        }
        
        int fanVal = 3;
        int fPos = remainingPart.indexOf('F');
        if (fPos != -1) {
          int startFanIdx = cPos + 1;
          String fanPart = remainingPart.substring(startFanIdx, fPos);
          fanPart.trim();
          if (fanPart.length() > 0) {
            fanVal = fanPart.toInt();
          }
          if (fanVal < 1 || fanVal > 3) {
            hasSyntaxError = true;
            syntaxErrorMsg = "Error line " + String(lineIdx) + ": Fan speed must be 1, 2, or 3.";
            return;
          }
        }
        
        instructions[instructionCount].timeInSeconds = totalSecs;
        instructions[instructionCount].temperature = tempVal;
        instructions[instructionCount].fanSpeed = fanVal;
        instructionCount++;
      }
      lineIdx++;
    }
    lineStart = lineEnd + 1;
  }
  
  if (instructionCount == 0 && !hasSyntaxError) {
    hasSyntaxError = true;
    syntaxErrorMsg = "Profile input payload contains zero instruction maps.";
  }
}

void recalculateDynamicRemainingTime() {
  if (currentState == MENU_SELECTION) {
    totalRemainingTimeSec = 0;
    for (int i = 0; i < instructionCount; i++) {
      totalRemainingTimeSec += instructions[i].timeInSeconds;
    }
  } else if (currentState == Roasting) {
    int accum = currentRemainingTimeSec;
    for (int i = currentInstructionIdx + 1; i < instructionCount; i++) {
      accum += instructions[i].timeInSeconds;
    }
    totalRemainingTimeSec = accum;
  }
}

// --- EEPROM Interfacing Functions ---
void saveProfileToEEPROM() {
  EEPROM.write(ADDR_PROFILE_MARKER, VALID_PROFILE_MAGIC);
  int len = rawProfileInput.length();
  if (len > 800) len = 800; 
  
  for (int i = 0; i < len; i++) {
    EEPROM.write(ADDR_PROFILE_TEXT + i, rawProfileInput[i]);
  }
  EEPROM.write(ADDR_PROFILE_TEXT + len, 0); 
  EEPROM.commit();
}

void loadProfileFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(ADDR_PROFILE_MARKER) == VALID_PROFILE_MAGIC) {
    rawProfileInput = "";
    for (int i = 0; i < 800; i++) {
      char c = EEPROM.read(ADDR_PROFILE_TEXT + i);
      if (c == 0) break;
      rawProfileInput += c;
    }
    if (rawProfileInput.length() > 0) {
      parseProfile(rawProfileInput);
      recalculateDynamicRemainingTime();
    }
  }
}

void saveOpModeToEEPROM(OpModeOption mode) {
  EEPROM.write(ADDR_OP_MODE, (uint8_t)mode);
  EEPROM.commit();
}

OpModeOption loadOpModeFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  uint8_t modeByte = EEPROM.read(ADDR_OP_MODE);
  if (modeByte == (uint8_t)MODE_STANDALONE) return MODE_STANDALONE;
  if (modeByte == (uint8_t)MODE_WIFI) return MODE_WIFI;
  return MODE_NONE;
}

void saveWifiToEEPROM(String ssidStr, String passStr) {
  EEPROM.write(ADDR_WIFI_MARKER, VALID_WIFI_MAGIC);
  for (int i = 0; i < 32; i++) EEPROM.write(ADDR_WIFI_SSID + i, 0);
  for (int i = 0; i < 64; i++) EEPROM.write(ADDR_WIFI_PASS + i, 0);
  for (size_t i = 0; i < ssidStr.length() && i < 31; i++) {
    EEPROM.write(ADDR_WIFI_SSID + i, ssidStr[i]);
  }
  for (size_t i = 0; i < passStr.length() && i < 63; i++) {
    EEPROM.write(ADDR_WIFI_PASS + i, passStr[i]);
  }
  EEPROM.commit();
}

void loadWifiFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  
  if (EEPROM.read(ADDR_WIFI_MARKER) == VALID_WIFI_MAGIC) {
    clientSsid = "";
    for (int i = 0; i < 32; i++) {
      char c = EEPROM.read(ADDR_WIFI_SSID + i);
      if (c == 0 || c == 0xFF) break; 
      clientSsid += c;
    }
    clientPassword = "";
    for (int i = 0; i < 64; i++) {
      char c = EEPROM.read(ADDR_WIFI_PASS + i);
      if (c == 0 || c == 0xFF) break;
      clientPassword += c;
    }
  } else {
    clientSsid = "";
    clientPassword = "";
  }
}

void eraseAllEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
}

void executeStandaloneAPProcess() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid.c_str(), defaultPassword);
  Serial.printf("[%s] [Network Status] Operational Mode: STANDALONE AP\n", getFormattedTime(totalRemainingTimeSec).c_str());
  Serial.printf("[%s] [Network Status] Broadcasting Local SSID: %s\n", getFormattedTime(totalRemainingTimeSec).c_str(), apSsid.c_str());
  Serial.printf("[%s] [Network Status] AP IP Address: %s\n", getFormattedTime(totalRemainingTimeSec).c_str(), WiFi.softAPIP().toString().c_str());
}

bool executeWifiConnectionProcess() {
  isConnectingNetwork = true;
  updateOledDisplay(); // Draw the splash connection screen immediately
  
  WiFi.disconnect(true); 
  delay(100);
  WiFi.mode(WIFI_STA);
  Serial.printf("[%s] [Network Status] Operational Mode: WIFI STATION\n", getFormattedTime(totalRemainingTimeSec).c_str());
  Serial.printf("[%s] [Network Status] Connecting to target SSID: %s\n", getFormattedTime(totalRemainingTimeSec).c_str(), clientSsid.c_str());
  if (DebugLevel >= 3) {
    Serial.printf("[%s] [Network Status] with Password: %s\n", getFormattedTime(totalRemainingTimeSec).c_str(), clientPassword.c_str());
  }
  WiFi.begin(clientSsid.c_str(), clientPassword.c_str());
  
  unsigned long startAttempt = millis();
  int dots = 0;
  while (WiFi.status() != WL_CONNECTED && (millis() - startAttempt < 30000UL)) {
    delay(500);
    yield(); 
    Serial.print(".");
    if (++dots % 20 == 0) Serial.println();
  }
  Serial.println();
  
  isConnectingNetwork = false;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("[%s] [Network Error] Connection failed or timed out after 30s.\n", getFormattedTime(totalRemainingTimeSec).c_str());
    EEPROM.write(ADDR_OP_MODE, (uint8_t)MODE_NONE);
    EEPROM.commit();
    delay(1000);
    ESP.restart();
    return false;
  }
  
  Serial.printf("[%s] [Network Status] Successfully connected to network!\n", getFormattedTime(totalRemainingTimeSec).c_str());
  Serial.printf("[%s] [Network Status] Assigned Station IP Address: %s\n", getFormattedTime(totalRemainingTimeSec).c_str(), WiFi.localIP().toString().c_str());
  return true;
}

// --- Asynchronous HTML Interfaces Generation ---
String generateWifiSetupHtml() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Coffee Roaster Setup " + RELEASE_VERSION + "</title>"; 
  html += "<style>body { font-family: Arial; text-align: center; margin-top: 50px; background-color: #f7f9fa; }";
  html += ".card { background: white; padding: 30px; max-width: 350px; margin: auto; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }";
  html += "input[type=text], input[type=password], select { width: 90%; padding: 10px; margin: 10px 0; font-size: 14px; }";
  html += "input[type=submit] { background-color: #795548; color: white; padding: 12px; border: none; border-radius: 4px; cursor: pointer; width: 96%; font-size: 16px; }</style>";
  html += "</head><body><div class='card'>";
  html += "<h2>Roaster Infrastructure Setup</h2>";
  html += "<form action='/save_config' method='POST'>";
  html += "<label style='float:left; margin-left:5%; font-size:13px;'>Operation Mode:</label>";
  html += "<select name='opmode'>";
  html += "  <option value='2' selected>WIFI</option>";
  html += "  <option value='1'>Standalone</option>";
  html += "</select><br>";
  html += "<input type='text' name='ssid' placeholder='WiFi SSID' value='" + clientSsid + "'><br>";
  html += "<input type='password' name='password' placeholder='WiFi Password' value='" + clientPassword + "'><br><br>";
  html += "<input type='submit' value='Save Configurations'>";
  html += "</form></div></body></html>";
  return html;
}

String generateHtml() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Coffee Roaster " + RELEASE_VERSION + "</title>"; 
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; text-align: center; background-color: #faf8f5; margin:20px; color: #3e2723; }";
  html += ".highlight { background-color: #ffe0b2; font-weight: bold; padding: 5px; border-radius: 4px; }";
  html += "textarea, button, select, input { padding: 10px; font-size: 16px; margin: 5px; }";
  html += "textarea { width: 90%; max-width: 360px; height: 180px; font-family: monospace; }";
  html += "button { background-color: #795548; color: white; border: none; cursor: pointer; border-radius: 4px;}";
  html += "button.pause { background-color: #ff9800; }";
  html += "button.reset { background-color: #f44336; }";
  html += "button.reset-confirm { background-color: #b71c1c; font-weight: bold; }"; 
  html += "button.erase-init { background-color: #757575; font-size: 13px; padding: 6px 12px; margin: 5px auto; display: block; }";
  html += "button.erase-confirm { background-color: #d32f2f; font-size: 13px; padding: 6px 12px; font-weight: bold; margin: 5px auto; display: none; }";
  html += "button.switch-init { background-color: #0288d1; font-size: 13px; padding: 6px 12px; margin: 5px auto; display: block; }";
  html += "button.switch-confirm { background-color: #01579b; font-size: 13px; padding: 6px 12px; font-weight: bold; margin: 5px auto; display: none; }";
  html += "button.update-btn { background-color: #4caf50; font-size: 14px; padding: 6px 10px; margin: 2px; }";
  html += ".card { background: white; padding: 20px; max-width: 440px; margin: auto; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); border-top: 5px solid #5d4037; }";
  html += ".error { color: #d32f2f; font-weight: bold; margin: 10px 0; }";
  html += ".spinner { display: inline-block; width: 16px; height: 16px; border: 3px solid #ffe0b2; border-radius: 50%; border-top-color: #795548; animation: spin 1s ease-in-out infinite; vertical-align: middle; margin-left: 5px; }";
  html += "@keyframes spin { to { transform: rotate(360deg); } }";
  html += "</style>";
  
  html += "<script>";
  if (currentState != MENU_SELECTION || isConfirmed) {
    html += "var reloadTimer = setInterval(function() {";
    html += "  if(document.activeElement && (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'SELECT')) return;";
    html += "  window.location.reload();";
    html += "}, 3500);";
  }
  html += "function exposeConfirmButton() {";
  html += "  document.getElementById('eraseInitBtn').style.display = 'none';";
  html += "  document.getElementById('eraseConfirmBtn').style.display = 'block';";
  html += "}";
  html += "function exposeWifiConfirmButton() {";
  html += "  document.getElementById('switchWifiInitBtn').style.display = 'none';";
  html += "  document.getElementById('switchWifiConfirmBtn').style.display = 'block';";
  html += "}";
  html += "function exposeStandaloneConfirmButton() {";
  html += "  document.getElementById('switchStandaloneInitBtn').style.display = 'none';";
  html += "  document.getElementById('switchStandaloneConfirmBtn').style.display = 'block';";
  html += "}";
  html += "function exposeRestartConfirmButton() {";
  html += "  document.getElementById('restartInitBtn').style.display = 'none';";
  html += "  document.getElementById('restartConfirmBtn').style.display = 'inline-block';";
  html += "}";
  html += "</script>";
  html += "</head><body><div class='card'>";
  html += "<h2>Coffee Roaster " + RELEASE_VERSION + "</h2>";

  html += "<p style='font-size: 18px; margin-bottom: 5px;'><strong>Status:</strong> ";
  if (isPaused) {
    html += "Paused";
  } else if (currentState == MENU_SELECTION && !isConfirmed) {
    html += "Input";
  } else if (currentState == MENU_SELECTION && isConfirmed) {
    html += "Confirm";
  } else if (currentState == Roasting) {
    html += "Roasting";
  } else if (currentState == Cooldown) {
    html += "Cooldown";
  } else {
    html += "Done";
  }
  html += "</p>";
  
  html += "<p style='margin-top: 5px;'><strong>Total Time:</strong> " + getFormattedTime(totalRemainingTimeSec);
  if ((currentState == Roasting || currentState == Cooldown) && !isPaused) {
    html += "<span class='spinner'></span>";
  }
  html += "</p>";

  // --- PT100 and Target Temperature Additions (Release 1.26) ---
  float tempVal = thermo.temperature(RNOMINAL, RREF);
  String tempStr = isnan(tempVal) ? "Error" : String(tempVal, 1) + " &deg;C";
  int activeTargetTemp = (currentState == Roasting && currentInstructionIdx >= 0) ? instructions[currentInstructionIdx].temperature : targetSyncTemp;

  html += "<p style='margin-top: 5px;'><strong>Current Temp (PT100):</strong> " + tempStr + "</p>";
  html += "<p style='margin-top: 5px;'><strong>Target Temp:</strong> " + String(activeTargetTemp) + " &deg;C</p>";
  
  if (currentState == Cooldown) {
    html += "<p class='highlight'>Roasting Done. Cooling Down</p>";
  } else if (currentState == Done) {
    html += "<p class='highlight'>Roasting Done. Cooldown 5 minutes</p>";
  }
  
  html += "<hr>";

  if (currentState == MENU_SELECTION) {
    if (hasSyntaxError) {
      html += "<div class='error'>" + syntaxErrorMsg + "</div>";
    }
    
    if (!isConfirmed) {
      html += "<form action='/validate_profile' method='POST'>";
      html += "<p>Enter Roasting Profile:</p>";
      html += "<textarea name='profileText' placeholder='Line 1: Bean Name&#10;Line 2: mm:ss aaaC bF'>" + rawProfileInput + "</textarea><br>";
      html += "<button type='submit'>Roast</button>";
      html += "</form>";
    } else {
      html += "<h3>Profile Verified</h3>";
      html += "<p><strong>Bean:</strong> " + beanName + "</p><ol style='text-align:left;'>";
      for (int i = 0; i < instructionCount; i++) {
        html += "<li>" + getFormattedTime(instructions[i].timeInSeconds) + " | " + String(instructions[i].temperature) + "C | Fan: " + String(instructions[i].fanSpeed) + "</li>";
      }
      html += "</ol>";
      html += "<button onclick='location.href=\"/run\"'>Confirm & Start</button>";
      html += "<button onclick='location.href=\"/edit_profile\"' style='background-color:#9e9e9e;'>Edit</button>";
    }
  } else {
    html += "<h3>Roasting Profile: " + beanName + "</h3>";
    html += "<div style='text-align:left; margin:15px; padding:10px; background:#f5f5f5;'>";
    for (int i = 0; i < instructionCount; i++) {
      String marker = "";
      String timeTrack = "";
      if (i == currentInstructionIdx && currentState == Roasting) {
        marker = "<span class='highlight'>* </span>";
        timeTrack = " -> <strong>Rem: " + getFormattedTime(currentRemainingTimeSec) + "</strong>";
      }
      
      html += "<div style='margin-bottom: 12px; padding: 4px; border-bottom: 1px dashed #ccc;'>";
      html += "<p style='margin:2px 0;'>" + marker + String(i+1) + ": " + getFormattedTime(instructions[i].timeInSeconds) + " @ " + String(instructions[i].temperature) + "C, Fan: " + String(instructions[i].fanSpeed) + timeTrack + "</p>";
      
      if (currentState == Roasting && i >= currentInstructionIdx) {
        html += "<form action='/adjust_step' method='GET' style='display:inline-block; margin-top:2px;'>";
        html += "  <input type='hidden' name='idx' value='" + String(i) + "'>";
        
        int stepMins = instructions[i].timeInSeconds / 60;
        int stepSecs = instructions[i].timeInSeconds % 60;
        char stepTimeStr[6];
        snprintf(stepTimeStr, sizeof(stepTimeStr), "%02d:%02d", stepMins, stepSecs);
        
        html += "  Time (mm:ss): <input type='text' name='time_mmss' value='" + String(stepTimeStr) + "' pattern='[0-5][0-9]:[0-5][0-9]' title='mm:ss format' style='width:65px; padding:3px; font-size:13px;'>";
        html += "  Temp: <input type='number' name='temp' value='" + String(instructions[i].temperature) + "' min='150' max='240' style='width:55px; padding:3px; font-size:13px;'>";
        html += "  Fan: <select name='fan' style='padding:2px; font-size:13px;'>";
        for (int f = 1; f <= 3; f++) {
          html += "    <option value='" + String(f) + "' " + String(instructions[i].fanSpeed == f ? "selected" : "") + ">" + String(f) + "</option>";
        }
        html += "  </select>";
        html += "  <button type='submit' class='update-btn'>Update</button>";
        html += "</form>";
      } else if (currentState == Roasting && i < currentInstructionIdx) {
        html += "  <span style='color:#757575; font-size:12px; font-style:italic;'>[Locked - Completed]</span>";
      }
      html += "</div>";
    }
    html += "</div>";
    
    html += "<button class='pause' onclick='location.href=\"/pause\"'>" + String(isPaused ? "Resume" : "Pause") + "</button><br>";
  }
  
  html += "<button id='restartInitBtn' class='reset' onclick='exposeRestartConfirmButton()'>Reset</button>";
  html += "<button id='restartConfirmBtn' class='reset reset-confirm' style='display:none;' onclick='location.href=\"/reset\"'>Confirm to Reset</button>";
  if (currentState == MENU_SELECTION) {
    html += "<hr>";
    html += "<div style='padding: 5px 0;'>";
    html += "  <button id='eraseInitBtn' class='erase-init' onclick='exposeConfirmButton()'>Erase Settings</button>";
    html += "  <button id='eraseConfirmBtn' class='erase-confirm' onclick='location.href=\"/erase_all\"'>Confirm to Erase Settings</button>";
    if (operationMode == MODE_STANDALONE) {
      html += "  <button id='switchWifiInitBtn' class='switch-init' onclick='exposeWifiConfirmButton()'>Switch to WIFI Settings</button>";
      html += "  <button id='switchWifiConfirmBtn' class='switch-confirm' onclick='location.href=\"/switch_mode?to=wifi\"'>Confirm to switch to WIFI</button>";
    } else if (operationMode == MODE_WIFI) {
      html += "  <button id='switchStandaloneInitBtn' class='switch-init' onclick='exposeStandaloneConfirmButton()'>Switch to Standalone</button>";
      html += "  <button id='switchStandaloneConfirmBtn' class='switch-confirm' onclick='location.href=\"/switch_mode?to=standalone\"'>Confirm to switch to Standalone</button>";
    }
    html += "</div>";
  }

  html += "</div></body></html>";
  return html;
}

bool isAuthorizedClient(AsyncWebServerRequest *request) {
  IPAddress clientIP = request->client()->remoteIP();
  if (!clientLocked) {
    allowedClientIP = clientIP;
    clientLocked = true;
    Serial.printf("[%s] [Security Guard] First client connected! Locking session to IP: %s\n", getFormattedTime(totalRemainingTimeSec).c_str(), allowedClientIP.toString().c_str());
    return true;
  }
  bool allowed = (clientIP == allowedClientIP);
  if (!allowed) {
    Serial.printf("[%s] [Security Guard] Blocked request from alternate IP: %s (Locked to: %s)\n", getFormattedTime(totalRemainingTimeSec).c_str(), clientIP.toString().c_str(), allowedClientIP.toString().c_str());
  }
  return allowed;
}

// --- OLED Rendering Utility ---
void updateOledDisplay() {
  float pt100Temp;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println("Auto Roaster " + RELEASE_VERSION);
  display.println("---------------------");

  if (isConnectingNetwork) {
    display.println("Connecting...");
    display.println("---------------------");
    display.display();
    return;
  }

// Check for kill switch confirmation screen first
  if (currentState == Roasting && awaitingCoolDownConfirm) {
    display.setTextSize(2);
    display.setCursor(0, 24);
    display.println("Cool Down?");
    display.display();
    return;
  }

  if (currentState == Roasting) {
    display.print("Total O/S  : "); 
    display.println(getFormattedTime(totalRemainingTimeSec));
    display.print( ((currentInstructionIdx+1) > 9) ? "Step " : "Step  ");
    display.print(currentInstructionIdx + 1);
    display.print("/");
    display.print(instructionCount);
    display.print((instructionCount > 9) ? " : " : "  : ");
    display.println(getFormattedTime(currentRemainingTimeSec));

    int activeFanSpeed = (currentInstructionIdx >= 0) ? instructions[currentInstructionIdx].fanSpeed : targetSyncFan;
    display.print("Fan   "); 
    display.print(activeFanSpeed);
    int targetTemp = (currentInstructionIdx >= 0) ? instructions[currentInstructionIdx].temperature : targetSyncTemp; 
    display.print("    : "); 
    display.print(targetTemp); 
    display.println(" C");
    
    pt100Temp = thermo.temperature(RNOMINAL, RREF);
    if (pt100Temp > -99) { 
      display.print("Stove Temp : ");
      if (isnan(pt100Temp)) {
        display.println("Error");
      } else {
        display.print(pt100Temp, 1); 
        display.println(" C");
      }
    }
  }
  else {
    if (currentState == WIFI_CONFIG_AP || operationMode == MODE_STANDALONE) {
      display.print("AP SSID: "); display.println(apSsid);
      display.print("AP IP  : "); display.println(WiFi.softAPIP().toString());
    } else {
      display.print("WiFi   : "); display.println(clientSsid.length() > 0 ? clientSsid : "Connecting...");
      display.print("STA IP : "); display.println(WiFi.localIP().toString());
    }

    display.print("Status : ");
    if (isPaused) {
      display.println("Paused");
    } else if (currentState == MENU_SELECTION) {
      display.println(isConfirmed ? "Confirmed" : "Input Mode");
    } else if (currentState == Cooldown) {
      display.println("Cooldown");
    } else if (currentState == Done) {
      display.println("Done");
    }

    display.print("Total  : "); 
    display.println(getFormattedTime(totalRemainingTimeSec));

    display.print("Target : "); 
    display.print(targetSyncTemp); 
    display.println(" C");
    pt100Temp = thermo.temperature(RNOMINAL, RREF);
    if (pt100Temp > -99) { 
      display.print("Current: ");
      if (isnan(pt100Temp)) {
        display.println("Error");
      } else {
        display.print(pt100Temp, 1); 
        display.println(" C");
      }
    }
  
  }
  display.display();
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  Serial.begin(115200);
  delay(200);
  if (DebugLevel >= 1) {
    for (int i = 0; i < 10; i++) {
      Serial.println("==================================================");
    }
    Serial.println("[System Initializing] ESP32 Coffee Roaster Control Stack Ready.");
    Serial.printf("[System Configuration] Execution Version: %s\n", RELEASE_VERSION.c_str());
  }

  // Initialize Custom I2C and OLED Display (Release 1.26)
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("[Hardware Alert] SSD1306 OLED initialization failed!"));
  } else {
    display.clearDisplay();
    display.display();
  }

  // Initialize Kill Switch Pin and Interrupt (Release 1.33)
  pinMode(KillSwitchPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KillSwitchPin), handleKillSwitch, FALLING);

  // Initialize MAX31865 PT100 Sensor 
  thermo.begin(MAX31865_3WIRE);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  ServoTemperature.setPeriodHertz(50);
  ServoTemperature.attach(TemperaturePin, 500, 2400);
  
  ServoTime.setPeriodHertz(50);
  ServoTime.attach(TimePin, 500, 2400);
  
  ServoOnOff.setPeriodHertz(50);
  ServoOnOff.attach(OnOffPin, 500, 2400);
  
  ServoFan.setPeriodHertz(50);
  ServoFan.attach(FanPin, 500, 2400);

  ServoTemperature.write(90);
  ServoTime.write(90);
  ServoOnOff.write(0);
  ServoFan.write(0);

  EEPROM.begin(EEPROM_SIZE); 
  loadWifiFromEEPROM();
  loadProfileFromEEPROM();
  
  OpModeOption storedMode = loadOpModeFromEEPROM();
  computeDynamicAPProperties();

  if (storedMode == MODE_STANDALONE) {
    operationMode = MODE_STANDALONE;
    executeStandaloneAPProcess();
    currentState = MENU_SELECTION;
  } else if (storedMode == MODE_WIFI) {
    operationMode = MODE_WIFI;
    if (executeWifiConnectionProcess()) {
      currentState = MENU_SELECTION;
    }
  } else {
    currentState = WIFI_CONFIG_AP;
    executeStandaloneAPProcess();
  }

  // Draw initial OLED State
  updateOledDisplay();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if (currentState == WIFI_CONFIG_AP) {
      request->send(200, "text/html", generateWifiSetupHtml());
    } else {
      if (!isAuthorizedClient(request)) {
        request->send(403, "text/plain", "403 Access Denied: Session IP Locked.");
        return;
      }
      request->send(200, "text/html", generateHtml());
    }
  });

  server.on("/adjust_step", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!isAuthorizedClient(request)) return;
    if (currentState == Roasting && request->hasParam("idx")) {
      int idx = request->getParam("idx")->value().toInt();
      if (idx >= currentInstructionIdx && idx < instructionCount) {
        
        bool timeChanged = false;
        bool tempChanged = false;
        bool fanChanged = false;

        int oldTemp = instructions[idx].temperature;
        int oldFan  = instructions[idx].fanSpeed;
        unsigned long oldTime = instructions[idx].timeInSeconds;

        if (request->hasParam("time_mmss")) {
          String mmss = request->getParam("time_mmss")->value();
          int colon = mmss.indexOf(':');
          if (colon != -1) {
            int m = mmss.substring(0, colon).toInt();
            int s = mmss.substring(colon + 1).toInt();
            unsigned long newTime = (m * 60) + s;
            if (newTime != oldTime) {
              instructions[idx].timeInSeconds = newTime;
              timeChanged = true;
            }
          }
        }
        if (request->hasParam("temp")) {
          int newTemp = request->getParam("temp")->value().toInt();
          if (newTemp != oldTemp) {
            instructions[idx].temperature = newTemp;
            tempChanged = true;
          }
        }
        if (request->hasParam("fan")) {
          int newFan = request->getParam("fan")->value().toInt();
          if (newFan != oldFan) {
            instructions[idx].fanSpeed = newFan;
            fanChanged = true;
          }
        }
        
        if (idx == currentInstructionIdx) {
          if (timeChanged) {
            unsigned long elapsedSecs = (millis() - stepTimer) / 1000UL;
            if (instructions[idx].timeInSeconds > elapsedSecs) {
              currentRemainingTimeSec = instructions[idx].timeInSeconds - elapsedSecs;
              stepDuration = instructions[idx].timeInSeconds * 1000UL;
            } else {
              currentRemainingTimeSec = 0;
              stepDuration = elapsedSecs * 1000UL; 
            }
            syncTimeFlag = true;
          }
          if (tempChanged) {
            targetSyncTemp = instructions[idx].temperature;
            syncTempFlag = true;
          }
          if (fanChanged) {
            targetSyncFan = instructions[idx].fanSpeed;
            syncFanFlag = true;
          }
          
          if (timeChanged || tempChanged || fanChanged) {
            triggerHardwareSync = true;
          }
        }
        recalculateDynamicRemainingTime();
      }
    }
    request->redirect("/");
  });

  server.on("/validate_profile", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!isAuthorizedClient(request)) return;
    if (request->hasParam("profileText", true)) {
      rawProfileInput = request->getParam("profileText", true)->value();
      parseProfile(rawProfileInput);
      if (!hasSyntaxError) {
        isConfirmed = true;
        recalculateDynamicRemainingTime();
        saveProfileToEEPROM();
      } else {
        isConfirmed = false;
      }
    }
    request->redirect("/");
  });

  server.on("/edit_profile", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!isAuthorizedClient(request)) return;
    isConfirmed = false;
    request->redirect("/");
  });

  server.on("/run", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!isAuthorizedClient(request)) return;
    if (currentState == MENU_SELECTION && isConfirmed && !hasSyntaxError) {
      triggerRun = true;
    }
    request->redirect("/");
  });

  server.on("/pause", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!isAuthorizedClient(request)) return;
    triggerPause = true;
    request->redirect("/");
  });

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!isAuthorizedClient(request)) return;
    triggerReset = true;
    String rHtml = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='4;url=/'></head>";
    rHtml += "<body style='font-family:Arial; text-align:center; padding-top:100px; background:#faf8f5; color:#3e2723;'>";
    rHtml += "<h3>System Resetting...</h3><p>Automatically reconnecting to home page in a few seconds.</p></body></html>";
    request->send(200, "text/html", rHtml);
  });

  server.on("/erase_all", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!isAuthorizedClient(request)) return;
    if (currentState != MENU_SELECTION) {
      request->send(403, "text/plain", "Denied: Active program cycle.");
      return;
    }
    triggerEraseAll = true;
    String rHtml = "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='5;url=/'></head>";
    rHtml += "<body style='font-family:Arial; text-align:center; padding-top:100px; background:#faf8f5; color:#3e2723;'>";
    rHtml += "<h3>Settings Erased.</h3><p>Device rebooting parameter structures. Reconnecting shortly...</p></body></html>";
    request->send(200, "text/html", rHtml);
  });

  server.on("/switch_mode", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!isAuthorizedClient(request)) return;
    if (currentState != MENU_SELECTION) {
      request->send(403, "text/plain", "Denied: Processing routine execution.");
      return;
    }
    if (request->hasParam("to")) {
      String target = request->getParam("to")->value();
      if (target == "wifi") {
        targetSwitchMode = MODE_WIFI;
        triggerSwitchMode = true;
        request->send(200, "text/plain", "Switching infrastructure to Client Station mode...");
        return;
      } else if (target == "standalone") {
        targetSwitchMode = MODE_STANDALONE;
        triggerSwitchMode = true;
        request->send(200, "text/plain", "Switching infrastructure to Standalone Access Point...");
        return;
      }
    }
    request->redirect("/");
  });

  server.on("/save_config", HTTP_POST, [](AsyncWebServerRequest *request){
    if (currentState == WIFI_CONFIG_AP) {
      int modeVal = MODE_WIFI;
      if (request->hasParam("opmode", true)) modeVal = request->getParam("opmode", true)->value().toInt();
      pendingOpMode = (OpModeOption)modeVal;
      if (request->hasParam("ssid", true)) pendingSsid = request->getParam("ssid", true)->value();
      if (request->hasParam("password", true)) pendingPassword = request->getParam("password", true)->value();
      triggerSaveConfig = true;
      request->send(200, "text/plain", "Credentials received. System rebooting parameters...");
    } else {
      request->redirect("/");
    }
  });

  server.begin();
}

void loop() {
  if (triggerReset) { delay(1000); ESP.restart(); }
  if (triggerEraseAll) { eraseAllEEPROM(); delay(1000); ESP.restart(); }
  if (triggerSwitchMode) { saveOpModeToEEPROM(targetSwitchMode); delay(1000); ESP.restart(); }
  if (triggerSaveConfig) {
    saveOpModeToEEPROM(pendingOpMode);
    saveWifiToEEPROM(pendingSsid, pendingPassword);
    delay(1000);
    ESP.restart();
  }

  // Actuates physical hardware servos safely on the main loop task thread, preventing Task WDT starvation
  if (triggerHardwareSync) {
    triggerHardwareSync = false;
    if (syncTimeFlag) {
      syncTimeFlag = false;
      setMaximumRoastingTime();
    }
    if (syncTempFlag) {
      syncTempFlag = false;
      adjustTemperature(targetSyncTemp);
    }
    if (syncFanFlag) {
      syncFanFlag = false;
      adjustFanSpeed(targetSyncFan);
    }
  }

  if (triggerPause) {
    triggerPause = false;
    if (currentState == Roasting || currentState == Cooldown) {
      isPaused = !isPaused;
      if (DebugLevel >= 1) {
        Serial.printf("[%s] [State] Program Execution %s.\n", getFormattedTime(totalRemainingTimeSec).c_str(), isPaused ? "Paused" : "Resumed");
      }
    }
  }

  if (triggerRun) {
    triggerRun = false;
    if (currentState == MENU_SELECTION && instructionCount > 0) {
      currentState = Roasting;
      currentInstructionIdx = 0;
      stepTimer = millis();
      
      unsigned long rawSecs = instructions[currentInstructionIdx].timeInSeconds;
      stepDuration = rawSecs * 1000UL; 
      currentRemainingTimeSec = rawSecs;
      
      if (DebugLevel >= 1) {
        Serial.printf("[%s] [Sequence] Starting Roasting Cycle. Pressing On/Off switch once.\n", getFormattedTime(totalRemainingTimeSec).c_str());
      }
      executeServoPress(ServoOnOff,OnOffPressDegree);
      setMaximumRoastingTime();
      adjustTemperature(instructions[currentInstructionIdx].temperature);
      adjustFanSpeed(instructions[currentInstructionIdx].fanSpeed);
    }
  }

// --- Kill Switch Intercept Engine (Release 1.33) ---
  if (killSwitchPressed) {
    killSwitchPressed = false; // Clear ISR flag
    static unsigned long lastKillSwitchPress = 0;
    
    if (millis() - lastKillSwitchPress > 250) { // 250ms software debounce filtering
      lastKillSwitchPress = millis();
      
      if (currentState == Roasting) {
        if (!awaitingCoolDownConfirm) {
          awaitingCoolDownConfirm = true;
          killSwitchPromptTime = millis(); // Start the 5-second window
        } else {
          awaitingCoolDownConfirm = false;
          if (DebugLevel >= 1) {
            Serial.printf("[%s] [Kill Switch] Confirmed! Bypassing remaining profile steps straight to Cooldown.\n", getFormattedTime(totalRemainingTimeSec).c_str());
          }
          
          // Physically click to Cool Down mode and initialize default safety overrides
          executeServoPress(ServoOnOff, OnOffPressDegree); 
          adjustTemperature(150); 
          setMinimumRoastingTime(); 
          currentState = Cooldown;
          
          // Initialize Cooldown timer parameters immediately
          stepTimer = millis();
          stepDuration = 5 * 60 * 1000UL;
          currentRemainingTimeSec = 5 * 60;
          totalRemainingTimeSec = currentRemainingTimeSec;
        }
        updateOledDisplay();
      }
    }
  }

// Monitor for the 5-second timeout if confirmation is pending
  if (currentState == Roasting && awaitingCoolDownConfirm) {
    if (millis() - killSwitchPromptTime >= 5000UL) { 
      awaitingCoolDownConfirm = false;
      if (DebugLevel >= 1) {
        Serial.printf("[%s] [Kill Switch] Confirmation timed out after 5 seconds. Returning to Roasting.\n", getFormattedTime(totalRemainingTimeSec).c_str());
      }
      updateOledDisplay(); // Restore regular roasting display data
    }
  }
  // Auto-reset confirmation monitor if state transitions out of Roasting asynchronously
  if (currentState != Roasting) {
    awaitingCoolDownConfirm = false;
  }
  unsigned long currentMillis = millis();
  if (!isPaused && (currentState == Roasting || currentState == Cooldown)) {

    if (currentState == Roasting) {
      unsigned long elapsed = currentMillis - stepTimer;
      unsigned long targetScale = 1000UL; 
      
      long calculatedRem = (long)instructions[currentInstructionIdx].timeInSeconds - (elapsed / targetScale);
      currentRemainingTimeSec = (calculatedRem < 0) ? 0 : calculatedRem;
      
      recalculateDynamicRemainingTime();

      if (elapsed >= stepDuration) {
        currentInstructionIdx++;
        if (currentInstructionIdx < instructionCount) {
          stepTimer = millis();
          unsigned long nextSecs = instructions[currentInstructionIdx].timeInSeconds;
          stepDuration = nextSecs * 1000UL;
          currentRemainingTimeSec = nextSecs;
          adjustTemperature(instructions[currentInstructionIdx].temperature);
          adjustFanSpeed(instructions[currentInstructionIdx].fanSpeed);
        } else {
          if (DebugLevel >= 1) {
            Serial.printf("[%s] [Sequence] All instruction phases finished. Triggering Cool Down.\n", getFormattedTime(totalRemainingTimeSec).c_str());
          }

          executeServoPress(ServoOnOff,OnOffPressDegree); // press OnOff so roaster goes to Cool Down mode.
          adjustTemperature(150); // safety measure to set Roast Temperature to lowest 150 C in case the OnOff button push fails.
          setMinimumRoastingTime(); // safety measure to set roast time to min 1 minute in case the OnOff button push fails.
          currentState = Cooldown;
          
          rawProfileInput = beanName + "\n";
          for (int i = 0; i < instructionCount; i++) {
            int m = instructions[i].timeInSeconds / 60;
            int s = instructions[i].timeInSeconds % 60;
            char stepBuf[32];
            sprintf(stepBuf, "%02d:%02d %dC %dF\n", m, s, instructions[i].temperature, instructions[i].fanSpeed);
            rawProfileInput += String(stepBuf);
          }
          saveProfileToEEPROM();

          stepTimer = millis();
          stepDuration = 5 * 60 * 1000UL;
          currentRemainingTimeSec = 5 * 60;
          totalRemainingTimeSec = currentRemainingTimeSec;
        }
      }
    } else if (currentState == Cooldown) {
      unsigned long elapsed = currentMillis - stepTimer;
      unsigned long targetScale = 1000UL;
      
      long calculatedRem = (5 * 60) - (elapsed / targetScale);
      currentRemainingTimeSec = (calculatedRem < 0) ? 0 : calculatedRem;
      totalRemainingTimeSec = currentRemainingTimeSec;
      if (elapsed >= stepDuration) {
        currentState = Done;
        totalRemainingTimeSec = 0;
        if (DebugLevel >= 1) {
          Serial.printf("[%s] [Sequence] Cooldown sequence finished. Returning machine control back to user.\n", getFormattedTime(totalRemainingTimeSec).c_str());
        }
      }
    }
  }

  // --- OLED Refresh Daemon (Refresh every 1000 ms) ---
  if (currentMillis - lastOledUpdate >= 1000) {
    lastOledUpdate = currentMillis;
    updateOledDisplay();
  }

  // --- Logging Engine ---
  if (DebugLevel >= 1) {
    bool stateChanged = (currentState != lastLoggedState);
    bool stepChanged  = (currentInstructionIdx != lastInstructionLogged);
    
    if (stateChanged || stepChanged) {
      Serial.printf("[%s] [System Log] State: ", getFormattedTime(totalRemainingTimeSec).c_str());
      if (currentState == MENU_SELECTION) Serial.print("MENU_SELECTION");
      else if (currentState == Roasting) Serial.print("Roasting");
      else if (currentState == Cooldown) Serial.print("Cooldown");
      else if (currentState == Done) Serial.print("Done");
      
      if (currentState == Roasting) {
        Serial.printf(" | Step Index: %d/%d", currentInstructionIdx + 1, instructionCount);
        Serial.printf(" | Target Instruction Time: %lu sec", instructions[currentInstructionIdx].timeInSeconds);
        Serial.printf(" | Target Temp: %dC | Target Fan: %d", instructions[currentInstructionIdx].temperature, instructions[currentInstructionIdx].fanSpeed);
      }
      Serial.println();
      
      lastLoggedState = currentState;
      lastInstructionLogged = currentInstructionIdx;
    }
  }
  
  // yield control to allow background tasks to feed the generic watchdog
  yield();
}