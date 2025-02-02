#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <DNSServer.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <Espalexa.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <NTPClient.h>
#include <WiFiUDP.h>

// Add debug flag near the top with other constants
const bool DEBUG_OUTPUT = true;  // Set to false in production

#define LED_PIN     D4     // Using D4 (GPIO2) on Wemos D1 Mini
#define NUM_LEDS    16     // Number of LEDs in the ring
#define EEPROM_SIZE 52     // 32 bytes for name, 1 for name length, 1 for power, 1 for brightness, 3 for RGB, 1 for checksum, 2 for battery interval, 1 for battery threshold, 1 for timezone, 9 for schedule
#define BUTTON_PIN  D3
#define LONG_PRESS_TIME 2000  // 2 second for long press
#define DEBOUNCE_TIME 50      // 50ms debounce
#define BATTERY_PIN A0
#define EEPROM_NAME_ADDR 0
#define EEPROM_NAME_LEN_ADDR 32
#define EEPROM_SETTINGS_ADDR 33

// Add these defines for clarity
#define EEPROM_POWER_ADDR      (EEPROM_SETTINGS_ADDR + 0)  // 33
#define EEPROM_BRIGHTNESS_ADDR (EEPROM_SETTINGS_ADDR + 1)  // 34
#define EEPROM_RED_ADDR        (EEPROM_SETTINGS_ADDR + 2)  // 35
#define EEPROM_GREEN_ADDR      (EEPROM_SETTINGS_ADDR + 3)  // 36
#define EEPROM_BLUE_ADDR       (EEPROM_SETTINGS_ADDR + 4)  // 37
#define EEPROM_CHECKSUM_ADDR   (EEPROM_BLUE_ADDR + 1)  // Add this define

// Change these from #define to const variables
const uint16_t DEFAULT_BATTERY_CHECK_INTERVAL = 10;  // 10 seconds
const float DEFAULT_BATTERY_LOW_THRESHOLD = 3.3;     // 3.3V

// Add EEPROM addresses for new settings
#define EEPROM_BATTERY_INTERVAL_ADDR (EEPROM_CHECKSUM_ADDR + 1)    // 39
#define EEPROM_BATTERY_THRESHOLD_ADDR (EEPROM_BATTERY_INTERVAL_ADDR + 2)  // 41

// Add these defines for EEPROM storage
#define EEPROM_TIMEZONE_ADDR (EEPROM_BATTERY_THRESHOLD_ADDR + 1)  // 42
#define EEPROM_SCHEDULE_ADDR (EEPROM_TIMEZONE_ADDR + 1)          // 43
#define MAX_SCHEDULES 1  // Start with 1 schedule for simplicity

// Add global variables for battery settings
uint16_t batteryCheckInterval = DEFAULT_BATTERY_CHECK_INTERVAL;  // in seconds
float batteryLowThreshold = DEFAULT_BATTERY_LOW_THRESHOLD;

// Add these globals
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Add this global variable for NTP update interval
unsigned long lastNTPUpdate = 0;
const unsigned long NTP_UPDATE_INTERVAL = 60000;  // Update NTP every minute
unsigned long ntpRetryInterval = NTP_UPDATE_INTERVAL;
bool ntpSynced = false;

// Schedule structure
struct Schedule {
    bool enabled;
    uint8_t startHour;
    uint8_t startMinute;
    uint8_t endHour;
    uint8_t endMinute;
    uint8_t brightness;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} schedule;

int8_t timezone = 10;  // Default to UTC+10
bool scheduleActive = false;

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
ESP8266WebServer server(80);
WiFiManager wifiManager;
Espalexa espalexa;
const char* DEFAULT_DEVICE_NAME = "SMART LAMP";
const bool useBattery = true;

// Variables for light control
bool isPowered = true;
int brightness = 100;  // 0-100
int redValue = 255;    // 0-255
int greenValue = 255;  // 0-255
int blueValue = 255;   // 0-255
int currentPreset = -1;       // Current preset index (-1 means custom color)
unsigned long buttonPressTime = 0;
unsigned long lastDebounceTime = 0;
bool lastButtonState = HIGH;
bool buttonState = HIGH;
bool longPressHandled = false;  // To prevent multiple triggers during one long press

// Add these color presets near the top with other variables
struct ColorPreset {
    const char* name;
    byte r, g, b;
} colorPresets[] = {
    {"Warm White", 255, 120, 80},
    {"Cool White", 255, 255, 255},
    {"Night Light", 255, 120, 0},
    {"Red", 255, 0, 0},
    {"Green", 0, 255, 0},
    {"Blue", 0, 0, 255},
    {"Purple", 255, 0, 255},
    {"Orange", 255, 50, 0}
};

// Add these variables with other globals
unsigned long lastBatteryCheck = 0;
// Using 100K and 100K resistors for voltage divider
//             A0
// B+ -- 100K -^- 100K -- GND
// 2.1V max at ADC
const float VOLTAGE_DIVIDER_RATIO = 2; //(100.0 + 100.0) / 100.0;

char deviceName[32] = {0};  // For storing configured device name in EEPROM

const char* getDeviceName() {
    if (strlen(deviceName) == 0) {
        return DEFAULT_DEVICE_NAME;
    }
    return deviceName;
}

void saveDeviceName(const char* name) {
    uint8_t len = strlen(name);
    if (len > 31) len = 31;  // Correct: ensures name fits in 32 bytes
    
    // Correct: writes to addresses 0-31
    for (int i = 0; i < len; i++) {
        EEPROM.write(EEPROM_NAME_ADDR + i, name[i]);
    }
    // Correct: writes length at address 32
    EEPROM.write(EEPROM_NAME_LEN_ADDR, len);
    EEPROM.commit();
    
    strcpy(deviceName, name);
}

void saveSettings() {
    // Save all settings first
    EEPROM.write(EEPROM_POWER_ADDR, isPowered ? 1 : 0);
    EEPROM.write(EEPROM_BRIGHTNESS_ADDR, brightness);
    EEPROM.write(EEPROM_RED_ADDR, redValue);
    EEPROM.write(EEPROM_GREEN_ADDR, greenValue);
    EEPROM.write(EEPROM_BLUE_ADDR, blueValue);
    
    // Save battery settings
    uint16_t interval = batteryCheckInterval;
    EEPROM.write(EEPROM_BATTERY_INTERVAL_ADDR, interval & 0xFF);
    EEPROM.write(EEPROM_BATTERY_INTERVAL_ADDR + 1, (interval >> 8) & 0xFF);
    
    uint8_t threshold = (uint8_t)(batteryLowThreshold * 10);
    EEPROM.write(EEPROM_BATTERY_THRESHOLD_ADDR, threshold);
    
    // Add timezone save
    EEPROM.write(EEPROM_TIMEZONE_ADDR, timezone);
    
    // Calculate new comprehensive checksum
    byte checksum = 0;
    checksum += (isPowered ? 1 : 0);
    checksum += brightness;
    checksum += redValue;
    checksum += greenValue;
    checksum += blueValue;
    checksum += (interval & 0xFF);
    checksum += ((interval >> 8) & 0xFF);
    checksum += threshold;
    checksum += timezone;
    checksum += schedule.enabled ? 1 : 0;
    checksum += schedule.startHour;
    checksum += schedule.startMinute;
    checksum += schedule.endHour;
    checksum += schedule.endMinute;
    checksum += schedule.brightness;
    checksum += schedule.red;
    checksum += schedule.green;
    checksum += schedule.blue;
    
    EEPROM.write(EEPROM_CHECKSUM_ADDR, checksum);
    EEPROM.commit();
}

void loadSettings() {
    // Load device name (not included in checksum)
    uint8_t nameLen = EEPROM.read(EEPROM_NAME_LEN_ADDR);
    if (nameLen > 0 && nameLen < 32) {
        for (int i = 0; i < nameLen; i++) {
            deviceName[i] = EEPROM.read(EEPROM_NAME_ADDR + i);
        }
        deviceName[nameLen] = 0;  // Null terminate
    } else {
        deviceName[0] = 0;  // Empty string if invalid length
    }

    // Load all settings
    isPowered = EEPROM.read(EEPROM_POWER_ADDR) == 1;
    brightness = EEPROM.read(EEPROM_BRIGHTNESS_ADDR);
    redValue = EEPROM.read(EEPROM_RED_ADDR);
    greenValue = EEPROM.read(EEPROM_GREEN_ADDR);
    blueValue = EEPROM.read(EEPROM_BLUE_ADDR);
    
    uint16_t interval = EEPROM.read(EEPROM_BATTERY_INTERVAL_ADDR) |
                       (EEPROM.read(EEPROM_BATTERY_INTERVAL_ADDR + 1) << 8);
    uint8_t threshold = EEPROM.read(EEPROM_BATTERY_THRESHOLD_ADDR);
    timezone = EEPROM.read(EEPROM_TIMEZONE_ADDR);  // Add timezone load
    
    // Load schedule settings
    schedule.enabled = EEPROM.read(EEPROM_SCHEDULE_ADDR + 0);
    schedule.startHour = EEPROM.read(EEPROM_SCHEDULE_ADDR + 1);
    schedule.startMinute = EEPROM.read(EEPROM_SCHEDULE_ADDR + 2);
    schedule.endHour = EEPROM.read(EEPROM_SCHEDULE_ADDR + 3);
    schedule.endMinute = EEPROM.read(EEPROM_SCHEDULE_ADDR + 4);
    schedule.brightness = EEPROM.read(EEPROM_SCHEDULE_ADDR + 5);
    schedule.red = EEPROM.read(EEPROM_SCHEDULE_ADDR + 6);
    schedule.green = EEPROM.read(EEPROM_SCHEDULE_ADDR + 7);
    schedule.blue = EEPROM.read(EEPROM_SCHEDULE_ADDR + 8);

    // Calculate checksum from loaded values
    byte calculatedChecksum = 0;
    calculatedChecksum += (isPowered ? 1 : 0);
    calculatedChecksum += brightness;
    calculatedChecksum += redValue;
    calculatedChecksum += greenValue;
    calculatedChecksum += blueValue;
    calculatedChecksum += (interval & 0xFF);
    calculatedChecksum += ((interval >> 8) & 0xFF);
    calculatedChecksum += threshold;
    calculatedChecksum += timezone;
    calculatedChecksum += schedule.enabled ? 1 : 0;
    calculatedChecksum += schedule.startHour;
    calculatedChecksum += schedule.startMinute;
    calculatedChecksum += schedule.endHour;
    calculatedChecksum += schedule.endMinute;
    calculatedChecksum += schedule.brightness;
    calculatedChecksum += schedule.red;
    calculatedChecksum += schedule.green;
    calculatedChecksum += schedule.blue;

    byte storedChecksum = EEPROM.read(EEPROM_CHECKSUM_ADDR);

    // Add debug prints
    if (DEBUG_OUTPUT) {
        Serial.println("Checksum comparison:");
        Serial.printf("Stored: %d, Calculated: %d\n", storedChecksum, calculatedChecksum);
        Serial.printf("Settings: power=%d, bright=%d, RGB=%d,%d,%d\n", 
                     isPowered, brightness, redValue, greenValue, blueValue);
        Serial.printf("Battery: interval=%d, threshold=%d\n", interval, threshold);
        Serial.printf("Schedule: enabled=%d, start=%02d:%02d, end=%02d:%02d\n",
                     schedule.enabled, schedule.startHour, schedule.startMinute,
                     schedule.endHour, schedule.endMinute);
    }

    // If checksum fails or values are invalid, reset to defaults
    if (storedChecksum != calculatedChecksum || 
        brightness > 100 || 
        schedule.brightness > 100 ||
        schedule.startHour > 23 ||
        schedule.endHour > 23 ||
        schedule.startMinute > 59 ||
        schedule.endMinute > 59 ||
        interval > 3600 ||          // Add battery settings validation
        threshold < 20 ||           // 2.0V minimum
        threshold > 50) {           // 5.0V maximum
        
        if (DEBUG_OUTPUT) {
            Serial.println("EEPROM checksum failed or invalid values, resetting to defaults");
        }
        
        // Reset to defaults
        isPowered = true;
        brightness = 100;
        redValue = 255;
        greenValue = 255;
        blueValue = 255;
        batteryCheckInterval = DEFAULT_BATTERY_CHECK_INTERVAL;
        batteryLowThreshold = DEFAULT_BATTERY_LOW_THRESHOLD;
        timezone = 10;  // Default timezone
        deviceName[0] = 0;  // Reset device name
        
        // Reset schedule
        schedule.enabled = false;
        schedule.startHour = 0;
        schedule.startMinute = 0;
        schedule.endHour = 0;
        schedule.endMinute = 0;
        schedule.brightness = 100;
        schedule.red = 255;
        schedule.green = 255;
        schedule.blue = 255;

        // Save defaults back to EEPROM
        saveDeviceName("");  // Clear device name
        saveSettings();      // Save default settings
    } else {
        // Apply loaded values with range validation
        batteryCheckInterval = (interval > 0 && interval <= 3600) ? interval : DEFAULT_BATTERY_CHECK_INTERVAL;
        batteryLowThreshold = (threshold >= 20 && threshold <= 50) ? (threshold / 10.0) : DEFAULT_BATTERY_LOW_THRESHOLD;
    }
}

void updateLEDs() {
    if (!isPowered) {
        for(int i = 0; i < NUM_LEDS; i++) {
            pixels.setPixelColor(i, pixels.Color(0, 0, 0));
        }
    } else {
        float brightnessScale = brightness / 100.0;
        byte r = (byte)(redValue * brightnessScale);
        byte g = (byte)(greenValue * brightnessScale);
        byte b = (byte)(blueValue * brightnessScale);
        if (DEBUG_OUTPUT) {
            Serial.printf("Update LED -- STATE: %d, Brightness: %d, RGB: %d %d %d\n", 
                         isPowered, brightness, redValue, greenValue, blueValue);
        }

        for(int i = 0; i < NUM_LEDS; i++) {
            pixels.setPixelColor(i, pixels.Color(r, g, b));
        }
    }
    pixels.show();
    if (DEBUG_OUTPUT) Serial.println("updated");
    saveSettings();
}

void applyPreset(int index) {
    if (!isPowered) {
        return;
    }

    if (index >= 0 && index < 8) {  // 8 is number of presets
        currentPreset = index;
        redValue = colorPresets[index].r;
        greenValue = colorPresets[index].g;
        blueValue = colorPresets[index].b;
        updateLEDs();
    }
}

void handleAlexa(EspalexaDevice* d) {
    if(d == nullptr) return;

    isPowered = d->getValue() > 0;

    if(isPowered){
        brightness = map(d->getValue(), 0, 255, 0, 100);
        redValue = d->getR();
        greenValue = d->getG();
        blueValue = d->getB();
    }

    if (DEBUG_OUTPUT) {
        Serial.printf("Alexa command STATE: %d, Brightness: %d, RGB: %d %d %d\n", isPowered, brightness, redValue, greenValue, blueValue);
    }
    updateLEDs();
}

float getBatteryVoltage() {
    int raw = analogRead(BATTERY_PIN);
    // Convert ADC value to voltage (ADC range 0-1023 maps to 0-3.3V)
    float voltage = (raw / 1023.0) * 3.3;
    // Account for voltage divider
    return voltage * VOLTAGE_DIVIDER_RATIO;
}

// Add this function to store current state and flash red
void flashLowBatteryWarning() {
    // Store current state
    bool wasOn = isPowered;
    byte oldColors[NUM_LEDS][3];
    for(int i = 0; i < NUM_LEDS; i++) {
        uint32_t color = pixels.getPixelColor(i);
        oldColors[i][0] = (color >> 16) & 0xFF;
        oldColors[i][1] = (color >> 8) & 0xFF;
        oldColors[i][2] = color & 0xFF;
    }
    
    // Flash red
    float brightnessScale = brightness / 100.0;
    for(int i = 0; i < NUM_LEDS; i++) {
        pixels.setPixelColor(i, pixels.Color(255 * brightnessScale, 0, 0));
    }

    pixels.show();
    delay(200);
    
    // Restore previous state
    for(int i = 0; i < NUM_LEDS; i++) {
        pixels.setPixelColor(i, pixels.Color(oldColors[i][0], oldColors[i][1], oldColors[i][2]));
    }
    pixels.show();
}

void handleConfig() {
    if (server.method() == HTTP_POST) {
        bool settingsChanged = false;
        
        if (server.hasArg("name")) {
            String newName = server.arg("name");
            if (newName.length() > 0) {
                saveDeviceName(newName.c_str());
                settingsChanged = true;
            }
        }
        
        if (server.hasArg("batteryInterval")) {
            uint16_t newInterval = server.arg("batteryInterval").toInt();
            if (newInterval > 0 && newInterval <= 3600) {  // Max 1 hour
                batteryCheckInterval = newInterval;
                settingsChanged = true;
            }
        }
        
        if (server.hasArg("batteryThreshold")) {
            float newThreshold = server.arg("batteryThreshold").toFloat();
            if (newThreshold >= 2.0 && newThreshold <= 5.0) {
                batteryLowThreshold = newThreshold;
                settingsChanged = true;
            }
        }
        
        if (server.hasArg("timezone")) {
            int8_t newTimezone = server.arg("timezone").toInt();
            if (newTimezone >= -12 && newTimezone <= 14) {
                timezone = newTimezone;
                settingsChanged = true;
            }
        }
        
        if (settingsChanged) {
            saveSettings();
            String html = "<!DOCTYPE html><html><head>"
                         "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                         "<style>"
                         "body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }"
                         "</style>"
                         "</head><body>"
                         "<h3>Settings saved!</h3>"
                         "<p>Device will restart in <span id='countdown'>5</span> seconds...</p>"
                         "<script>"
                         "let count = 5;"
                         "const counter = setInterval(() => {"
                         "  count--;"
                         "  document.getElementById('countdown').textContent = count;"
                         "  if (count <= 0) clearInterval(counter);"
                         "}, 1000);"
                         "</script>"
                         "</body></html>";
            server.send(200, "text/html", html);
            delay(5000);  // Wait 5 seconds
            ESP.reset();  // Reset the device
        } else {
            server.send(400, "text/plain", "No valid settings changes");
        }
        return;
    }

    String html = "<!DOCTYPE html>"
                "<html>"
                "<head>"
                "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                "<title>Settings " + String(getDeviceName()) + "</title>"
                "<style>"
                "body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }"
                ".container { max-width: 400px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
                ".form-group { margin-bottom: 15px; }"
                "label { display: block; margin-bottom: 5px; }"
                "input[type='text'] { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; }"
                "button { background: #2196F3; color: white; border: none; padding: 10px 20px; border-radius: 4px; cursor: pointer; }"
                "button:hover { background: #1976D2; }"
                "input[type='number'] { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; }"
                ".nav-links { margin-bottom: 20px; }"
                ".nav-links a { color: #2196F3; text-decoration: none; margin-right: 15px; }"
                ".nav-links a:hover { text-decoration: underline; }"
                ".footer { text-align: center; margin-top: 30px; padding-top: 20px; "
                "border-top: 1px solid #ddd; color: #666; font-size: 0.9em; }"
                ".footer a { color: #2196F3; text-decoration: none; }"
                ".footer a:hover { text-decoration: underline; }"
                "</style>"
                "</head>"
                "<body>"
                "<div class='container'>"
                "<div class='nav-links'>"
                "<a href='/'>Control</a>"
                "<a href='/schedule'>Schedule</a>"
                "<a href='/config'>Settings</a>"
                "</div>"
                "<h1>Settings</h1>"
                "<form method='post'>"
                "<div class='form-group'>"
                "<label>Device Name:</label>"
                "<input type='text' name='name' value='" + String(getDeviceName()) + "' maxlength='31'>"
                "</div>"
                "<div class='form-group'>"
                "<label>Battery Check Interval (seconds):</label>"
                "<input type='number' name='batteryInterval' value='" + String(batteryCheckInterval) + "' min='1' max='3600'>"
                "</div>"
                "<div class='form-group'>"
                "<label>Battery Low Threshold (volts):</label>"
                "<input type='number' name='batteryThreshold' value='" + String(batteryLowThreshold, 1) + "' min='2.0' max='5.0' step='0.1'>"
                "</div>"
                "<div class='form-group'>"
                "<label>Timezone (UTC offset):</label>"
                "<select name='timezone'>";
    
    for (int i = -12; i <= 14; i++) {
        html += "<option value='" + String(i) + "'" + 
               (i == timezone ? " selected" : "") + 
               ">UTC" + (i >= 0 ? "+" : "") + String(i) + "</option>";
    }
    
    html += "</select>"
                "</div>"
                "<button type='submit'>Save Settings</button>"
                "</form>"
                "<div class='footer'>"
                "ESP LAMP by Eric Xu<br>"
                "<a href='https://github.com/utsxumiao/esp-lamp' target='_blank'>GitHub Repository</a>"
                "</div>"
                "</div>"
                "</body>"
                "</html>";

    server.send(200, "text/html", html);
}

// Add this helper function
String padZero(int num) {
    return (num < 10) ? "0" + String(num) : String(num);
}

void handleSchedule() {
    if (server.method() == HTTP_POST) {
        if (DEBUG_OUTPUT) {
            Serial.println("Schedule POST received:");
            Serial.printf("enabled: %s\n", server.arg("enabled").c_str());
            Serial.printf("startHour: %s\n", server.arg("startHour").c_str());
            Serial.printf("startMinute: %s\n", server.arg("startMinute").c_str());
            Serial.printf("endHour: %s\n", server.arg("endHour").c_str());
            Serial.printf("endMinute: %s\n", server.arg("endMinute").c_str());
            Serial.printf("brightness: %s\n", server.arg("brightness").c_str());
            Serial.printf("RGB: %s,%s,%s\n", 
                server.arg("red").c_str(),
                server.arg("green").c_str(),
                server.arg("blue").c_str());
        }

        schedule.enabled = server.hasArg("enabled");
        if (server.hasArg("startHour") && server.hasArg("startMinute")) {
            schedule.startHour = server.arg("startHour").toInt();
            schedule.startMinute = server.arg("startMinute").toInt();
        }
        if (server.hasArg("endHour") && server.hasArg("endMinute")) {
            schedule.endHour = server.arg("endHour").toInt();
            schedule.endMinute = server.arg("endMinute").toInt();
        }
        if (server.hasArg("brightness")) {
            schedule.brightness = server.arg("brightness").toInt();
        }
        if (server.hasArg("red") && server.hasArg("green") && server.hasArg("blue")) {
            schedule.red = server.arg("red").toInt();
            schedule.green = server.arg("green").toInt();
            schedule.blue = server.arg("blue").toInt();
        }

        saveSchedule();
        
        if (DEBUG_OUTPUT) {
            Serial.println("Schedule saved with values:");
            Serial.printf("enabled: %d\n", schedule.enabled);
            Serial.printf("time: %02d:%02d - %02d:%02d\n", 
                schedule.startHour, schedule.startMinute,
                schedule.endHour, schedule.endMinute);
            Serial.printf("brightness: %d\n", schedule.brightness);
            Serial.printf("RGB: %d,%d,%d\n", schedule.red, schedule.green, schedule.blue);
        }

        server.send(200, "text/plain", "Schedule saved successfully");
        return;
    }

    String html = "<!DOCTYPE html><html><head>"
                 "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                 "<title>Schedule - " + String(getDeviceName()) + "</title>"
                 "<style>"
                 "body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }"
                 ".container { max-width: 400px; margin: 0 auto; background: white; padding: 20px; "
                 "border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
                 ".form-group { margin-bottom: 15px; display: flex; align-items: center; gap: 10px; }"
                 ".form-group label { margin: 0; min-width: 80px; }"
                 ".control-wrap { flex: 1; }"
                 ".time-group { display: flex; gap: 20px; margin-bottom: 30px; }"
                 ".time-input { flex: 1; }"
                 "input[type='time'] { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; }"
                 "input[type='range'] { width: 100%; height: 25px; -webkit-appearance: none; border-radius: 12px; outline: none; }"
                 "input[type='range']::-webkit-slider-thumb { -webkit-appearance: none; width: 25px; height: 25px; "
                 "background: white; border: 2px solid #666; border-radius: 50%; cursor: pointer; }"
                 ".value-display { min-width: 45px; text-align: right; }"
                 ".nav-links { margin-bottom: 20px; }"
                 ".nav-links a { color: #2196F3; text-decoration: none; margin-right: 15px; }"
                 ".nav-links a:hover { text-decoration: underline; }"
                 ".current-time { font-size: 1.2em; text-align: center; margin: 20px 0; color: #666; }"
                 ".color-preview { width: 100%; height: 50px; margin: 10px 0; border-radius: 4px; }"
                 "#red { background: linear-gradient(to right, #FFFFFF, #FF0000); }"
                 "#green { background: linear-gradient(to right, #FFFFFF, #00FF00); }"
                 "#blue { background: linear-gradient(to right, #FFFFFF, #0000FF); }"
                 "#brightness { background: #ddd; }"
                 "button { background: #2196F3; color: white; border: none; padding: 10px 20px; "
                 "border-radius: 4px; cursor: pointer; width: 100%; margin-top: 20px; }"
                 "button:hover { background: #1976D2; }"
                 ".footer { text-align: center; margin-top: 30px; padding-top: 20px; "
                 "border-top: 1px solid #ddd; color: #666; font-size: 0.9em; }"
                 ".footer a { color: #2196F3; text-decoration: none; }"
                 ".footer a:hover { text-decoration: underline; }"
                 "</style></head><body>"
                 "<div class='container'>"
                 "<div class='nav-links'>"
                 "<a href='/'>Control</a>"
                 "<a href='/schedule'>Schedule</a>"
                 "<a href='/config'>Settings</a>"
                 "</div>"
                 "<h1>Schedule</h1>"
                 "<div class='current-time'>"
                 "Current Time: <span id='currentTime'>--:--:--</span>"
                 "</div>"
                 "<form method='post' id='scheduleForm'>"
                 "<div class='form-group'>"
                 "<label><input type='checkbox' name='enabled'" + 
                 (schedule.enabled ? " checked" : "") + "> Enable Schedule</label>"
                 "</div>"
                 "<div class='time-group'>"
                 "<div class='time-input'>"
                 "<label>Start Time:</label>"
                 "<input type='time' name='startTime' value='" + 
                 padZero(schedule.startHour) + ":" + padZero(schedule.startMinute) + "'>"
                 "</div>"
                 "<div class='time-input'>"
                 "<label>End Time:</label>"
                 "<input type='time' name='endTime' value='" + 
                 padZero(schedule.endHour) + ":" + padZero(schedule.endMinute) + "'>"
                 "</div>"
                 "</div>"
                 "<div class='form-group'>"
                 "<label>Red</label>"
                 "<div class='control-wrap'><input type='range' id='red' name='red' min='0' max='255' value='" + 
                 String(schedule.red) + "' oninput='updateColor()'></div>"
                 "<div class='value-display'><span id='redValue'>" + String(schedule.red) + "</span></div>"
                 "</div>"
                 "<div class='form-group'>"
                 "<label>Green</label>"
                 "<div class='control-wrap'><input type='range' id='green' name='green' min='0' max='255' value='" + 
                 String(schedule.green) + "' oninput='updateColor()'></div>"
                 "<div class='value-display'><span id='greenValue'>" + String(schedule.green) + "</span></div>"
                 "</div>"
                 "<div class='form-group'>"
                 "<label>Blue</label>"
                 "<div class='control-wrap'><input type='range' id='blue' name='blue' min='0' max='255' value='" + 
                 String(schedule.blue) + "' oninput='updateColor()'></div>"
                 "<div class='value-display'><span id='blueValue'>" + String(schedule.blue) + "</span></div>"
                 "</div>"
                 "<div class='form-group'>"
                 "<label>Brightness</label>"
                 "<div class='control-wrap'><input type='range' id='brightness' name='brightness' min='10' max='100' step='10' value='" + 
                 String(schedule.brightness) + "' oninput='updateBrightness(this.value)'></div>"
                 "<div class='value-display'><span id='brightnessValue'>" + String(schedule.brightness) + "%</span></div>"
                 "</div>"
                 "<div class='color-preview' id='colorPreview'></div>"
                 "<button type='submit'>Save Schedule</button>"
                 "</form></div>"
                 "<script>"
                 "function updateBrightness(val) {"
                 "  document.getElementById('brightnessValue').textContent = val + '%';"
                 "}"
                 "function updateColor() {"
                 "  const r = document.getElementById('red').value;"
                 "  const g = document.getElementById('green').value;"
                 "  const b = document.getElementById('blue').value;"
                 "  document.getElementById('colorPreview').style.backgroundColor = "
                 "    `rgb(${r},${g},${b})`;"
                 "  document.getElementById('redValue').textContent = r;"
                 "  document.getElementById('greenValue').textContent = g;"
                 "  document.getElementById('blueValue').textContent = b;"
                 "}"
                 "function updateTime() {"
                 "  const now = new Date();"
                 "  const utc = now.getTime() + (now.getTimezoneOffset() * 60000);"
                 "  const localTime = new Date(utc + (3600000 * " + String(timezone) + "));"
                 "  const hours = String(localTime.getHours()).padStart(2, '0');"
                 "  const minutes = String(localTime.getMinutes()).padStart(2, '0');"
                 "  const seconds = String(localTime.getSeconds()).padStart(2, '0');"
                 "  document.getElementById('currentTime').textContent = "
                 "    `${hours}:${minutes}:${seconds}`;"
                 "}"
                 "document.addEventListener('DOMContentLoaded', function() {"
                 "  updateColor();"
                 "  updateTime();"
                 "  setInterval(updateTime, 1000);"
                 "  document.getElementById('scheduleForm').onsubmit = function(e) {"
                 "    e.preventDefault();"
                 "    const startTime = document.querySelector('input[name=\"startTime\"]').value.split(':');"
                 "    const endTime = document.querySelector('input[name=\"endTime\"]').value.split(':');"
                 "    const data = new FormData(this);"
                 "    data.set('enabled', document.querySelector('input[name=\"enabled\"]').checked);"
                 "    data.set('startHour', startTime[0]);"
                 "    data.set('startMinute', startTime[1]);"
                 "    data.set('endHour', endTime[0]);"
                 "    data.set('endMinute', endTime[1]);"
                 "    fetch('/schedule', {method: 'POST', body: data})"
                 "      .then(response => response.text())"
                 "      .then(text => alert(text));"
                 "    return false;"
                 "  };"
                 "});"
                 "</script>"
                 "<div class='footer'>"
                 "ESP LAMP by Eric Xu<br>"
                 "<a href='https://github.com/utsxumiao/esp-lamp' target='_blank'>GitHub Repository</a>"
                 "</div>"
                 "</body>"
                 "</html>";

    server.send(200, "text/html", html);
}

void saveSchedule() {
    EEPROM.write(EEPROM_SCHEDULE_ADDR + 0, schedule.enabled);
    EEPROM.write(EEPROM_SCHEDULE_ADDR + 1, schedule.startHour);
    EEPROM.write(EEPROM_SCHEDULE_ADDR + 2, schedule.startMinute);
    EEPROM.write(EEPROM_SCHEDULE_ADDR + 3, schedule.endHour);
    EEPROM.write(EEPROM_SCHEDULE_ADDR + 4, schedule.endMinute);
    EEPROM.write(EEPROM_SCHEDULE_ADDR + 5, schedule.brightness);
    EEPROM.write(EEPROM_SCHEDULE_ADDR + 6, schedule.red);
    EEPROM.write(EEPROM_SCHEDULE_ADDR + 7, schedule.green);
    EEPROM.write(EEPROM_SCHEDULE_ADDR + 8, schedule.blue);
    
    // Calculate and save checksum for all settings
    byte checksum = 0;
    checksum += (isPowered ? 1 : 0);
    checksum += brightness;
    checksum += redValue;
    checksum += greenValue;
    checksum += blueValue;
    checksum += (batteryCheckInterval & 0xFF);
    checksum += ((batteryCheckInterval >> 8) & 0xFF);
    checksum += (uint8_t)(batteryLowThreshold * 10);
    checksum += timezone;
    checksum += schedule.enabled ? 1 : 0;
    checksum += schedule.startHour;
    checksum += schedule.startMinute;
    checksum += schedule.endHour;
    checksum += schedule.endMinute;
    checksum += schedule.brightness;
    checksum += schedule.red;
    checksum += schedule.green;
    checksum += schedule.blue;
    
    EEPROM.write(EEPROM_CHECKSUM_ADDR, checksum);
    EEPROM.commit();
}

void loadSchedule() {
    schedule.enabled = EEPROM.read(EEPROM_SCHEDULE_ADDR + 0);
    schedule.startHour = EEPROM.read(EEPROM_SCHEDULE_ADDR + 1);
    schedule.startMinute = EEPROM.read(EEPROM_SCHEDULE_ADDR + 2);
    schedule.endHour = EEPROM.read(EEPROM_SCHEDULE_ADDR + 3);
    schedule.endMinute = EEPROM.read(EEPROM_SCHEDULE_ADDR + 4);
    schedule.brightness = EEPROM.read(EEPROM_SCHEDULE_ADDR + 5);
    schedule.red = EEPROM.read(EEPROM_SCHEDULE_ADDR + 6);
    schedule.green = EEPROM.read(EEPROM_SCHEDULE_ADDR + 7);
    schedule.blue = EEPROM.read(EEPROM_SCHEDULE_ADDR + 8);

    // Validate and initialize schedule values if invalid
    if (schedule.startHour > 23 || schedule.endHour > 23 ||
        schedule.startMinute > 59 || schedule.endMinute > 59 ||
        schedule.brightness > 100) {
        
        // Initialize with default values
        schedule.enabled = false;
        schedule.startHour = 0;
        schedule.startMinute = 0;
        schedule.endHour = 0;
        schedule.endMinute = 0;
        schedule.brightness = 100;
        schedule.red = 255;
        schedule.green = 255;
        schedule.blue = 255;
        
        // Save the initialized values
        saveSchedule();
    }
}

// Add this function to check if current time is within schedule
bool isTimeInSchedule() {
    if (!schedule.enabled) return false;
    
    // Get current hour and minute from NTP
    int currentHour = timeClient.getHours();
    int currentMinute = timeClient.getMinutes();
    int currentTime = currentHour * 60 + currentMinute;  // Convert to minutes since midnight
    int startTime = schedule.startHour * 60 + schedule.startMinute;
    int endTime = schedule.endHour * 60 + schedule.endMinute;
    
    // Handle schedules that cross midnight
    if (endTime <= startTime) {  // Changed from < to <= to handle equal times
        // Example: Start 22:00, End 06:00
        // The schedule is active from start time until midnight
        // AND from midnight until end time
        return currentTime >= startTime || currentTime < endTime;
    } else {
        // Normal case: Start 06:00, End 22:00
        // The schedule is active between start and end time
        return currentTime >= startTime && currentTime < endTime;
    }
}

// Add this function to apply scheduled settings
void applyScheduledSettings() {
    isPowered = true;
    brightness = schedule.brightness;
    redValue = schedule.red;
    greenValue = schedule.green;
    blueValue = schedule.blue;
    updateLEDs();
}

void handleRoot() {
    String html = "<!DOCTYPE html>"
                "<html>"
                "<head>"
                "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
                "<title>" + String(getDeviceName()) + "</title>"
                "<style>"
                "body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }"
                ".container { max-width: 400px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }"
                ".header-group { display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; }"
                ".power-group { display: flex; align-items: center; }"
                ".battery-level { color: #666; font-size: 0.9em; }"
                ".form-group { margin-bottom: 15px; display: flex; align-items: center; gap: 10px; }"
                ".form-group label { margin: 0; min-width: 80px; }"
                ".control-wrap { flex: 1; }"
                ".power-control { margin-bottom: 15px; display: flex; align-items: center; gap: 10px; }"
                ".power-control label { margin: 0; min-width: 36px; }"
                ".presets { display: grid; grid-template-columns: repeat(4, 1fr); gap: 16px; margin-bottom: 15px; }"
                ".preset-btn { height: 60px; width: 60px; border: 2px solid #666; border-radius: 5px; cursor: pointer; transition: 0.3s; margin: 0 auto; }"
                ".preset-btn:hover { transform: scale(1.1); }"
                ".switch { position: relative; display: inline-block; width: 36px; !important height: 20px; }"
                ".switch input { opacity: 0; width: 0; height: 0; }"
                ".slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .4s; border-radius: 34px; }"
                ".slider:before { position: absolute; content: ''; height: 16px; width: 16px; left: 2px; bottom: 2px; background-color: white; transition: .4s; border-radius: 50%; }"
                "input:checked + .slider { background-color: #2196F3; }"
                "input:checked + .slider:before { transform: translateX(16px); }"
                "input[type='range'] { width: 100%; height: 25px; -webkit-appearance: none; border-radius: 12px; outline: none; }"
                "input[type='range']::-webkit-slider-thumb { -webkit-appearance: none; width: 25px; height: 25px; background: white; border: 2px solid #666; border-radius: 50%; cursor: pointer; }"
                "#red { background: linear-gradient(to right, #FFFFFF, #FF0000); }"
                "#green { background: linear-gradient(to right, #FFFFFF, #00FF00); }"
                "#blue { background: linear-gradient(to right, #FFFFFF, #0000FF); }"
                "#brightness { background: #ddd; }"
                ".nav-links { margin-bottom: 20px; }"
                ".nav-links a { color: #2196F3; text-decoration: none; margin-right: 15px; }"
                ".nav-links a:hover { text-decoration: underline; }"
                ".footer { text-align: center; margin-top: 30px; padding-top: 20px; "
                "border-top: 1px solid #ddd; color: #666; font-size: 0.9em; }"
                ".footer a { color: #2196F3; text-decoration: none; }"
                ".footer a:hover { text-decoration: underline; }"
                "</style>"
                "</head>"
                "<body>"
                "<div class='container'>"
                "<div class='nav-links'>"
                "<a href='/'>Control</a>"
                "<a href='/schedule'>Schedule</a>"
                "<a href='/config'>Settings</a>"
                "</div>"
                "<h1>" + String(getDeviceName()) + "</h1>"
                "<div class='header-group'>"
                "<div class='power-control'>"
                "<label>ON/OFF</label>"
                "<label class='switch'>"
                "<input type='checkbox' id='power' " + String(isPowered ? "checked" : "") + ">"
                "<span class='slider'></span>"
                "</label>"
                "</div>"
                "<div class='battery-level'>Battery: " + 
                String((int)getBatteryPercentage()) + "% (" + 
                String(getBatteryVoltage(), 2) + "V)</div>"
                "</div>"
                "<div class='presets'>";

    // Generate preset buttons without text
    for (int i = 0; i < 8; i++) {
        String style = String("background: rgb(") + colorPresets[i].r + "," + 
                      colorPresets[i].g + "," + colorPresets[i].b + ");" +
                      "title='" + colorPresets[i].name + "'";  // Add tooltip instead
        html += "<button class='preset-btn' style='" + style + "' onclick='setPreset(" + 
               colorPresets[i].r + "," + colorPresets[i].g + "," + colorPresets[i].b + 
               ")'></button>";
    }

    html += "</div>"
                "<div class='control'>"
                "<label>Red</label>"
                "<div class='control-wrap'><input type='range' id='red' min='0' max='255' value='" + String(redValue) + "'></div>"
                "<div class='value-display'><span id='redValue'>" + String(redValue) + "</span></div>"
                "</div>"
                "<div class='control'>"
                "<label>Green</label>"
                "<div class='control-wrap'><input type='range' id='green' min='0' max='255' value='" + String(greenValue) + "'></div>"
                "<div class='value-display'><span id='greenValue'>" + String(greenValue) + "</span></div>"
                "</div>"
                "<div class='control'>"
                "<label>Blue</label>"
                "<div class='control-wrap'><input type='range' id='blue' min='0' max='255' value='" + String(blueValue) + "'></div>"
                "<div class='value-display'><span id='blueValue'>" + String(blueValue) + "</span></div>"
                "</div>"
                "<div class='control'>"
                "<label>Brightness</label>"
                "<div class='control-wrap'><input type='range' id='brightness' min='10' max='100' step='10' value='" + String(brightness) + "'></div>"
                "<div class='value-display'><span id='brightnessValue'>" + String(brightness) + "%</span></div>"
                "</div>"
                "</div>"
                "<script>"
                "function setPreset(r, g, b) {"
                "  document.getElementById('red').value = r;"
                "  document.getElementById('green').value = g;"
                "  document.getElementById('blue').value = b;"
                "  document.getElementById('redValue').innerHTML = r;"
                "  document.getElementById('greenValue').innerHTML = g;"
                "  document.getElementById('blueValue').innerHTML = b;"
                "  fetch('/update?r=' + r + '&g=' + g + '&b=' + b);"
                "}"
                "document.getElementById('power').onchange = function() {"
                "  fetch('/update?power=' + (this.checked ? '1' : '0'));"
                "};"
                "function updateColor() {"
                "  const r = document.getElementById('red').value;"
                "  const g = document.getElementById('green').value;"
                "  const b = document.getElementById('blue').value;"
                "  fetch('/update?r=' + r + '&g=' + g + '&b=' + b);"
                "}"
                "['red', 'green', 'blue'].forEach(color => {"
                "  const slider = document.getElementById(color);"
                "  slider.oninput = function() {"
                "    document.getElementById(color + 'Value').innerHTML = this.value;"
                "  };"
                "  slider.onchange = updateColor;"
                "});"
                "document.getElementById('brightness').oninput = function() {"
                "  document.getElementById('brightnessValue').innerHTML = this.value + '%';"
                "};"
                "document.getElementById('brightness').onchange = function() {"
                "  fetch('/update?brightness=' + this.value);"
                "};"
                "</script>"
                "<div class='footer'>"
                "ESP LAMP by Eric Xu<br>"
                "<a href='https://github.com/utsxumiao/esp-lamp' target='_blank'>GitHub Repository</a>"
                "</div>"
                "</div></body></html>";
    
    server.send(200, "text/html", html);
}

void handleUpdate() {
    if (DEBUG_OUTPUT) {
        Serial.println("\n--- Update received ---");
    }
    
    if (server.hasArg("power")) {
        isPowered = server.arg("power") == "1";
        if (DEBUG_OUTPUT) {
            Serial.printf("Power: %s\n", isPowered ? "ON" : "OFF");
        }
    }
    if (server.hasArg("brightness")) {
        String brightVal = server.arg("brightness");
        if (DEBUG_OUTPUT) {
            Serial.printf("Brightness value: %s\n", brightVal.c_str());
        }
        brightness = brightVal.toInt();
    }
    if (server.hasArg("r") && server.hasArg("g") && server.hasArg("b")) {
        redValue = server.arg("r").toInt();
        greenValue = server.arg("g").toInt();
        blueValue = server.arg("b").toInt();
        
        // Check if these values match any preset
        currentPreset = -1;  // Reset to custom color
        for (int i = 0; i < 8; i++) {
            if (redValue == colorPresets[i].r && 
                greenValue == colorPresets[i].g && 
                blueValue == colorPresets[i].b) {
                currentPreset = i;
                break;
            }
        }
        
        if (DEBUG_OUTPUT) {
            Serial.printf("Received RGB: %d,%d,%d (Preset: %d)\n", 
                         redValue, greenValue, blueValue, currentPreset);
        }
    }
    
    updateLEDs();
    server.send(200, "text/plain", "OK");
}

// Add this helper function to calculate battery percentage
int getBatteryPercentage() {
    float voltage = getBatteryVoltage();
    // Assuming 3.0V is empty and 4.2V is full for a Li-ion battery
    int percentage = round((voltage - 3.2) / (4.2 - 3.2) * 100.0);
    return constrain(percentage, 0, 100);  // Ensure value is between 0-100
}

void setup() {
    if (DEBUG_OUTPUT) {
        Serial.begin(115200);
        Serial.println("Starting...");
    }
    
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    
    EEPROM.begin(EEPROM_SIZE);
    loadSchedule(); 
    loadSettings();
    
    pixels.begin();
    pixels.setBrightness(255);
    updateLEDs();
    
    wifiManager.autoConnect(getDeviceName());
    
    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Add OTA initialization
    ArduinoOTA.setHostname(getDeviceName());  // Set the hostname for OTA
    
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {  // U_FS
            type = "filesystem";
        }
        if (DEBUG_OUTPUT) {
            Serial.println("Start updating " + type);
        }
        // Turn all LEDs blue during update
        for(int i = 0; i < NUM_LEDS; i++) {
            pixels.setPixelColor(i, pixels.Color(0, 0, 50));
        }
        pixels.show();
    });

    ArduinoOTA.onEnd([]() {
        if (DEBUG_OUTPUT) {
            Serial.println("\nUpdate Complete!");
        }
        // Turn LEDs green on completion
        for(int i = 0; i < NUM_LEDS; i++) {
            pixels.setPixelColor(i, pixels.Color(0, 50, 0));
        }
        pixels.show();
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        if (DEBUG_OUTPUT) {
            Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        if (DEBUG_OUTPUT) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) {
                Serial.println("Auth Failed");
            }
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
            // Turn LEDs red on error
            for(int i = 0; i < NUM_LEDS; i++) {
                pixels.setPixelColor(i, pixels.Color(50, 0, 0));
            }
            pixels.show();
        }
    });

    ArduinoOTA.begin();
    
    server.on("/", HTTP_GET, handleRoot);
    server.on("/update", HTTP_GET, handleUpdate);
    server.on("/config", HTTP_ANY, handleConfig);  // Handle both GET and POST
    server.on("/schedule", HTTP_ANY, handleSchedule);  // Handle both GET and POST
    server.onNotFound([](){
        if (!espalexa.handleAlexaApiCall(server.uri(),server.arg(0))) //if you don't know the URI, ask espalexa whether it is an Alexa control request
        {
            //whatever you want to do with 404s
            server.send(404, "text/plain", "Not found");
        }
    });
    // Initialize Alexa support
    espalexa.addDevice(getDeviceName(), handleAlexa, EspalexaDeviceType::extendedcolor);  // Note: removed quotes around DEVICE_NAME
    espalexa.begin(&server);

    // Initialize NTP
    timeClient.begin();
    timeClient.setTimeOffset(timezone * 3600);  // Convert timezone to seconds
}

void loop() {
    ArduinoOTA.handle();
    espalexa.loop();
    
    // Update NTP time every minute
    if (millis() - lastNTPUpdate >= ntpRetryInterval) {
        if (timeClient.update()) {
            ntpSynced = true;
            ntpRetryInterval = NTP_UPDATE_INTERVAL;  // Reset to normal interval
            if (DEBUG_OUTPUT) Serial.println("NTP time synced");
        } else {
            ntpSynced = false;
            if (DEBUG_OUTPUT) Serial.println("NTP sync failed, will retry in 1 minute");
        }
        lastNTPUpdate = millis();
    }
    
    // Only run schedule if we have valid time
    if (schedule.enabled && ntpSynced) {
        bool shouldBeOn = isTimeInSchedule();
        
        if (shouldBeOn && !scheduleActive) {
            // Schedule just became active
            scheduleActive = true;
            applyScheduledSettings();
            if (DEBUG_OUTPUT) Serial.println("Schedule activated");
        } 
        else if (!shouldBeOn && scheduleActive) {
            // Schedule just ended
            scheduleActive = false;
            isPowered = false;
            updateLEDs();
            if (DEBUG_OUTPUT) Serial.println("Schedule deactivated");
        }
    }
    
    // Battery monitoring
    if (useBattery && isPowered) {
        if (millis() - lastBatteryCheck >= (batteryCheckInterval * 1000)) {  // Convert seconds to milliseconds
            float batteryVoltage = getBatteryVoltage();
            if (DEBUG_OUTPUT) {
                Serial.printf("Battery Voltage: %.2fV\n", batteryVoltage);
            }
            if (batteryVoltage < batteryLowThreshold) {
                flashLowBatteryWarning();
            }
            lastBatteryCheck = millis();
        }
    }
    
    // Button handling with debounce
    int reading = digitalRead(BUTTON_PIN);
    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_TIME) {
        if (reading != buttonState) {
            buttonState = reading;

            if (buttonState == LOW) {  // Button pressed
                buttonPressTime = millis();
                longPressHandled = false;
            } else {  // Button released
                if (!longPressHandled && isPowered) {  // Only handle short press if long press wasn't triggered
                    // Cycle through presets
                    currentPreset = (currentPreset + 1) % 8;
                    applyPreset(currentPreset);
                    if (DEBUG_OUTPUT) {
                        Serial.printf("Switched to preset: %s\n", colorPresets[currentPreset].name);
                    }
                }
            }
        }

        // Check for long press while button is still pressed
        if (buttonState == LOW && !longPressHandled) {
            unsigned long pressDuration = millis() - buttonPressTime;
            if (pressDuration >= LONG_PRESS_TIME) {
                isPowered = !isPowered;
                updateLEDs();
                if (DEBUG_OUTPUT) {
                    Serial.printf("Power toggled: %s\n", isPowered ? "ON" : "OFF");
                }
                longPressHandled = true;  // Prevent multiple triggers
            }
        }
    }
    lastButtonState = reading;
}
