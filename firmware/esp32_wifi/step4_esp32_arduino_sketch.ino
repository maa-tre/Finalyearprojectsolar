/*
 * =============================================================================
 * STEP 4: ESP32 Arduino Sketch - Solar Panel Fault Detection
 * =============================================================================
 * 
 * Hardware Setup:
 * - ESP32 DevKit
 * - Voltage Sensor (voltage divider) -> GPIO 34 (ADC1_CH6)
 * - INA219 Current Sensor -> I2C (SDA: GPIO 21, SCL: GPIO 22)
 * - DS18B20 Temperature Sensor -> GPIO 4 (OneWire)
 * - TSL2561 Light Sensor -> I2C (shared bus)
 * - Fault LED -> GPIO 2 (onboard LED)
 * - Buzzer (optional) -> GPIO 5
 * 
 * Libraries Required:
 * - Wire.h (built-in)
 * - Adafruit_INA219
 * - OneWire
 * - DallasTemperature
 * - Adafruit_TSL2561
 * 
 * =============================================================================
 */

#include <Wire.h>

// Include the ML model (generated from Python)
#include "model_manual.h"

// =============================================================================
// PIN DEFINITIONS
// =============================================================================
#define VOLTAGE_SENSOR_PIN  34    // ADC pin for voltage sensor
#define TEMP_SENSOR_PIN     4     // OneWire pin for DS18B20
#define FAULT_LED_PIN       2     // Onboard LED (fault indicator)
#define BUZZER_PIN          5     // Buzzer for audible alert
#define NORMAL_LED_PIN      15    // Green LED for normal status

// =============================================================================
// SENSOR CALIBRATION CONSTANTS
// =============================================================================
// Voltage divider calibration (adjust based on your circuit)
#define VOLTAGE_DIVIDER_RATIO   11.0    // R1=10k, R2=1k -> ratio = (R1+R2)/R2
#define ADC_RESOLUTION          4095.0  // 12-bit ADC
#define ADC_REFERENCE_VOLTAGE   3.3     // ESP32 reference voltage

// =============================================================================
// TIMING CONSTANTS
// =============================================================================
#define SENSOR_READ_INTERVAL    1000    // Read sensors every 1 second
#define SERIAL_BAUD_RATE        115200

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
unsigned long lastReadTime = 0;
int currentFaultStatus = 0;  // 0 = Normal
int consecutiveFaults = 0;   // Count consecutive fault readings
const int FAULT_THRESHOLD = 3;  // Require 3 consecutive faults before alert

// Sensor readings
float voltage = 0.0;
float current = 0.0;
float temperature = 0.0;
float lightIntensity = 0.0;

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================
void initializePins();
void initializeSensors();
float readVoltage();
float readCurrent();
float readTemperature();
float readLightIntensity();
void processFaultDetection();
void handleFaultStatus(int faultType);
void printSensorData();
void blinkLED(int pin, int times, int delayMs);

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) {
        ; // Wait for serial port
    }
    
    Serial.println();
    Serial.println("╔═══════════════════════════════════════════════════════╗");
    Serial.println("║     SOLAR PANEL FAULT DETECTION SYSTEM - ESP32        ║");
    Serial.println("║         Random Forest ML Model Active                 ║");
    Serial.println("╚═══════════════════════════════════════════════════════╝");
    Serial.println();
    
    // Initialize hardware
    initializePins();
    initializeSensors();
    
    // Print model info
    Serial.println("📊 Model Configuration:");
    Serial.print("   Features: ");
    Serial.println(NUM_FEATURES);
    Serial.print("   Classes: ");
    Serial.println(NUM_CLASSES);
    Serial.print("   Trees: ");
    Serial.println(NUM_TREES);
    Serial.println();
    
    Serial.println("🏷️  Fault Classes:");
    for (int i = 0; i < NUM_CLASSES; i++) {
        Serial.print("   ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(CLASS_NAMES[i]);
    }
    Serial.println();
    
    // Startup indication
    blinkLED(FAULT_LED_PIN, 3, 200);
    
    Serial.println("✅ System initialized. Starting monitoring...");
    Serial.println("─────────────────────────────────────────────────────────");
    Serial.println();
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    unsigned long currentTime = millis();
    
    // Read sensors at defined interval
    if (currentTime - lastReadTime >= SENSOR_READ_INTERVAL) {
        lastReadTime = currentTime;
        
        // Read all sensors
        voltage = readVoltage();
        current = readCurrent();
        temperature = readTemperature();
        lightIntensity = readLightIntensity();
        
        // Process fault detection using ML model
        processFaultDetection();
        
        // Print data to serial
        printSensorData();
    }
    
    // Small delay to prevent watchdog issues
    delay(10);
}

// =============================================================================
// INITIALIZATION FUNCTIONS
// =============================================================================
void initializePins() {
    // Configure output pins
    pinMode(FAULT_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(NORMAL_LED_PIN, OUTPUT);
    
    // Configure input pins
    pinMode(VOLTAGE_SENSOR_PIN, INPUT);
    
    // Set initial states
    digitalWrite(FAULT_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(NORMAL_LED_PIN, LOW);
    
    Serial.println("✅ GPIO pins initialized");
}

void initializeSensors() {
    // Initialize I2C
    Wire.begin();
    
    // TODO: Initialize INA219
    // Adafruit_INA219 ina219;
    // if (!ina219.begin()) {
    //     Serial.println("❌ INA219 not found!");
    // }
    
    // TODO: Initialize DS18B20
    // OneWire oneWire(TEMP_SENSOR_PIN);
    // DallasTemperature tempSensor(&oneWire);
    // tempSensor.begin();
    
    // TODO: Initialize TSL2561
    // Adafruit_TSL2561_Unified tsl(TSL2561_ADDR_FLOAT);
    // if (!tsl.begin()) {
    //     Serial.println("❌ TSL2561 not found!");
    // }
    
    Serial.println("✅ Sensors initialized (using placeholders)");
}

// =============================================================================
// SENSOR READING FUNCTIONS
// =============================================================================

/**
 * Read voltage from voltage divider sensor
 * TODO: Replace with actual sensor reading
 */
float readVoltage() {
    // Read ADC value
    int adcValue = analogRead(VOLTAGE_SENSOR_PIN);
    
    // Convert to voltage
    float measuredVoltage = (adcValue / ADC_RESOLUTION) * ADC_REFERENCE_VOLTAGE;
    
    // Apply voltage divider ratio
    float actualVoltage = measuredVoltage * VOLTAGE_DIVIDER_RATIO;
    
    // PLACEHOLDER: Generate test values for demonstration
    // Remove this block when using real sensors
    #ifdef USE_PLACEHOLDER_DATA
    static int testCase = 0;
    float testVoltages[] = {20.0, 10.0, 22.0, 1.0};
    actualVoltage = testVoltages[testCase % 4] + random(-10, 10) / 10.0;
    #endif
    
    return actualVoltage;
}

/**
 * Read current from INA219 sensor
 * TODO: Replace with actual INA219 reading
 */
float readCurrent() {
    // PLACEHOLDER: Replace with actual INA219 code
    // float current_mA = ina219.getCurrent_mA();
    // return current_mA / 1000.0; // Convert to Amps
    
    // Placeholder: Generate test values
    #ifdef USE_PLACEHOLDER_DATA
    static int testCase = 0;
    float testCurrents[] = {5.0, 2.0, 0.05, 8.0};
    return testCurrents[testCase % 4] + random(-5, 5) / 10.0;
    #else
    return 5.0; // Default placeholder
    #endif
}

/**
 * Read temperature from DS18B20 sensor
 * TODO: Replace with actual DS18B20 reading
 */
float readTemperature() {
    // PLACEHOLDER: Replace with actual DS18B20 code
    // tempSensor.requestTemperatures();
    // return tempSensor.getTempCByIndex(0);
    
    // Placeholder: Generate test values
    #ifdef USE_PLACEHOLDER_DATA
    static int testCase = 0;
    float testTemps[] = {35.0, 45.0, 30.0, 65.0};
    return testTemps[testCase % 4] + random(-20, 20) / 10.0;
    #else
    return 35.0; // Default placeholder
    #endif
}

/**
 * Read light intensity from TSL2561 sensor
 * TODO: Replace with actual TSL2561 reading
 */
float readLightIntensity() {
    // PLACEHOLDER: Replace with actual TSL2561 code
    // sensors_event_t event;
    // tsl.getEvent(&event);
    // return event.light;
    
    // Placeholder: Generate test values
    #ifdef USE_PLACEHOLDER_DATA
    static int testCase = 0;
    float testLight[] = {1000.0, 800.0, 900.0, 700.0};
    float light = testLight[testCase++ % 4] + random(-50, 50);
    return light;
    #else
    return 1000.0; // Default placeholder
    #endif
}

// =============================================================================
// FAULT DETECTION (ML MODEL)
// =============================================================================

/**
 * Process sensor data through the Random Forest model
 */
void processFaultDetection() {
    // Prepare feature array [Voltage, Current, Temperature, Light_Intensity]
    float features[NUM_FEATURES] = {
        voltage,
        current,
        temperature,
        lightIntensity
    };
    
    // Get prediction from ML model
    int prediction = predict(features);
    currentFaultStatus = prediction;
    
    // Handle fault status
    handleFaultStatus(prediction);
}

/**
 * Handle the detected fault status
 * @param faultType: 0=Normal, 1=Open_Circuit, 2=Partial_Shading, 3=Short_Circuit
 */
void handleFaultStatus(int faultType) {
    // Check if it's a fault (anything other than Normal)
    if (is_fault((float[]){voltage, current, temperature, lightIntensity})) {
        consecutiveFaults++;
        
        // Only trigger alert after consecutive faults (debouncing)
        if (consecutiveFaults >= FAULT_THRESHOLD) {
            // Turn on fault LED
            digitalWrite(FAULT_LED_PIN, HIGH);
            digitalWrite(NORMAL_LED_PIN, LOW);
            
            // Sound buzzer based on fault severity
            switch (faultType) {
                case 3: // Short_Circuit - most critical
                    // Continuous buzzer
                    digitalWrite(BUZZER_PIN, HIGH);
                    break;
                    
                case 1: // Open_Circuit - critical
                    // Intermittent beep
                    digitalWrite(BUZZER_PIN, HIGH);
                    delay(100);
                    digitalWrite(BUZZER_PIN, LOW);
                    break;
                    
                case 2: // Partial_Shading - warning
                    // Short beep
                    digitalWrite(BUZZER_PIN, HIGH);
                    delay(50);
                    digitalWrite(BUZZER_PIN, LOW);
                    break;
                    
                default:
                    break;
            }
            
            // Print fault alert
            Serial.println("🚨 ═══════════════════════════════════════════════════");
            Serial.print("🚨 FAULT DETECTED: ");
            Serial.println(CLASS_NAMES[faultType]);
            Serial.println("🚨 ═══════════════════════════════════════════════════");
        }
    } else {
        // Normal operation
        consecutiveFaults = 0;
        
        // Turn off fault indicators
        digitalWrite(FAULT_LED_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(NORMAL_LED_PIN, HIGH);
    }
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/**
 * Print sensor data and prediction to serial
 */
void printSensorData() {
    Serial.println("┌─────────────────────────────────────────────────────────┐");
    Serial.print("│ 🔋 Voltage: ");
    Serial.print(voltage, 2);
    Serial.print(" V");
    Serial.print("    ⚡ Current: ");
    Serial.print(current, 2);
    Serial.println(" A        │");
    
    Serial.print("│ 🌡️  Temp: ");
    Serial.print(temperature, 1);
    Serial.print(" °C");
    Serial.print("       ☀️  Light: ");
    Serial.print(lightIntensity, 0);
    Serial.println(" lux     │");
    
    Serial.print("│ 📊 Status: ");
    Serial.print(CLASS_NAMES[currentFaultStatus]);
    
    // Add padding
    int padding = 40 - strlen(CLASS_NAMES[currentFaultStatus]);
    for (int i = 0; i < padding; i++) Serial.print(" ");
    Serial.println("│");
    
    Serial.println("└─────────────────────────────────────────────────────────┘");
    Serial.println();
}

/**
 * Blink LED for visual feedback
 */
void blinkLED(int pin, int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        digitalWrite(pin, HIGH);
        delay(delayMs);
        digitalWrite(pin, LOW);
        delay(delayMs);
    }
}

// =============================================================================
// OPTIONAL: WiFi REPORTING (Uncomment to enable)
// =============================================================================
/*
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* serverUrl = "http://your-server.com/api/fault";

void setupWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi connected");
}

void reportFaultToServer(int faultType) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(serverUrl);
        http.addHeader("Content-Type", "application/json");
        
        String payload = "{\"fault_type\":" + String(faultType) + 
                        ",\"voltage\":" + String(voltage) +
                        ",\"current\":" + String(current) +
                        ",\"temperature\":" + String(temperature) +
                        ",\"light\":" + String(lightIntensity) + "}";
        
        int httpCode = http.POST(payload);
        http.end();
    }
}
*/

// =============================================================================
// END OF SKETCH
// =============================================================================
