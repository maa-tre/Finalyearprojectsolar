/*
 * =============================================================================
 * SOLAR PANEL FAULT DETECTION - ESP32 FIRMWARE
 * =============================================================================
 * 
 * Features:
 * - Real-time sensor reading
 * - ML model prediction on-device
 * - Serial communication with Python dashboard
 * - Standalone mode with LED/Buzzer alerts
 * 
 * Communication Protocol:
 * - Send "GET_DATA\n" to receive sensor readings
 * - Response format: "DATA:voltage,current,temperature,light\n"
 * - Send "GET_PREDICTION\n" to get fault prediction
 * - Response format: "PRED:class_index,class_name\n"
 * 
 * Hardware:
 * - ESP32 DevKit
 * - Voltage Sensor -> GPIO 34
 * - INA219 (I2C) -> SDA: 21, SCL: 22
 * - DS18B20 -> GPIO 4
 * - TSL2561 (I2C) -> Shared bus
 * - Fault LED -> GPIO 2
 * - Buzzer -> GPIO 5
 * 
 * =============================================================================
 */

#include <Wire.h>

// Include the ML model
#include "model_manual.h"

// =============================================================================
// CONFIGURATION
// =============================================================================
#define SERIAL_BAUD_RATE        115200
#define SENSOR_READ_INTERVAL    500      // ms between readings

// Pin Definitions
#define VOLTAGE_PIN             34
#define TEMP_PIN                4
#define FAULT_LED_PIN           2
#define NORMAL_LED_PIN          15
#define BUZZER_PIN              5

// Sensor Calibration
#define VOLTAGE_DIVIDER_RATIO   11.0
#define ADC_RESOLUTION          4095.0
#define ADC_REF_VOLTAGE         3.3

// Simulation mode (set to true for testing without sensors)
#define SIMULATION_MODE         true

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
unsigned long lastReadTime = 0;
unsigned long lastSerialCheck = 0;

// Current sensor readings
float voltage = 0.0;
float current = 0.0;
float temperature = 0.0;
float lightIntensity = 0.0;

// Current prediction
int currentFaultClass = 0;
String currentFaultName = "Normal";

// Simulation variables
int simulationMode = 0;  // 0=Normal, 1=Partial_Shading, 2=Open_Circuit, 3=Short_Circuit
unsigned long lastSimChange = 0;
bool autoSimulation = false;

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================
void setupPins();
void setupSensors();
void readSensors();
void runPrediction();
void handleSerialCommands();
void updateOutputs();
void sendSensorData();
void sendPrediction();
void simulateSensorData();

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial) { delay(10); }
    
    Serial.println();
    Serial.println("========================================");
    Serial.println("  SOLAR PANEL FAULT DETECTION SYSTEM");
    Serial.println("  ESP32 Firmware v1.0");
    Serial.println("========================================");
    Serial.println();
    
    setupPins();
    setupSensors();
    
    Serial.println("System ready!");
    Serial.println("Commands: GET_DATA, GET_PREDICTION, SIM:0-3, AUTO:0/1");
    Serial.println();
    
    // Startup blink
    for (int i = 0; i < 3; i++) {
        digitalWrite(FAULT_LED_PIN, HIGH);
        delay(100);
        digitalWrite(FAULT_LED_PIN, LOW);
        delay(100);
    }
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    unsigned long currentTime = millis();
    
    // Check for serial commands
    handleSerialCommands();
    
    // Read sensors at interval
    if (currentTime - lastReadTime >= SENSOR_READ_INTERVAL) {
        lastReadTime = currentTime;
        
        // Get sensor data (real or simulated)
        if (SIMULATION_MODE) {
            simulateSensorData();
        } else {
            readSensors();
        }
        
        // Run ML prediction
        runPrediction();
        
        // Update LEDs/Buzzer
        updateOutputs();
    }
    
    // Auto simulation mode - cycle through faults
    if (autoSimulation && (currentTime - lastSimChange >= 5000)) {
        lastSimChange = currentTime;
        simulationMode = (simulationMode + 1) % 4;
        Serial.print("Auto-sim: Switching to mode ");
        Serial.println(CLASS_NAMES[simulationMode]);
    }
    
    delay(10);
}

// =============================================================================
// SETUP FUNCTIONS
// =============================================================================
void setupPins() {
    pinMode(FAULT_LED_PIN, OUTPUT);
    pinMode(NORMAL_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(VOLTAGE_PIN, INPUT);
    
    digitalWrite(FAULT_LED_PIN, LOW);
    digitalWrite(NORMAL_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    
    Serial.println("[OK] GPIO pins configured");
}

void setupSensors() {
    Wire.begin();
    
    // TODO: Initialize actual sensors here
    // - INA219 current sensor
    // - DS18B20 temperature sensor
    // - TSL2561 light sensor
    
    if (SIMULATION_MODE) {
        Serial.println("[OK] Simulation mode enabled");
    } else {
        Serial.println("[OK] Sensors initialized");
    }
}

// =============================================================================
// SENSOR READING
// =============================================================================
void readSensors() {
    // Read voltage from ADC
    int adcValue = analogRead(VOLTAGE_PIN);
    float adcVoltage = (adcValue / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    voltage = adcVoltage * VOLTAGE_DIVIDER_RATIO;
    
    // TODO: Read from INA219
    // current = ina219.getCurrent_mA() / 1000.0;
    current = 5.0;  // Placeholder
    
    // TODO: Read from DS18B20
    // temperature = tempSensor.getTempCByIndex(0);
    temperature = 35.0;  // Placeholder
    
    // TODO: Read from TSL2561
    // lightIntensity = tsl.getEvent(&event).light;
    lightIntensity = 1000.0;  // Placeholder
}

void simulateSensorData() {
    // Generate realistic simulated data based on mode
    float noise = 0.1;
    
    switch (simulationMode) {
        case 0:  // Normal
            voltage = 20.0 + random(-20, 20) / 10.0;
            current = 5.0 + random(-10, 10) / 10.0;
            temperature = 35.0 + random(-50, 50) / 10.0;
            lightIntensity = 1000.0 + random(-100, 100);
            break;
            
        case 1:  // Open_Circuit
            voltage = 22.0 + random(-10, 10) / 10.0;
            current = 0.05 + random(0, 5) / 100.0;
            temperature = 30.0 + random(-30, 30) / 10.0;
            lightIntensity = 950.0 + random(-100, 100);
            break;
            
        case 2:  // Partial_Shading
            voltage = 11.0 + random(-20, 20) / 10.0;
            current = 2.0 + random(-10, 10) / 10.0;
            temperature = 40.0 + random(-50, 50) / 10.0;
            lightIntensity = 800.0 + random(-100, 100);
            break;
            
        case 3:  // Short_Circuit
            voltage = 1.5 + random(-10, 10) / 10.0;
            current = 8.0 + random(-10, 10) / 10.0;
            temperature = 65.0 + random(-50, 50) / 10.0;
            lightIntensity = 700.0 + random(-100, 100);
            break;
    }
    
    // Clamp values
    voltage = max(0.0f, voltage);
    current = max(0.0f, current);
    lightIntensity = max(0.0f, lightIntensity);
}

// =============================================================================
// ML PREDICTION
// =============================================================================
void runPrediction() {
    // Prepare feature array
    float features[NUM_FEATURES] = {
        voltage,
        current,
        temperature,
        lightIntensity
    };
    
    // Get prediction from model
    currentFaultClass = predict(features);
    currentFaultName = String(CLASS_NAMES[currentFaultClass]);
}

// =============================================================================
// OUTPUT CONTROL
// =============================================================================
void updateOutputs() {
    // Check if fault detected
    bool isFault = (currentFaultClass != 0);  // 0 = Normal
    
    if (isFault) {
        // Fault detected
        digitalWrite(FAULT_LED_PIN, HIGH);
        digitalWrite(NORMAL_LED_PIN, LOW);
        
        // Different buzzer patterns for different faults
        switch (currentFaultClass) {
            case 3:  // Short_Circuit - Critical
                digitalWrite(BUZZER_PIN, HIGH);
                break;
            case 1:  // Open_Circuit
                digitalWrite(BUZZER_PIN, (millis() / 200) % 2);
                break;
            case 2:  // Partial_Shading
                digitalWrite(BUZZER_PIN, (millis() / 500) % 2);
                break;
        }
    } else {
        // Normal operation
        digitalWrite(FAULT_LED_PIN, LOW);
        digitalWrite(NORMAL_LED_PIN, HIGH);
        digitalWrite(BUZZER_PIN, LOW);
    }
}

// =============================================================================
// SERIAL COMMUNICATION
// =============================================================================
void handleSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "GET_DATA") {
            sendSensorData();
        }
        else if (command == "GET_PREDICTION") {
            sendPrediction();
        }
        else if (command.startsWith("SIM:")) {
            // Set simulation mode: SIM:0, SIM:1, SIM:2, SIM:3
            int mode = command.substring(4).toInt();
            if (mode >= 0 && mode <= 3) {
                simulationMode = mode;
                Serial.print("OK:SIM_MODE=");
                Serial.println(CLASS_NAMES[mode]);
            }
        }
        else if (command.startsWith("AUTO:")) {
            // Auto simulation: AUTO:1 (enable), AUTO:0 (disable)
            autoSimulation = (command.substring(5).toInt() == 1);
            Serial.print("OK:AUTO_SIM=");
            Serial.println(autoSimulation ? "ON" : "OFF");
        }
        else if (command == "STATUS") {
            // Send full status
            Serial.println("STATUS:BEGIN");
            Serial.print("  Voltage: "); Serial.print(voltage); Serial.println(" V");
            Serial.print("  Current: "); Serial.print(current); Serial.println(" A");
            Serial.print("  Temperature: "); Serial.print(temperature); Serial.println(" C");
            Serial.print("  Light: "); Serial.print(lightIntensity); Serial.println(" lux");
            Serial.print("  Power: "); Serial.print(voltage * current); Serial.println(" W");
            Serial.print("  Fault: "); Serial.println(currentFaultName);
            Serial.print("  Sim Mode: "); Serial.println(SIMULATION_MODE ? "ON" : "OFF");
            Serial.println("STATUS:END");
        }
        else if (command == "HELP") {
            Serial.println("Commands:");
            Serial.println("  GET_DATA      - Get sensor readings");
            Serial.println("  GET_PREDICTION - Get fault prediction");
            Serial.println("  SIM:0-3       - Set simulation mode");
            Serial.println("  AUTO:0/1      - Auto-cycle simulation");
            Serial.println("  STATUS        - Full system status");
        }
    }
}

void sendSensorData() {
    // Format: DATA:voltage,current,temperature,light
    Serial.print("DATA:");
    Serial.print(voltage, 2);
    Serial.print(",");
    Serial.print(current, 2);
    Serial.print(",");
    Serial.print(temperature, 2);
    Serial.print(",");
    Serial.println(lightIntensity, 2);
}

void sendPrediction() {
    // Format: PRED:class_index,class_name,confidence
    Serial.print("PRED:");
    Serial.print(currentFaultClass);
    Serial.print(",");
    Serial.println(currentFaultName);
}

// =============================================================================
// END OF FIRMWARE
// =============================================================================
