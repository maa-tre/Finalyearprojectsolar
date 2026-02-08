/*
 * Solar Panel Fault Detection - Arduino Nano USB Firmware
 * 
 * This firmware allows the Arduino Nano to:
 * 1. Read sensor data from analog pins
 * 2. Send data via USB serial to the computer
 * 3. The ML prediction is done on the computer (Python backend)
 * 
 * Hardware Connections:
 * - Voltage Sensor: A0 (with voltage divider for 0-30V range)
 * - Current Sensor: A1 (ACS712 or similar)
 * - Temperature Sensor: A2 (LM35 or thermistor)
 * - Light Sensor: A3 (LDR with resistor divider)
 * - Status LED: D13 (built-in)
 * 
 * Serial Protocol:
 * - Baud Rate: 115200
 * - Output Format: JSON {"v":voltage,"i":current,"t":temp,"l":light}
 * - Commands: "START", "STOP", "STATUS", "CALIB"
 */

// Pin Definitions
#define VOLTAGE_PIN A0
#define CURRENT_PIN A1
#define TEMP_PIN A2
#define LIGHT_PIN A3
#define LED_PIN 13

// Sensor calibration values (adjust based on your hardware)
// Voltage divider: Vin -> 100k -> ADC -> 10k -> GND (1:11 ratio)
const float VOLTAGE_DIVIDER_RATIO = 11.0;
const float VREF = 5.0;
const float ADC_MAX = 1023.0;

// ACS712-05B: 185mV/A, Zero point at 2.5V
const float ACS712_SENSITIVITY = 0.185;  // V/A
const float ACS712_ZERO = 2.5;           // V

// LM35: 10mV/°C
const float LM35_SCALE = 0.01;  // V/°C

// LDR calibration (depends on your LDR and resistor)
const float LDR_MAX_LUX = 1500.0;

// Sampling settings
const int NUM_SAMPLES = 20;
const unsigned long SAMPLE_INTERVAL = 500;  // ms

// State
bool isMonitoring = false;
unsigned long lastSampleTime = 0;
float voltage = 0, current = 0, temperature = 0, light = 0;

// Smoothing (exponential moving average)
const float ALPHA = 0.3;  // Smoothing factor (0-1, higher = less smooth)

void setup() {
    Serial.begin(115200);
    
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(CURRENT_PIN, INPUT);
    pinMode(TEMP_PIN, INPUT);
    pinMode(LIGHT_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    
    // Startup indication
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
    
    Serial.println("{\"status\":\"ready\",\"device\":\"Arduino Nano\",\"version\":\"1.0\"}");
}

void loop() {
    // Handle serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command == "START") {
            isMonitoring = true;
            Serial.println("{\"status\":\"monitoring_started\"}");
            digitalWrite(LED_PIN, HIGH);
        }
        else if (command == "STOP") {
            isMonitoring = false;
            Serial.println("{\"status\":\"monitoring_stopped\"}");
            digitalWrite(LED_PIN, LOW);
        }
        else if (command == "STATUS") {
            sendStatus();
        }
        else if (command == "READ") {
            // Single read
            readSensors();
            sendData();
        }
        else if (command == "CALIB") {
            calibrate();
        }
        else if (command == "PING") {
            Serial.println("{\"pong\":true}");
        }
    }
    
    // Continuous monitoring mode
    if (isMonitoring && (millis() - lastSampleTime >= SAMPLE_INTERVAL)) {
        lastSampleTime = millis();
        readSensors();
        sendData();
        
        // Toggle LED to show activity
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}

void readSensors() {
    // Take multiple samples and average
    long vSum = 0, iSum = 0, tSum = 0, lSum = 0;
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        vSum += analogRead(VOLTAGE_PIN);
        iSum += analogRead(CURRENT_PIN);
        tSum += analogRead(TEMP_PIN);
        lSum += analogRead(LIGHT_PIN);
        delayMicroseconds(100);
    }
    
    // Calculate raw values
    float vRaw = (vSum / (float)NUM_SAMPLES) * (VREF / ADC_MAX) * VOLTAGE_DIVIDER_RATIO;
    float iRaw = ((iSum / (float)NUM_SAMPLES) * (VREF / ADC_MAX) - ACS712_ZERO) / ACS712_SENSITIVITY;
    float tRaw = (tSum / (float)NUM_SAMPLES) * (VREF / ADC_MAX) / LM35_SCALE;
    float lRaw = (lSum / (float)NUM_SAMPLES) / ADC_MAX * LDR_MAX_LUX;
    
    // Apply exponential smoothing
    voltage = ALPHA * vRaw + (1 - ALPHA) * voltage;
    current = ALPHA * abs(iRaw) + (1 - ALPHA) * current;  // abs() because ACS712 can read negative
    temperature = ALPHA * tRaw + (1 - ALPHA) * temperature;
    light = ALPHA * lRaw + (1 - ALPHA) * light;
    
    // Clamp to valid ranges
    voltage = constrain(voltage, 0, 30);
    current = constrain(current, 0, 12);
    temperature = constrain(temperature, -10, 100);
    light = constrain(light, 0, 1500);
}

void sendData() {
    // Send JSON formatted data
    Serial.print("{");
    Serial.print("\"v\":");
    Serial.print(voltage, 2);
    Serial.print(",\"i\":");
    Serial.print(current, 2);
    Serial.print(",\"t\":");
    Serial.print(temperature, 1);
    Serial.print(",\"l\":");
    Serial.print(light, 0);
    Serial.print(",\"p\":");
    Serial.print(voltage * current, 2);
    Serial.print(",\"ts\":");
    Serial.print(millis());
    Serial.println("}");
}

void sendStatus() {
    Serial.print("{");
    Serial.print("\"monitoring\":");
    Serial.print(isMonitoring ? "true" : "false");
    Serial.print(",\"uptime\":");
    Serial.print(millis());
    Serial.print(",\"samples\":");
    Serial.print(NUM_SAMPLES);
    Serial.print(",\"interval\":");
    Serial.print(SAMPLE_INTERVAL);
    Serial.println("}");
}

void calibrate() {
    Serial.println("{\"status\":\"calibrating\"}");
    
    // Read raw ADC values for calibration
    Serial.print("{\"raw_adc\":{");
    Serial.print("\"voltage\":");
    Serial.print(analogRead(VOLTAGE_PIN));
    Serial.print(",\"current\":");
    Serial.print(analogRead(CURRENT_PIN));
    Serial.print(",\"temp\":");
    Serial.print(analogRead(TEMP_PIN));
    Serial.print(",\"light\":");
    Serial.print(analogRead(LIGHT_PIN));
    Serial.println("}}");
}
