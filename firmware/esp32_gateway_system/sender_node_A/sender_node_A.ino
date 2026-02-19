/* 
 * METHOD: SSID SCANNING (Station 2)
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel()
#include <DHT.h>      // For DHT22 Temperature/Humidity Sensor
#include <math.h>     // For log() in Thermistor calculation

// --- SENDER CONFIGURATION ---
// Unique identifier for Station 2
const int SENDER_ID = 2; 

// --- Wi-Fi & ESP-NOW Configuration ---
// MAC Address of the Central Gateway (Receiver)
uint8_t centralNodeAddress[] = {0x88, 0x57, 0x21, 0x8E, 0xC2, 0xBC}; 
const uint8_t DEFAULT_WIFI_CHANNEL = 1;

// --- SSID Scanning Configuration (Discovery Method) ---
// This name MUST match the Gateway's SoftAP SSID
const char* GATEWAY_SSID = "Solar_Panel_Gateway"; 

// --- Sensor Pin Definitions (STATION 2 SPECIFIC) ---
#define DHTPIN               12 
#define DHTTYPE              DHT22
#define LDR_PIN              35
#define THERMISTOR_PIN       34
#define VOLTAGE_SENSOR_PIN   33
#define CURRENT_SENSOR_PIN   32
#define RELAY_PIN            2 // Active LOW Relay

// --- Thermistor Constants ---
const float VIN_MV                   = 3240.0; // Your 3.24V measurement
const float R_FIXED                  = 9700.0; // Your 9.7k resistor measurement
const float BETA                     = 3950.0;
const float R_NOMINAL                = 10000.0;
const float TEMP_OFFSET              = 0.0;    

// --- EMPIRICAL CALIBRATION CONSTANTS FOR CURRENT SENSOR ---
const float V_PIN_ZERO       = 1.490;   
const float REAL_SENSITIVITY = 0.1755;  
const float ESP_VREF         = 3.3;     
const float ESP32_ADC_MAX    = 4095.0;  

// --- Voltage Sensor Calibration ---
const float VOLTAGE_CALIBRATION_FACTOR   = 1.00; 
const float VOLTAGE_CALIBRATION_OFFSET_V = 0.0;

// --- General Measurement Constants ---
const int NUM_ADC_READINGS_AVG = 50;
const int MEASUREMENT_INTERVAL_MS = 5000;
const int CURRENT_SENSOR_SAMPLES = 500; 

// --- Sensor Objects ---
DHT dht(DHTPIN, DHTTYPE);

// --- Command Structure (for receiving commands from central node) ---
typedef struct struct_command {
    char command[32]; 
} struct_command;

// --- Data Structure (MUST match Receiver) ---
typedef struct struct_message {
    int   senderId; 
    int   ldrValue;
    float dhtTemp;
    float humidity;
    float thermistorTemp;
    float voltage;
    float current; 
    bool  relayStatus;
    bool  valid;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// --- ESP-NOW Send Callback ---
void OnDataSent(const esp_now_send_info_t* send_info, esp_now_send_status_t status) {
    Serial.print("Last Packet Send Status to ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", send_info->des_addr[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.print(" : ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// --- ESP-NOW Receive Callback (for commands from central node) ---
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    struct_command received_cmd;
    memcpy(&received_cmd, incomingData, sizeof(received_cmd));

    Serial.printf("[RELAY CMD] Received: '%s'\n", received_cmd.command);

    if (strcmp(received_cmd.command, "ACTIVATE_RELAY") == 0) {
        Serial.println("[RELAY] Activating relay...");
        digitalWrite(RELAY_PIN, HIGH);
    } else if (strcmp(received_cmd.command, "DEACTIVATE_RELAY") == 0) {
        Serial.println("[RELAY] Deactivating relay...");
        digitalWrite(RELAY_PIN, LOW);
    } else if (strcmp(received_cmd.command, "TOGGLE_RELAY") == 0) {
        Serial.println("[RELAY] Toggling relay...");
        digitalWrite(RELAY_PIN, !digitalRead(RELAY_PIN));
    }
}

// --- SSID Scanner Helper ---
int32_t getWiFiChannel(const char *ssid) {
  Serial.printf("Scanning for Gateway SSID: %s...\n", ssid);
  int32_t n = WiFi.scanNetworks();
  for (uint8_t i=0; i<n; i++) {
    if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
      int32_t ch = WiFi.channel(i);
      Serial.printf("  Found Gateway! Channel: %d\n", ch);
      return ch;
    }
  }
  Serial.println("  Gateway not found in scan. Using default channel.");
  return 0;
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("\n--- ESP32 Sensor Node (Sender 2) Starting ---");
    Serial.println("METHOD: SSID SCANNING");

    // Initialize Relay Pin
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); 
    
    // Initialize sensors
    dht.begin();

    // Wi-Fi & ESP-NOW setup
    WiFi.mode(WIFI_STA);
    Serial.print("Sender MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Discovery Logic: Scan for Gateway before starting ESP-NOW
    int32_t gatewayChannel = getWiFiChannel(GATEWAY_SSID);
    if (gatewayChannel == 0) gatewayChannel = DEFAULT_WIFI_CHANNEL;

    esp_wifi_set_channel(gatewayChannel, WIFI_SECOND_CHAN_NONE);
    Serial.printf("ESP-NOW Channel locked to: %d\n", gatewayChannel);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW. Restarting...");
        ESP.restart();
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv); 

    memcpy(peerInfo.peer_addr, centralNodeAddress, 6);
    peerInfo.channel = gatewayChannel; 
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer. Restarting...");
        ESP.restart();
    }

    Serial.println("--- ESP-NOW Sender Ready ---");
}

// --- Loop ---
unsigned long lastMeasurementTime = 0;

void loop() {
    if (millis() - lastMeasurementTime >= MEASUREMENT_INTERVAL_MS) {
        lastMeasurementTime = millis();
        Serial.printf("\n--- Reading Sensors for Sender %d ---\n", SENDER_ID);

        myData.senderId = SENDER_ID;
        myData.valid = true;

        // --- LDR ---
        myData.ldrValue = analogRead(LDR_PIN);
        Serial.printf("LDR: %d\n", myData.ldrValue);

        // --- DHT22 ---
        myData.dhtTemp = dht.readTemperature();
        myData.humidity = dht.readHumidity();
        if (!isnan(myData.dhtTemp) && !isnan(myData.humidity)) {
            Serial.printf("DHT Temp: %.2f C | Humidity: %.2f %%\n", myData.dhtTemp, myData.humidity);
        } else {
            Serial.println("DHT Temp: Error | Humidity: Error");
        }

        // --- Thermistor (Calibrated Logic) ---
        uint32_t mvSum = 0;
        for(int i = 0; i < 30; i++) {
            mvSum += analogReadMilliVolts(THERMISTOR_PIN);
            delay(2);
        }
        float vOutMV = mvSum / 30.0;

        if (vOutMV > 100) { 
            float resistance = R_FIXED * ((VIN_MV / vOutMV) - 1.0);
            float steinhart = log(resistance / R_NOMINAL);
            steinhart /= BETA;
            steinhart += 1.0 / (25.0 + 273.15);
            steinhart = 1.0 / steinhart;
            myData.thermistorTemp = (steinhart - 273.15) + TEMP_OFFSET;
        } else {
            myData.thermistorTemp = 0.00;
        }
        Serial.printf("Thermistor Temp: %.2f C\n", myData.thermistorTemp);

        // --- VOLTAGE SENSOR (DUAL-SCALE CALIBRATION) ---
        int voltageRawADC = analogRead(VOLTAGE_SENSOR_PIN);
        float voltageAtPinV = (voltageRawADC / ESP32_ADC_MAX) * ESP_VREF;
        
        float baseVoltage = (voltageAtPinV * 5.0);
        
        if (baseVoltage < 1.0) {
            // Low Voltage Mode: Maintain clean 0V floor
            myData.voltage = baseVoltage * VOLTAGE_CALIBRATION_FACTOR;
        } else {
            // High Voltage Mode: Apply manual hardware compensation (+0.140V at pin)
            myData.voltage = (voltageAtPinV + 0.140) * 5.0 * VOLTAGE_CALIBRATION_FACTOR;
        }
        myData.voltage += VOLTAGE_CALIBRATION_OFFSET_V;
        
        Serial.printf("Voltage: %.2f V (Pin V: %.3f)\n", myData.voltage, voltageAtPinV);

        // --- CURRENT SENSOR (NEW EMPIRICAL LOGIC) ---
        long adcRawSum = 0;
        for(int i = 0; i < CURRENT_SENSOR_SAMPLES; i++) {
            adcRawSum += analogRead(CURRENT_SENSOR_PIN);
            delayMicroseconds(50);
        }
        float avgADC = (float)adcRawSum / CURRENT_SENSOR_SAMPLES;
        
        // Calculate voltage seen at the ESP32 Pin
        float voltageAtPin = (avgADC / ESP32_ADC_MAX) * ESP_VREF;
        
        // Calculate Current using Empirical Sensitivity
        myData.current = (voltageAtPin - V_PIN_ZERO) / REAL_SENSITIVITY;

        // Noise gate: ignore very small currents
        if (abs(myData.current) < 0.03) myData.current = 0.00;

        Serial.printf("Pin Voltage: %.3f V | Current: %.2f A\n", voltageAtPin, myData.current);

        // --- Send via ESP-NOW ---
        myData.relayStatus = (digitalRead(RELAY_PIN) == HIGH);
        esp_err_t result = esp_now_send(centralNodeAddress, (uint8_t*)&myData, sizeof(myData));
        
        Serial.printf("Packet: %s | Relay: %s\n", 
                      (result == ESP_OK ? "Queued" : "Error"), 
                      (myData.relayStatus ? "ON" : "OFF"));
    }
}
