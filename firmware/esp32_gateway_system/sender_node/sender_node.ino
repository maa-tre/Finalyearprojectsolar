/*
 * METHOD: SSID SCANNING (Station 1)
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Required for esp_wifi_set_channel()
#include <DHT.h>      // For DHT22 Temperature/Humidity Sensor
#include <math.h>     // For log() in Thermistor calculation

// --- SENDER CONFIGURATION ---
const int SENDER_ID = 1; 

// --- Wi-Fi & ESP-NOW Configuration ---
// MAC Address of the Central Gateway (Receiver)
uint8_t centralNodeAddress[] = {0x10, 0x52, 0x1C, 0xA7, 0x54, 0x08}; 
const uint8_t DEFAULT_WIFI_CHANNEL = 1;

// --- SSID Scanning Configuration (Discovery Method) ---
// This name MUST match the Gateway's SoftAP SSID
const char* GATEWAY_SSID = "Solar_Panel_Gateway"; 

// --- Sensor Pin Definitions ---
#define DHTPIN               15
#define DHTTYPE              DHT22
#define LDR_PIN              33
#define THERMISTOR_PIN       32
#define VOLTAGE_SENSOR_PIN   35
#define CURRENT_SENSOR_PIN   34
#define RELAY_PIN            13 // Active LOW Relay (HIGH = OFF, LOW = Triggered)

// --- Thermistor Constants ---
const float VIN_MV                   = 3240.0; // Your 3.24V measurement
const float R_FIXED                  = 9700.0; // Your 9.7k resistor measurement
const float BETA                     = 3950.0;
const float R_NOMINAL                = 10000.0;
const float TEMP_OFFSET              = 0.0;    

// --- VOLTAGE SENSOR CALIBRATION (Regression) ---
float smoothedVoltageAdc = 0;
const float voltageSmoothing = 0.9;  // Faster settling time (90% new data, 10% old)

float voltageCal[][2] = {
  { 0.0,    0.00 },
  { 100.0,  1.80 },
  { 190.0,  2.44 },
  { 676.0,  6.00 },
  { 1645.0, 12.80 },
  { 2065.0, 15.80 },
  { 2336.0, 17.80 },
  { 2719.0, 20.03 },
  { 3000.0, 25.00 }
};
const int numVoltagePoints = sizeof(voltageCal) / sizeof(voltageCal[0]);

// --- CURRENT SENSOR CALIBRATION (ACS712-5A) ---
const float V_PIN_ZERO       = 1.563;   
const float REAL_SENSITIVITY = 0.125;  
const float ESP_VREF         = 3.3;     
const float ESP32_ADC_MAX    = 4095.0;

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

// --- Voltage Regression Function ---
float getRegressedVoltage(float adc) {
    if (adc <= voltageCal[0][0]) return voltageCal[0][1];
    if (adc >= voltageCal[numVoltagePoints-1][0]) return voltageCal[numVoltagePoints-1][1];
    
    for (int i = 0; i < numVoltagePoints - 1; i++) {
        if (adc >= voltageCal[i][0] && adc <= voltageCal[i+1][0]) {
            return voltageCal[i][1] + (adc - voltageCal[i][0]) * (voltageCal[i+1][1] - voltageCal[i][1]) / 
                   (voltageCal[i+1][0] - voltageCal[i][0]);
        }
    }
    return 0;
}

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
        digitalWrite(RELAY_PIN, LOW);  // Pull LOW to trigger relay
    } else if (strcmp(received_cmd.command, "DEACTIVATE_RELAY") == 0) {
        digitalWrite(RELAY_PIN, HIGH);  // Pull HIGH to deactivate
    } else if (strcmp(received_cmd.command, "TOGGLE_RELAY") == 0) {
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

    Serial.println("\n--- ESP32 Sensor Node (Sender 1) Starting ---");
    Serial.println("METHOD: SSID SCANNING");

    // Initialize Relay Pin
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, HIGH);  // Default HIGH (relay not cut off) 
    
    // Initialize sensors
    dht.begin();
    smoothedVoltageAdc = analogRead(VOLTAGE_SENSOR_PIN);

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
        
        // Handle NaN values - set to 0.0 if read fails
        if (isnan(myData.dhtTemp)) myData.dhtTemp = 0.0;
        if (isnan(myData.humidity)) myData.humidity = 0.0;
        
        Serial.printf("DHT Temp: %.2f C | Humidity: %.2f %%\n", myData.dhtTemp, myData.humidity);

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

        // --- VOLTAGE SENSOR (REGRESSION CALIBRATION) ---
        long voltageSum = 0;
        for(int i = 0; i < 100; i++) voltageSum += analogRead(VOLTAGE_SENSOR_PIN);
        float voltageAvg = (float)voltageSum / 100.0;
        smoothedVoltageAdc = (voltageAvg * voltageSmoothing) + (smoothedVoltageAdc * (1.0 - voltageSmoothing));
        myData.voltage = getRegressedVoltage(smoothedVoltageAdc);
        
        Serial.printf("Voltage: %.2f V\n", myData.voltage);

        // --- CURRENT SENSOR (CALIBRATED LOGIC) ---
        long adcRawSum = 0;
        for(int i = 0; i < CURRENT_SENSOR_SAMPLES; i++) {
            adcRawSum += analogRead(CURRENT_SENSOR_PIN);
            delayMicroseconds(50);
        }
        float currentAvgADC = (float)adcRawSum / CURRENT_SENSOR_SAMPLES;
        float currentVoltageAtPin = (currentAvgADC / ESP32_ADC_MAX) * ESP_VREF;
        myData.current = -1.0 * (currentVoltageAtPin - V_PIN_ZERO) / REAL_SENSITIVITY;

        if (fabs(myData.current) < 0.03) myData.current = 0.00;

        Serial.printf("Current: %.2f A\n", myData.current);

        // --- Send via ESP-NOW ---
        myData.relayStatus = (digitalRead(RELAY_PIN) == LOW);  // LOW = Active/Triggered
        esp_err_t result = esp_now_send(centralNodeAddress, (uint8_t*)&myData, sizeof(myData));
        
        Serial.printf("Packet: %s | Relay: %s\n", 
                      (result == ESP_OK ? "Queued" : "Error"), 
                      (myData.relayStatus ? "ON" : "OFF"));
    }
}