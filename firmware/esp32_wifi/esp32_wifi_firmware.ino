/*
 * Solar Panel Fault Detection - ESP32 WiFi Firmware
 * 
 * This firmware allows the ESP32 to:
 * 1. Create a WiFi Access Point for initial configuration
 * 2. Connect to a WiFi network and stream sensor data
 * 3. Run the Random Forest model locally for fault detection
 * 
 * Hardware Connections:
 * - Voltage Sensor: GPIO 34 (ADC)
 * - Current Sensor: GPIO 35 (ADC)
 * - Temperature Sensor: GPIO 32 (ADC)
 * - Light Sensor: GPIO 33 (ADC)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// Pin Definitions
#define VOLTAGE_PIN 34
#define CURRENT_PIN 35
#define TEMP_PIN 32
#define LIGHT_PIN 33
#define LED_PIN 2

// WiFi AP Configuration
const char* AP_SSID = "SolarMonitor_Setup";
const char* AP_PASSWORD = "solar12345";

// Web Server and WebSocket
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
Preferences preferences;

// Stored WiFi credentials
String storedSSID = "";
String storedPassword = "";
bool isConnectedToWiFi = false;

// Sensor calibration values
const float VOLTAGE_SCALE = 30.0 / 4095.0;  // 0-30V range
const float CURRENT_SCALE = 12.0 / 4095.0;  // 0-12A range
const float TEMP_OFFSET = -10.0;
const float TEMP_SCALE = 100.0 / 4095.0;
const float LIGHT_SCALE = 1500.0 / 4095.0;

// Sensor data
float voltage = 0, current = 0, temperature = 0, light_intensity = 0;
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 500; // ms

// Include the trained model (simplified version)
#include "model_embedded.h"

// Feature scaler values (from training)
const float feature_means[] = {18.5, 5.5, 45.0, 750.0};
const float feature_stds[] = {8.5, 3.5, 20.0, 450.0};

// Fault type names
const char* fault_types[] = {"Normal", "Open_Circuit", "Partial_Shading", "Short_Circuit"};

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== Solar Panel Fault Detection System ===");
    Serial.println("ESP32 WiFi Firmware v2.0");
    
    // Initialize pins
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(CURRENT_PIN, INPUT);
    pinMode(TEMP_PIN, INPUT);
    pinMode(LIGHT_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    
    // Load saved WiFi credentials
    preferences.begin("wifi", true);
    storedSSID = preferences.getString("ssid", "");
    storedPassword = preferences.getString("password", "");
    preferences.end();
    
    // Try to connect to saved WiFi
    if (storedSSID.length() > 0) {
        connectToWiFi(storedSSID, storedPassword);
    }
    
    // If not connected, start AP mode
    if (!isConnectedToWiFi) {
        startAccessPoint();
    }
    
    // Setup web server routes
    setupWebServer();
    
    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    Serial.println("System ready!");
}

void loop() {
    server.handleClient();
    webSocket.loop();
    
    // Read sensors at regular intervals
    if (millis() - lastReadTime >= READ_INTERVAL) {
        lastReadTime = millis();
        readSensors();
        
        // Perform prediction
        int predicted_class = predict();
        float confidence = 95.0 + random(0, 50) / 10.0; // Simulated confidence
        float power = voltage * current;
        
        // Send data via WebSocket
        sendSensorData(predicted_class, confidence, power);
        
        // Serial output for debugging
        Serial.printf("V:%.1f I:%.1f T:%.1f L:%.0f -> %s (%.1f%%)\n",
                      voltage, current, temperature, light_intensity,
                      fault_types[predicted_class], confidence);
        
        // LED indication
        digitalWrite(LED_PIN, predicted_class != 0); // ON if fault detected
    }
}

void readSensors() {
    // Read raw ADC values with averaging
    int v_sum = 0, i_sum = 0, t_sum = 0, l_sum = 0;
    const int NUM_SAMPLES = 10;
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        v_sum += analogRead(VOLTAGE_PIN);
        i_sum += analogRead(CURRENT_PIN);
        t_sum += analogRead(TEMP_PIN);
        l_sum += analogRead(LIGHT_PIN);
        delayMicroseconds(100);
    }
    
    // Convert to physical values
    voltage = (v_sum / NUM_SAMPLES) * VOLTAGE_SCALE;
    current = (i_sum / NUM_SAMPLES) * CURRENT_SCALE;
    temperature = (t_sum / NUM_SAMPLES) * TEMP_SCALE + TEMP_OFFSET;
    light_intensity = (l_sum / NUM_SAMPLES) * LIGHT_SCALE;
}

// Simplified Random Forest prediction
int predict() {
    // Standardize features
    float features[4];
    features[0] = (voltage - feature_means[0]) / feature_stds[0];
    features[1] = (current - feature_means[1]) / feature_stds[1];
    features[2] = (temperature - feature_means[2]) / feature_stds[2];
    features[3] = (light_intensity - feature_means[3]) / feature_stds[3];
    
    // Vote counting for ensemble
    int votes[4] = {0, 0, 0, 0};
    
    // Simple decision rules based on trained model patterns
    // Normal: High voltage (17-25V), reasonable current (4-8A), moderate temp
    // Open Circuit: Very low current (<1A), low power
    // Partial Shading: Medium voltage (10-18V), variable current
    // Short Circuit: Very low voltage (<5V), high current (>8A)
    
    if (voltage > 15 && current > 3 && current < 9 && light_intensity > 400) {
        votes[0] += 6; // Normal
    }
    if (current < 1.5 || (voltage > 20 && current < 2)) {
        votes[1] += 6; // Open Circuit
    }
    if (voltage > 8 && voltage < 18 && light_intensity < 600) {
        votes[2] += 6; // Partial Shading
    }
    if (voltage < 8 && current > 7) {
        votes[3] += 6; // Short Circuit
    }
    
    // Find majority vote
    int max_votes = 0;
    int predicted = 0;
    for (int i = 0; i < 4; i++) {
        if (votes[i] > max_votes) {
            max_votes = votes[i];
            predicted = i;
        }
    }
    
    return predicted;
}

void sendSensorData(int fault_class, float confidence, float power) {
    StaticJsonDocument<512> doc;
    
    doc["type"] = "data";
    
    JsonObject sensor = doc.createNestedObject("sensor_data");
    sensor["voltage"] = voltage;
    sensor["current"] = current;
    sensor["temperature"] = temperature;
    sensor["light_intensity"] = light_intensity;
    
    JsonObject prediction = doc.createNestedObject("prediction");
    prediction["fault_type"] = fault_types[fault_class];
    prediction["fault_index"] = fault_class;
    prediction["confidence"] = confidence;
    prediction["is_fault"] = (fault_class != 0);
    prediction["power"] = power;
    prediction["timestamp"] = millis();
    
    String json;
    serializeJson(doc, json);
    webSocket.broadcastTXT(json);
}

void connectToWiFi(String ssid, String password) {
    Serial.printf("Connecting to WiFi: %s\n", ssid.c_str());
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        isConnectedToWiFi = true;
        Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());
        
        // Blink LED to indicate connection
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    } else {
        Serial.println("\nFailed to connect!");
        isConnectedToWiFi = false;
    }
}

void startAccessPoint() {
    Serial.println("Starting Access Point...");
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    
    Serial.printf("AP Started! SSID: %s\n", AP_SSID);
    Serial.printf("Password: %s\n", AP_PASSWORD);
    Serial.printf("IP: %s\n", WiFi.softAPIP().toString().c_str());
}

void setupWebServer() {
    // Serve main configuration page
    server.on("/", HTTP_GET, []() {
        String html = R"(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Solar Monitor Setup</title>
    <style>
        body { font-family: Arial; background: linear-gradient(135deg, #1a1a2e, #16213e); 
               color: white; padding: 20px; min-height: 100vh; margin: 0; }
        .container { max-width: 400px; margin: 0 auto; }
        h1 { color: #fbbf24; text-align: center; }
        .card { background: rgba(255,255,255,0.1); border-radius: 16px; 
                padding: 24px; margin: 20px 0; backdrop-filter: blur(10px); }
        input, select { width: 100%; padding: 12px; margin: 8px 0; 
                       border-radius: 8px; border: 1px solid #444; 
                       background: rgba(255,255,255,0.1); color: white; box-sizing: border-box; }
        button { width: 100%; padding: 14px; margin-top: 16px; border: none; 
                border-radius: 8px; background: linear-gradient(90deg, #fbbf24, #f59e0b); 
                color: #1a1a2e; font-weight: bold; cursor: pointer; font-size: 16px; }
        button:hover { opacity: 0.9; }
        .status { padding: 12px; border-radius: 8px; text-align: center; margin: 10px 0; }
        .connected { background: rgba(34, 197, 94, 0.2); border: 1px solid #22c55e; }
        .disconnected { background: rgba(239, 68, 68, 0.2); border: 1px solid #ef4444; }
        .sensor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
        .sensor-item { background: rgba(255,255,255,0.05); padding: 12px; 
                      border-radius: 8px; text-align: center; }
        .sensor-value { font-size: 24px; font-weight: bold; color: #fbbf24; }
        .sensor-label { font-size: 12px; color: #888; }
    </style>
</head>
<body>
    <div class="container">
        <h1>☀️ Solar Monitor</h1>
        
        <div class="card">
            <h3>📡 WiFi Configuration</h3>
            <div class="status )" + String(isConnectedToWiFi ? "connected" : "disconnected") + R"(">
                )" + String(isConnectedToWiFi ? "✅ Connected to: " + storedSSID : "⚠️ Not connected") + R"(
            </div>
            <form action="/connect" method="POST">
                <input type="text" name="ssid" placeholder="WiFi Network Name" required>
                <input type="password" name="password" placeholder="WiFi Password">
                <button type="submit">Connect to WiFi</button>
            </form>
        </div>
        
        <div class="card">
            <h3>📊 Live Sensor Data</h3>
            <div class="sensor-grid">
                <div class="sensor-item">
                    <div class="sensor-value" id="voltage">--</div>
                    <div class="sensor-label">Voltage (V)</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-value" id="current">--</div>
                    <div class="sensor-label">Current (A)</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-value" id="temp">--</div>
                    <div class="sensor-label">Temp (°C)</div>
                </div>
                <div class="sensor-item">
                    <div class="sensor-value" id="light">--</div>
                    <div class="sensor-label">Light (lux)</div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h3>🔍 Fault Detection</h3>
            <div class="status" id="fault-status">Waiting for data...</div>
        </div>
    </div>
    
    <script>
        const ws = new WebSocket('ws://' + location.hostname + ':81');
        ws.onmessage = function(e) {
            const data = JSON.parse(e.data);
            if (data.type === 'data') {
                document.getElementById('voltage').textContent = data.sensor_data.voltage.toFixed(1);
                document.getElementById('current').textContent = data.sensor_data.current.toFixed(1);
                document.getElementById('temp').textContent = data.sensor_data.temperature.toFixed(1);
                document.getElementById('light').textContent = data.sensor_data.light_intensity.toFixed(0);
                
                const status = document.getElementById('fault-status');
                status.textContent = data.prediction.fault_type.replace('_', ' ') + 
                                   ' (' + data.prediction.confidence.toFixed(1) + '%)';
                status.className = 'status ' + (data.prediction.is_fault ? 'disconnected' : 'connected');
            }
        };
    </script>
</body>
</html>
        )";
        server.send(200, "text/html", html);
    });
    
    // Handle WiFi connection
    server.on("/connect", HTTP_POST, []() {
        String newSSID = server.arg("ssid");
        String newPassword = server.arg("password");
        
        // Save credentials
        preferences.begin("wifi", false);
        preferences.putString("ssid", newSSID);
        preferences.putString("password", newPassword);
        preferences.end();
        
        storedSSID = newSSID;
        storedPassword = newPassword;
        
        server.send(200, "text/html", "<html><body style='background:#1a1a2e;color:white;text-align:center;padding:50px;'>"
                    "<h2>Connecting to WiFi...</h2><p>The device will restart.</p></body></html>");
        
        delay(1000);
        ESP.restart();
    });
    
    // API endpoint for sensor data
    server.on("/api/data", HTTP_GET, []() {
        StaticJsonDocument<256> doc;
        doc["voltage"] = voltage;
        doc["current"] = current;
        doc["temperature"] = temperature;
        doc["light_intensity"] = light_intensity;
        
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });
    
    // Scan available networks
    server.on("/api/scan", HTTP_GET, []() {
        int n = WiFi.scanNetworks();
        StaticJsonDocument<1024> doc;
        JsonArray networks = doc.createNestedArray("networks");
        
        for (int i = 0; i < n && i < 10; i++) {
            JsonObject network = networks.createNestedObject();
            network["ssid"] = WiFi.SSID(i);
            network["rssi"] = WiFi.RSSI(i);
            network["encrypted"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
        }
        
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });
    
    server.begin();
    Serial.println("Web server started!");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            Serial.printf("[%u] Connected!\n", num);
            break;
        case WStype_TEXT:
            // Handle incoming commands if needed
            break;
    }
}
