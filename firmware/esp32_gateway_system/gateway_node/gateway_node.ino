/*
 * Solar Panel Fault Detection - Gateway Node Firmware
 * 
 * ROLE: Receives data from multiple Sender Nodes via ESP-NOW and forwards it to the Backend via WiFi.
 * HARDWARE: ESP32 (No sensors required)
 * 
 * Based on: updatereciever.ino from myproject
 */

#include <esp_now.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <map>
#include <vector>
#include <ArduinoJson.h>
#include <esp_wifi.h> 

// --- Wi-Fi Credentials ---
// Updating to your new network while keeping the "OLD/NEW" comment style if preferred
const char* ssid     = "Acerhotspot";
const char* password = "123456780";
//const char* ssid     = "NCE Student";      // NEW Updated WiFi name
//const char* password = "welcome@nce";      // UPDATED

// --- Discovery Configuration (For Scanning Method) ---
// This starts an AP so Senders can find the Gateway's channel
const char* GATEWAY_SOFTAP_SSID = "Solar_Panel_Gateway"; 

// --- Fixed Wi-Fi Channel ---
// Must match the channel of the Sender nodes when not scanning
const uint8_t FIXED_CHANNEL = 1; 

// --- Backend Server URL ---
// Using your NEW Flask IP
const char* flaskServerUrl = "http://192.168.1.69:8000/api/gateway-data";

// --- MAC Addresses of Sender Devices ---
// UPDATE THESE MAC ADDRESSES IF HARDWARE CHANGES
uint8_t sender1_mac[] = {0x5C, 0x01, 0x3B, 0x4C, 0xD3, 0x18}; 
uint8_t sender2_mac[] = {0x10, 0x52, 0x1C, 0xA7, 0x54, 0x08}; 

// --- Data Structures ---
// Must match Sender's structure exactly
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

typedef struct struct_command {
    char command[32]; 
} struct_command;

struct SenderData {
    struct_message data;
    unsigned long lastReceivedTimestamp;
};

// Map: senderId -> latest data
std::map<int, SenderData> incomingDataMap;

// Timers
unsigned long lastFlaskSendTime = 0;
unsigned long flaskSendInterval = 2000;      // 2 seconds
unsigned long senderTimeoutInterval = 25000; // 25 seconds for stale data

// --- ESP-NOW Callbacks ---
void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t* incomingDataPtr, int len) {
    struct_message tempIncomingData;
    memcpy(&tempIncomingData, incomingDataPtr, sizeof(tempIncomingData));

    incomingDataMap[tempIncomingData.senderId].data = tempIncomingData;
    incomingDataMap[tempIncomingData.senderId].lastReceivedTimestamp = millis();

    Serial.printf(
        "Data received from Sender ID %d | MAC: %02X:%02X:%02X:%02X:%02X:%02X\n"
        "  - V: %.2f V, I: %.3f A, T: %.2f C\n",
        tempIncomingData.senderId,
        recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
        recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5],
        tempIncomingData.voltage, tempIncomingData.current, tempIncomingData.thermistorTemp
    );
}

void OnDataSent(const esp_now_send_info_t* send_info, esp_now_send_status_t status) {
    // Optional: Log send status if sending commands back
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("Command delivery Success");
    } else {
        Serial.println("Command delivery Fail");
    }
}

// --- Send Data to Backend ---
void sendAggregatedDataToFlask() {
    if (WiFi.status() == WL_CONNECTED) {
        if (incomingDataMap.empty()) {
            return;
        }

        HTTPClient http;
        http.begin(flaskServerUrl);
        http.addHeader("Content-Type", "application/json");

        // Prepare JSON Payload
        // Using DynamicJsonDocument for compatibility
        const int capacity = JSON_ARRAY_SIZE(incomingDataMap.size()) + incomingDataMap.size() * JSON_OBJECT_SIZE(10);
        DynamicJsonDocument jsonDoc(capacity);
        JsonArray records = jsonDoc.to<JsonArray>();

        std::vector<int> sendersToRemove;

        for (auto const& [senderId, senderData] : incomingDataMap) {
            // Check for stale data
            if (millis() - senderData.lastReceivedTimestamp < senderTimeoutInterval) {
                JsonObject record = records.createNestedObject();
                record["senderId"]           = senderData.data.senderId;
                record["ldrValue"]           = senderData.data.ldrValue;
                record["dhtTemp"]            = senderData.data.dhtTemp;
                record["humidity"]           = senderData.data.humidity;
                record["thermistorTemp"]     = senderData.data.thermistorTemp;
                record["voltage"]            = senderData.data.voltage;
                record["current"]            = senderData.data.current; 
                record["relayStatus"]        = senderData.data.relayStatus;
                record["valid"]              = senderData.data.valid;
                record["gateway_timestamp_ms"] = millis();
            } else {
                Serial.printf("Data from sender ID %d is stale.\n", senderId);
                sendersToRemove.push_back(senderId);
            }
        }
        
        // Safely remove elements after iteration
        for (int id : sendersToRemove) {
            incomingDataMap.erase(id);
        }

        if (records.size() == 0) {
            http.end();
            return;
        }

        String jsonPayload;
        serializeJson(jsonDoc, jsonPayload);

        Serial.print("Sending to Backend: ");
        Serial.println(jsonPayload);

        int httpResponseCode = http.POST(jsonPayload);

        if (httpResponseCode > 0) {
            Serial.printf("HTTP Response code: %d\n", httpResponseCode);
            if (httpResponseCode == 200) {
                 // Serial.println(http.getString());
            }
        } else {
            Serial.printf("Error code: %s\n", http.errorToString(httpResponseCode).c_str());
        }

        http.end();
    } else {
        Serial.println("WiFi Disconnected");
    }
}

// --- Send Command to a Specific Sender ---
void sendCommandToSender(int senderId, const char* command) {
    struct_command cmd_to_send;
    strncpy(cmd_to_send.command, command, sizeof(cmd_to_send.command) - 1);
    cmd_to_send.command[sizeof(cmd_to_send.command) - 1] = '\0'; // Ensure null termination

    uint8_t* target_mac = nullptr;
    if (senderId == 1) {
        target_mac = sender1_mac;
    } else if (senderId == 2) {
        target_mac = sender2_mac;
    } else {
        Serial.printf("No MAC address registered for sender ID %d\n", senderId);
        return;
    }

    esp_err_t result = esp_now_send(target_mac, (uint8_t *) &cmd_to_send, sizeof(cmd_to_send));
   
    if (result == ESP_OK) {
        Serial.printf("Command '%s' sent to sender %d successfully.\n", command, senderId);
    } else {
        Serial.printf("Error sending command '%s' to sender %d.\n", command, senderId);
    }
}

// --- Poll Server for Commands for a Specific Station ---
void pollForCommandsForStation(int stationId) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        
        // Build the URL for this specific station
        String urlString = String(flaskServerUrl); 
        urlString.replace("gateway-data", "get-command/");
        urlString += String(stationId);
        
        Serial.printf("[POLL] Station %d: %s\n", stationId, urlString.c_str());
        http.begin(urlString);
        http.setTimeout(3000);
        int httpResponseCode = http.GET();

        if (httpResponseCode == 200) {
            String payload = http.getString();
            Serial.printf("[COMMAND] Station %d received: %s\n", stationId, payload.c_str());

            DynamicJsonDocument doc(256);
            DeserializationError error = deserializeJson(doc, payload);

            if (error) {
                Serial.print("[ERROR] deserializeJson() failed: ");
                Serial.println(error.c_str());
                http.end();
                return;
            }

            // API returns { "station_id": 1, "command": "CMD" }
            int res_id = doc["station_id"];
            const char* command = doc["command"];
            
            if (res_id > 0 && command) {
                if (res_id == stationId) {
                    Serial.printf("[SEND] Sending '%s' to sender %d via ESP-NOW\n", command, res_id);
                    sendCommandToSender(res_id, command);
                }
            }

        } else if (httpResponseCode == 204) {
              // No Content - Normal (no command queued)
        } else {
            Serial.printf("[ERROR] HTTP GET failed code: %d\n", httpResponseCode);
        }
        http.end();
    } else {
        Serial.println("[ERROR] WiFi Disconnected");
    }
}

// --- Poll Server for Commands for All Stations ---
void pollForCommands() {
    pollForCommandsForStation(1);
    delay(100);
    pollForCommandsForStation(2);
}

// Timers for Polling
unsigned long lastCommandPollTime = 0;
const unsigned long commandPollInterval = 5000; // 5s

// --- Setup ---
void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("\n--- ESP32 Gateway Node Starting ---");

    // CRITICAL for Scanning Method: Must be AP_STA mode
    WiFi.mode(WIFI_AP_STA);
    
    // Start SoftAP so Senders can find the Gateway's current channel
    WiFi.softAP(GATEWAY_SOFTAP_SSID);
    Serial.printf("Discovery AP Started: %s\n", GATEWAY_SOFTAP_SSID);

    // Connect to router
    Serial.printf("Connecting to %s...", ssid);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected.");
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
    Serial.print("Gateway MAC (STA): "); Serial.println(WiFi.macAddress());
    
    // Debug Print actual WiFi channel assigned by router
    Serial.printf("[DEBUG] Running on Channel: %d\n", WiFi.channel());

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    Serial.println("[DEBUG] ESP-NOW Initialized Successfully");

    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
    Serial.println("[DEBUG] ESP-NOW Callbacks Registered");

    // Register Sender Peers
    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0; // 0 means follow system channel
    peerInfo.encrypt = false;
    
    memcpy(peerInfo.peer_addr, sender1_mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) Serial.println("Failed to add sender 1");
    else Serial.println("[DEBUG] Sender 1 (Station 1) Added");

    memcpy(peerInfo.peer_addr, sender2_mac, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) Serial.println("Failed to add sender 2");
    else Serial.println("[DEBUG] Sender 2 (Station 2) Added");

    Serial.println("\n=== Gateway Ready. Waiting for data ===");
}

void loop() {
    // Send data to backend periodically
    if (millis() - lastFlaskSendTime >= flaskSendInterval) {
        sendAggregatedDataToFlask();
        lastFlaskSendTime = millis();
    }

    // Poll for commands
    if (millis() - lastCommandPollTime >= commandPollInterval) {
        pollForCommands();
        lastCommandPollTime = millis();
    }
}
