# Distributed Architecture: ESP32 ML Gateway + PC Backend + PC Frontend

## 🏗️ Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     SOLAR PANEL SYSTEM                          │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────────────────┐         ┌──────────────────────────┐
│                              │         │                          │
│  ESP32 GATEWAY NODE          │         │    PERSONAL COMPUTER     │
│  ────────────────────        │         │    ────────────────────  │
│                              │         │                          │
│  1. Read Sensors (8)         │         │  Backend API (FastAPI)   │
│     • Voltage                │         │  • Port 8000             │
│     • Current                │         │  • Receive from ESP32    │
│     • Temperature            │    WiFi │  • Send to Frontend      │
│     • Light Intensity        │◄────────►  • Store CSV logs       │
│     • Humidity               │         │                          │
│     • ThermistorTemp         │         │  Frontend (NextJS)       │
│     • RelayStatus            │         │  • Port 3000             │
│     • Efficiency             │         │  • Dashboard UI          │
│                              │         │  • Real-time charts      │
│  2. Scale Features           │         └──────────────────────────┘
│     (StandardScaler)         │
│                              │
│  3. Run ML Model             │
│     (8-feature RF)           │
│     • 95.94% accuracy        │
│     • 15 decision trees      │
│                              │
│  4. Generate Prediction      │
│     • Fault type             │
│     • Confidence score       │
│     • Recommendation         │
│                              │
│  5. Send to Backend (HTTP)   │
│     POST /api/predict        │
│                              │
└──────────────────────────────┘
```

---

## 📝 Files to Change & Modifications Required

### **1. ESP32 GATEWAY FIRMWARE** 🔴 MAJOR CHANGES

#### File: `firmware/esp32_gateway_system/gateway_node.ino`

**Current State**:
- Reads sensor data
- Sends raw data to backend API
- Backend does inference

**New State**:
- Reads sensor data ✅ (no change)
- **Runs ML model locally** (NEW)
- Sends prediction + raw data to backend
- Acts as "smart gateway"

**Changes Needed**:
```cpp
// 1. INCLUDE THE 8-FEATURE MODEL
#include "model_8feature.h"  // NEW - C++ model

// 2. FEATURE SCALING CONSTANTS (from Python model)
const float FEATURE_MEANS[8] = {14.44, 3.82, 44.94, 730.23, 51.48, 50.85, 0.41, 9.28};
const float FEATURE_STDS[8] = {7.20, 2.73, 14.47, 288.37, 14.62, 18.70, 0.49, 7.75};

// 3. INFERENCE FUNCTION (NEW)
int runMLPrediction(float features[8]) {
  // Scale features
  float scaled[8];
  for(int i = 0; i < 8; i++) {
    scaled[i] = (features[i] - FEATURE_MEANS[i]) / FEATURE_STDS[i];
  }
  
  // Run model and return prediction
  return predictFault(scaled);
}

// 4. DATA STRUCTURE UPDATE (8 features instead of 5)
struct SensorData {
  float voltage;
  float current;
  float temperature;
  float light_intensity;
  float humidity;              // NEW
  float thermistor_temp;       // NEW
  uint8_t relay_status;        // NEW
  float efficiency;
  
  // Add prediction results
  uint8_t fault_type;          // NEW - 0-4
  float confidence;            // NEW - 0-100
};

// 5. SEND TO BACKEND WITH PREDICTION (NEW)
void sendPredictionToBackend(SensorData data) {
  // Instead of sending just raw data,
  // send: raw data + local prediction
  
  String payload = "{";
  payload += "\"voltage\":" + String(data.voltage) + ",";
  payload += "\"current\":" + String(data.current) + ",";
  // ... all 8 features ...
  payload += "\"local_prediction\":" + String(data.fault_type) + ",";
  payload += "\"confidence\":" + String(data.confidence);
  payload += "}";
  
  // POST to backend
  postToBackend("/api/gateway/predict", payload);
}
```

---

### **2. BACKEND API** 🟡 MEDIUM CHANGES

#### File: `backend/main.py`

**Current State**:
- Receives raw sensor data → runs ML → sends prediction to frontend

**New State**:
- Receives prediction from ESP32 (as source of truth) → validates → stores → sends to frontend
- Falls back to local ML if ESP32 offline
- Logs both ESP32 and backend predictions

**Changes Needed**:

```python
# 1. NEW ENDPOINT FOR ESP32 PREDICTIONS
@app.post("/api/gateway/predict")
async def receive_gateway_prediction(data: dict):
    """
    Receive prediction from ESP32 gateway
    ESP32 has already run ML model locally
    """
    fault_type = data['local_prediction']  # Trust ESP32
    confidence = data['confidence']
    
    # Store to CSV with SOURCE = 'ESP32'
    log_to_csv(data, source='ESP32')
    
    # Broadcast to frontend via WebSocket
    await broadcast_to_clients({
        'source': 'ESP32_GATEWAY',
        'fault_type': fault_type,
        'confidence': confidence,
        'timestamp': datetime.now()
    })
    
    return {'status': 'received', 'fault_type': fault_type}

# 2. FALLBACK ENDPOINT (if ESP32 offline)
@app.post("/api/predict")
async def predict_fault_fallback(data: SensorData):
    """
    Fallback: PC-based ML prediction
    Used if ESP32 is offline
    """
    # Run local model
    prediction = predict_fault_local(data)
    
    # Store with SOURCE = 'PC_BACKEND'
    log_to_csv(data, source='PC_BACKEND')
    
    return prediction

# 3. ADD GATEWAY STATUS ENDPOINT
@app.get("/api/gateway/status")
async def gateway_status():
    """
    Check if ESP32 gateway is online
    """
    return {
        'gateway_online': is_gateway_online(),
        'last_prediction_time': last_esp32_prediction_time,
        'predictions_received': total_predictions_from_esp32
    }

# 4. ADD DUAL LOGGING FOR VALIDATION
def log_to_csv(data, source='ESP32', prediction=None):
    """
    Log all sensor data + source information
    Use for comparing ESP32 vs PC predictions
    """
    row = {
        'timestamp': datetime.now(),
        'source': source,  # 'ESP32_GATEWAY' or 'PC_BACKEND'
        'voltage': data.voltage,
        'current': data.current,
        # ... all 8 features ...
        'fault_type': prediction.fault_type if prediction else None,
        'confidence': prediction.confidence if prediction else None
    }
    write_to_csv(row)
```

---

### **3. FRONTEND** 🟢 MINIMAL CHANGES

#### File: `frontend/src/app/page.tsx`

**Current State**:
- Connects to PC backend on port 8000
- Receives predictions from backend

**New State**:
- Connects to PC backend on port 8000 (NO CHANGE)
- Now shows "SOURCE: ESP32" when prediction is from gateway
- Shows "SOURCE: PC_BACKEND" when fallback is used
- Add gateway status indicator

**Changes Needed**:

```typescript
// 1. ADD GATEWAY STATUS INDICATOR
const [gatewayOnline, setGatewayOnline] = useState(false);

// 2. FETCH GATEWAY STATUS
useEffect(() => {
  const checkGateway = async () => {
    const response = await fetch('http://localhost:8000/api/gateway/status');
    const data = await response.json();
    setGatewayOnline(data.gateway_online);
  };
  
  checkGateway();
  const interval = setInterval(checkGateway, 5000); // Check every 5s
  return () => clearInterval(interval);
}, []);

// 3. DISPLAY SOURCE IN UI
<div className="status-badge">
  Source: {prediction.source === 'ESP32_GATEWAY' ? 
    <span className="green">🟢 ESP32 Gateway</span> : 
    <span className="yellow">🟡 PC Backend (Fallback)</span>
  }
</div>

// 4. GATEWAY STATUS INDICATOR
<div className="gateway-status">
  {gatewayOnline ? 
    <span className="green">✓ Gateway Online</span> :
    <span className="red">✗ Gateway Offline - Using PC Model</span>
  }
</div>
```

---

### **4. SENDER NODE FIRMWARE** 🟡 MEDIUM CHANGES

#### File: `firmware/esp32_gateway_system/sender_node.ino`

**Current State**:
- Reads sensors from solar panel
- Sends raw data to gateway via radio/serial

**New State**:
- Same (no change needed if using radio/serial)

**OR if directly sending to backend**:
```cpp
// If sender node connects directly to WiFi:
// Send all 8 features instead of 5

struct RadioPacket {
  float voltage;
  float current;
  float temperature;
  float light_intensity;
  float humidity;           // NEW
  float thermistor_temp;    // NEW
  uint8_t relay_status;     // NEW
  float efficiency;
};
```

---

### **5. ARDUINO NANO FIRMWARE** 🟢 MINIMAL CHANGES

#### File: `firmware/arduino_nano/arduino_nano_firmware.ino`

**Current State**:
- Reads analog sensors

**New State**:
- Add humidity sensor reading (if not already done)
- Add thermistor reading (if not already done)

```cpp
// NEW - if not already reading these
float readHumidity() {
  // DHT22, BME280, or capacitive sensor
  return humidity_value;
}

float readThermistor() {
  // NTC thermistor via voltage divider
  return temperature_celsius;
}

uint8_t getRelayStatus() {
  return digitalRead(RELAY_PIN);
}
```

---

### **6. MODEL EXPORT SCRIPT** 🟡 MEDIUM CHANGES

#### File: `ml/step3_export_to_esp32.py`

**Current State**:
- Exports 5-feature model to C header

**New State**:
- Export 8-feature model to C header
- Include all 8 feature means/stds
- Ensure model.h stays < 100KB

```python
# Changes needed:
# 1. Update feature list
FEATURES = ['Voltage', 'Current', 'Temperature', 'Light_Intensity',
            'Humidity', 'ThermistorTemp', 'RelayStatus', 'Efficiency']

# 2. Generate with 8 means/stds
feature_means = scaler.mean_  # Should be 8 values
feature_stds = scaler.scale_  # Should be 8 values

# 3. Output code
output_h = generate_model_header(
  model=trained_model,
  feature_means=feature_means,
  feature_stds=feature_stds,
  output_file='model_8feature.h'
)
```

---

### **7. CONFIGURATION & CONSTANTS** 🟡 MEDIUM CHANGES

#### File: `backend/.env` (or config file)

**New/Changed Variables**:
```env
# ESP32 Gateway configuration
ESP32_GATEWAY_IP=192.168.x.x
ESP32_GATEWAY_PORT=80
ESP32_GATEWAY_TIMEOUT=5000  # ms

# Prediction source strategy
PREFER_ESP32_PREDICTION=true  # Use ESP32 as source of truth
FALLBACK_TO_PC_MODEL=true    # Use PC model if ESP32 offline

# Logging
LOG_PREDICTION_SOURCE=true   # Log where prediction came from
COMPARE_PREDICTIONS=true     # Log ESP32 vs PC predictions
```

---

## 🔀 Communication Flow (NEW vs OLD)

### OLD ARCHITECTURE
```
Sensors → ESP32 → WiFi → Backend API (PC) → ML Model → Frontend
         (reads)        (predicts)
```

### NEW ARCHITECTURE
```
Sensors → ESP32 → ML Model → Backend API (PC) → Frontend
         (reads)  (predicts)  (stores)        (displays)
         (8 feat)             (logs source)   (shows "ESP32")
```

---

## 📊 Summary Table: Files That Change

| File | Type | Changes | Priority |
|------|------|---------|----------|
| `firmware/esp32_gateway_system/gateway_node.ino` | 🔴 Major | Add ML model, run inference locally, send prediction | 🔴 Critical |
| `backend/main.py` | 🟡 Medium | Add `/api/gateway/predict` endpoint, fallback logic, source tracking | 🔴 Critical |
| `frontend/src/app/page.tsx` | 🟢 Minor | Add gateway status indicator, show prediction source | 🟡 Important |
| `firmware/esp32_gateway_system/sender_node.ino` | 🟡 Medium | Update for 8 features (if applicable) | 🟡 Important |
| `firmware/arduino_nano/arduino_nano_firmware.ino` | 🟢 Minor | Ensure all 8 sensors read | 🟢 Optional |
| `ml/step3_export_to_esp32.py` | 🟡 Medium | Export 8-feature model to C header | 🔴 Critical |
| `.env` / config | 🟢 Minor | ESP32 gateway IP, fallback settings | 🟡 Important |

---

## 🔗 Sequence Diagram

```
┌──────────┐         ┌──────────┐         ┌──────────┐
│ ESP32    │         │ Backend  │         │ Frontend │
│ Gateway  │         │ on PC    │         │ on PC    │
└────┬─────┘         └────┬─────┘         └────┬─────┘
     │                    │                    │
     │ 1. Read 8 sensors  │                    │
     ├──────────┐         │                    │
     │ 2. Scale │         │                    │
     │ 3. Run ML│         │                    │
     │ 4. Get prediction          │                    │
     │                    │                    │
     │ 5. POST /api/gateway/predict (prediction + data)
     ├───────────────────>│                    │
     │                    │ 6. Validate data          │
     │                    │ 7. Store to CSV           │
     │                    │ 8. Broadcast to clients
     │                    ├───────────────────>│
     │                    │                    │ 9. Display
     │                    │                    │    "Source: ESP32"
     │                    │                    │
     │ (repeat every 2s)  │                    │
     │                    │                    │
```

---

## ✅ Implementation Checklist

- [ ] Generate `model_8feature.h` from trained model
- [ ] Add model to `gateway_node.ino`
- [ ] Add feature scaling to `gateway_node.ino`
- [ ] Add inference function to `gateway_node.ino`
- [ ] Test ESP32 local predictions (serial output)
- [ ] Add `/api/gateway/predict` endpoint to backend
- [ ] Add gateway status endpoint to backend
- [ ] Update frontend to show gateway status
- [ ] Update frontend to show prediction source
- [ ] Test end-to-end: ESP32 → Backend → Frontend
- [ ] Add fallback logic if ESP32 offline
- [ ] Implement CSV logging with source tracking

---

## 🚀 Next: Ready for Implementation?

Which file would you like to start with?
1. **gateway_node.ino** - Add ML model
2. **backend/main.py** - Add gateway endpoint
3. **step3_export_to_esp32.py** - Generate model.h
