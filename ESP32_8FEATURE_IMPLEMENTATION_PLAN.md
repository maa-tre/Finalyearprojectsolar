# ESP32 8-Feature ML Model Implementation Plan

## 🎯 Overview
Convert the trained 8-feature Random Forest model to ESP32-compatible C/C++ format and integrate with existing firmware.

---

## 📋 Phase 1: Model Export to C/C++ (Days 1-2)

### Step 1.1: Generate model.h Header File
**Objective**: Convert trained model to C array format

**Files to Create/Modify**:
- `models/model_8feature.h` (NEW)
- `ml/step3_export_to_esp32.py` (UPDATE)

**Key Changes**:
```python
# Add 8-feature export logic
# Instead of 5 features: [V, I, T, L, Eff]
# Now export 8 features: [V, I, T, L, H, ThermT, Relay, Eff]

# Update feature means and stds for scaler in C:
FEATURE_MEANS = [14.44, 3.82, 44.94, 730.23, 51.48, 50.85, 0.41, 9.28]
FEATURE_STDS = [7.20, 2.73, 14.47, 288.37, 14.62, 18.70, 0.49, 7.75]

# Ensure model.h stays under ESP32 memory limits
# Target: < 100KB for model.h
```

**Output**:
```
✅ model_8feature.h (~95KB)
✅ Feature scaling constants embedded
✅ All 15 decision trees as C arrays
```

---

## 📋 Phase 2: Firmware Updates (Days 2-3)

### Step 2.1: Update ESP32 Gateway Firmware
**File**: `firmware/esp32_gateway_system/gateway_node.ino`

**Changes Required**:
1. **Add 3 new sensor inputs**:
```cpp
// OLD (5 sensors)
float humidity = analogRead(HUMIDITY_PIN) * 0.0976;
float thermistor_temp = readThermistor();
bool relay_status = digitalRead(RELAY_PIN);

// THESE ARE ALREADY BEING READ!
// Just need to include in model input
```

2. **Update prediction structure**:
```cpp
// OLD - 5 features
float features[5] = {voltage, current, temp, light, efficiency};

// NEW - 8 features
float features[8] = {
  voltage, 
  current, 
  temperature, 
  light_intensity,
  humidity,           // NEW
  thermistor_temp,    // NEW
  relay_status,       // NEW
  efficiency
};
```

3. **Update feature scaling**:
```cpp
// Apply StandardScaler with 8 coefficients
for(int i = 0; i < 8; i++) {
  features[i] = (features[i] - MEANS[i]) / STDS[i];
}
```

4. **Include new model.h**:
```cpp
#include "model_8feature.h"  // 8-feature model
```

### Step 2.2: Update Sender Node Firmware
**Files**: 
- `firmware/esp32_gateway_system/sender_node.ino`
- `firmware/esp32_gateway_system/sender_node_A.ino`

**Changes**:
```cpp
// Ensure all 8 features are being transmitted
// Create sensor data packet with 8 floats
sensor_data_t {
  float voltage;
  float current;
  float temperature;
  float light_intensity;
  float humidity;              // NEW
  float thermistor_temperature; // NEW
  uint8_t relay_status;        // NEW
  float efficiency;
};
```

### Step 2.3: Update Arduino Nano Firmware
**File**: `firmware/arduino_nano/arduino_nano_firmware.ino`

**Changes**:
```cpp
// If reading additional sensors, update ADC reads
// For humidity sensor (likely DHT22 or similar)
// For thermistor (NTC sensor)
// For relay status (GPIO digital read)

// Ensure data matches sender_node format
```

---

## 📋 Phase 3: Hardware & Sensor Verification (Days 3-4)

### Step 3.1: Verify Sensor Availability
**Checklist**:
- [ ] Humidity sensor connected (DHT22, BME280, or capacitive)
- [ ] Thermistor connected to ADC (NTC or PTC)
- [ ] Relay status GPIO connected to digital pin
- [ ] All sensors calibrated

### Step 3.2: Memory Optimization
**Constraints**:
- ESP32 RAM: 520KB (SRAM)
- Flash: 4MB (SPIFFS)
- Model.h: < 100KB
- Runtime variables: < 50KB

**Actions**:
```cpp
// Use PROGMEM for model arrays to save RAM
const uint8_t model_tree_data[] PROGMEM = { ... };

// Pre-allocate buffers
float scaled_features[8];
uint8_t prediction_probabilities[5];
```

---

## 📋 Phase 4: Testing (Days 4-5)

### Step 4.1: Simulation Testing
**Before deploying to hardware**:
```python
# Test step3_export_to_esp32.py output
# Verify C code compiles without errors
# Check model predictions match Python version
```

### Step 4.2: ESP32 Compilation
**Commands**:
```bash
# Arduino IDE or PlatformIO
platformio run -e esp32

# Check warnings and memory usage
# Should show: < 100KB for model.h
```

### Step 4.3: Real Hardware Testing
**On ESP32 Gateway Node**:
1. Flash updated firmware
2. Read 8 features from sensors
3. Run inference
4. Verify predictions match backend API
5. Log to SD card / SPIFFS

**Test Cases**:
- [ ] Normal operation: All features in normal range
- [ ] Partial Shading: Low light, high humidity
- [ ] Dust Accumulation: High temperature, low humidity
- [ ] Open Circuit: High voltage, ~0 current
- [ ] Short Circuit: Low voltage, high current, high humidity

### Step 4.4: Comparison Testing
```
ESP32 Prediction vs Backend Prediction for same inputs
Should match with 99%+ accuracy
```

---

## 📊 Implementation Timeline

| Phase | Task | Duration | Status |
|-------|------|----------|--------|
| 1 | Export model to C/C++ | 2 days | ⏳ Not Started |
| 2 | Update all firmware | 2 days | ⏳ Not Started |
| 3 | Hardware verification | 1.5 days | ⏳ Not Started |
| 4 | Testing & validation | 1.5 days | ⏳ Not Started |
| **Total** | **Complete ESP32 Integration** | **7 days** | |

---

## 🔧 Deliverables

### Code Changes:
1. ✅ `models/model_8feature.h` - C/C++ model export
2. ✅ `ml/step3_export_to_esp32.py` - Export script (UPDATE)
3. ✅ `firmware/esp32_gateway_system/gateway_node.ino` - 8-feature predictions
4. ✅ `firmware/esp32_gateway_system/sender_node.ino` - 8-feature transmission
5. ✅ `firmware/esp32_gateway_system/sender_node_A.ino` - 8-feature transmission
6. ✅ `firmware/arduino_nano/arduino_nano_firmware.ino` - Updated sensors

### Documentation:
1. `ESP32_8FEATURE_README.md` - Implementation guide
2. `SENSOR_CALIBRATION_GUIDE.md` - Hardware setup
3. `MODEL_MEMORY_ANALYSIS.md` - Performance metrics

### Testing:
1. Unit tests for C model inference
2. Integration tests with ESP32 board
3. Comparison tests (ESP32 vs Backend)

---

## ⚠️ Potential Issues & Mitigations

| Issue | Risk | Mitigation |
|-------|------|-----------|
| Model.h too large | 🔴 High | Optimize tree structure, use uint8_t for indices |
| Sensor read timing | 🟡 Medium | Use interrupts, optimize ADC sampling |
| Floating point precision | 🟡 Medium | Use float32, validate against Python |
| WiFi bandwidth | 🟢 Low | Compress data, batch transmissions |
| Humidity sensor not available | 🔴 High | Fall back to estimating humidity from temperature |

---

## 🚀 Next Steps

1. **Immediate (Today)**:
   - [ ] Review current `step3_export_to_esp32.py`
   - [ ] Check ESP32 model.h size constraints
   - [ ] Verify firmware compilation

2. **This Week**:
   - [ ] Export 8-feature model
   - [ ] Update gateway_node.ino
   - [ ] Update sender nodes

3. **Next Week**:
   - [ ] Hardware testing
   - [ ] Sensor calibration
   - [ ] Validation testing

---

## 📞 Decision Points

Before starting Phase 2, decide:

1. **Humidity Sensor**: Which sensor is installed?
   - [ ] DHT22 (temp + humidity)
   - [ ] BME280 (pressure + humidity + temp)
   - [ ] Capacitive sensor
   - [ ] Not installed - use estimation

2. **Thermistor**: Is one already connected?
   - [ ] Yes - which GPIO?
   - [ ] No - need to add

3. **Relay Status**: Is GPIO connected?
   - [ ] Yes - which GPIO?
   - [ ] No - need to identify

4. **Model Size**: Priority?
   - [ ] Accuracy (keep full 15 trees)
   - [ ] Size (compress to 5-7 trees)

---

## 📈 Expected Benefits

After implementation:
- ✅ **95.94% accuracy** on ESP32
- ✅ **Real-time** local inference (no WiFi needed)
- ✅ **More robust** predictions with 8 features
- ✅ **Better fault detection** for safety-critical cases
- ✅ **Reduced latency** vs cloud predictions

---

**Status**: 🟡 **READY FOR PLANNING**

Next: Choose Phase 2.1 to begin implementation!
