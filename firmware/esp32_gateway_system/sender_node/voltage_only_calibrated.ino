/*
 * VOLTAGE SENSOR - CALIBRATED TEST ONLY
 * Tests the voltage sensor with calibrated DIVIDER_RATIO
 * Upload this to verify voltage readings are correct
 */

#define VOLTAGE_SENSOR_PIN 35
const float ESP_VREF = 3.3;
const float ESP32_ADC_MAX = 4095.0;

// --- CALIBRATION SETTINGS ---
// Linear for 0-6V range
const float DIVIDER_RATIO = 11.5;

// Interpolation table for 6V+ range (CALIBRATED from actual measurements)
// Format: { ADC_value, voltage_value }
float voltageCal[][2] = {
    { 1100, 9.0 },      // CALIBRATED
    { 1500, 11.86 },    // CALIBRATED
    { 1530, 12.0 },     // CALIBRATED
    { 1800, 14.0 },     // CALIBRATED
    { 2090, 16.0 },     // CALIBRATED
    { 2384, 18.0 },     // CALIBRATED
    { 2767, 20.0 },     // CALIBRATED
    { 3000, 22.0 }      // CALIBRATED
};
const int numCalPoints = sizeof(voltageCal) / sizeof(voltageCal[0]);

// Exponential smoothing to reduce noise
float smoothedVoltage = 0.0;
const float SMOOTHING_FACTOR = 0.8;  // 80% new, 20% old

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== VOLTAGE SENSOR - HYBRID CALIBRATION ===");
    Serial.println("Linear 0-6V | Interpolation 6V+\n");
    
    // Warm-up period: Initialize smoothedVoltage with actual reading to avoid startup spike
    Serial.println("Warming up voltage sensor...");
    for(int i = 0; i < 10; i++) {
        long voltageSum = 0;
        for(int j = 0; j < 200; j++) {
            voltageSum += analogRead(VOLTAGE_SENSOR_PIN);
            delayMicroseconds(50);
        }
        float voltageAvg = (float)voltageSum / 200.0;
        float outputVoltage = getVoltage(voltageAvg);
        smoothedVoltage = (outputVoltage * SMOOTHING_FACTOR) + (smoothedVoltage * (1.0 - SMOOTHING_FACTOR));
        delay(100);
    }
    Serial.println("Sensor ready!\n");
}

// Hybrid voltage calculation function
float getVoltage(float adc) {
    float pinVoltage = (adc / ESP32_ADC_MAX) * ESP_VREF;
    float linearOutput = pinVoltage * DIVIDER_RATIO;
    
    // Use linear for 0-6V
    if (linearOutput <= 6.0) {
        return linearOutput;
    }
    
    // Use interpolation for 6V+
    // Find surrounding calibration points
    for (int i = 0; i < numCalPoints - 1; i++) {
        if (adc >= voltageCal[i][0] && adc <= voltageCal[i+1][0]) {
            // Linear interpolation between two points
            float voltage = voltageCal[i][1] + 
                          (adc - voltageCal[i][0]) * 
                          (voltageCal[i+1][1] - voltageCal[i][1]) / 
                          (voltageCal[i+1][0] - voltageCal[i][0]);
            return voltage;
        }
    }
    
    // If above highest point, return highest value
    if (adc > voltageCal[numCalPoints-1][0]) {
        return voltageCal[numCalPoints-1][1];
    }
    
    return 0.0;
}

void loop() {
    // Take 200 samples (more samples = better averaging)
    long voltageSum = 0;
    for(int i = 0; i < 200; i++) {
        voltageSum += analogRead(VOLTAGE_SENSOR_PIN);
        delayMicroseconds(50);
    }
    
    float voltageAvg = (float)voltageSum / 200.0;
    
    // Calculate pin voltage
    float pinVoltage = (voltageAvg / ESP32_ADC_MAX) * ESP_VREF;
    
    // Get voltage using hybrid method (linear 0-6V, interpolation 6V+)
    float outputVoltage = getVoltage(voltageAvg);
    
    // Apply exponential smoothing to reduce fluctuations
    smoothedVoltage = (outputVoltage * SMOOTHING_FACTOR) + (smoothedVoltage * (1.0 - SMOOTHING_FACTOR));
    
    // Apply noise filter (< 0.5V = 0)
    if (smoothedVoltage < 0.5) {
        smoothedVoltage = 0.0;
    }
    
    // Apply voltage offset for 1-3V range
    if (smoothedVoltage >= 1.0 && smoothedVoltage <= 3.0) {
        smoothedVoltage += 0.6;
    }
    
    // Apply voltage offset for 3-5V range
    if (smoothedVoltage > 3.0 && smoothedVoltage <= 5.0) {
        smoothedVoltage -= 0.2;
    }
    
    // Display results
    Serial.println("===== VOLTAGE MEASUREMENT =====");
    Serial.printf("Raw ADC Value:        %.0f / 4095\n", voltageAvg);
    Serial.printf("Pin Voltage:          %.4f V\n", pinVoltage);
    Serial.printf("Output (Raw):         %.2f V\n", outputVoltage);
    Serial.printf("Output (Smoothed):    %.2f V\n", smoothedVoltage);
    Serial.println("================================\n");
    
    delay(2000);
}
