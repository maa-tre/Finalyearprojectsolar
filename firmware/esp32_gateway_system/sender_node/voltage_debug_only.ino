/*
 * VOLTAGE SENSOR DEBUGGING SKETCH
 * Only measures voltage to diagnose sensor issues
 * Upload this to ESP32 to test voltage measurements
 */

#define VOLTAGE_SENSOR_PIN 35
const float ESP_VREF = 3.3;
const float ESP32_ADC_MAX = 4095.0;

// Test different divider ratios
const float DIVIDER_RATIO_2 = 2.0;    // Two equal resistors
const float DIVIDER_RATIO_9 = 9.14;   // Your suspected ratio
const float DIVIDER_RATIO_10 = 10.0;  // Standard 10:1 divider

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== VOLTAGE SENSOR DEBUG ONLY ===");
    Serial.println("Reading PIN 35 continuously...\n");
}

void loop() {
    // Take 100 samples FAST (no delay between samples)
    long voltageSum_fast = 0;
    for(int i = 0; i < 100; i++) {
        voltageSum_fast += analogRead(VOLTAGE_SENSOR_PIN);
        // NO delay - back to back reads
    }
    float voltage_fast = (float)voltageSum_fast / 100.0;
    float pin_fast = (voltage_fast / ESP32_ADC_MAX) * ESP_VREF;
    
    // Take 100 samples SLOW (with delay like the real code)
    long voltageSum_slow = 0;
    for(int i = 0; i < 100; i++) {
        voltageSum_slow += analogRead(VOLTAGE_SENSOR_PIN);
        delayMicroseconds(100);  // Same as sender_node.ino
    }
    float voltage_slow = (float)voltageSum_slow / 100.0;
    float pin_slow = (voltage_slow / ESP32_ADC_MAX) * ESP_VREF;
    
    // Display results
    Serial.println("===== VOLTAGE COMPARISON =====");
    Serial.println("FAST reads (no delay):");
    Serial.printf("  ADC: %.0f → Pin: %.4f V → Output (9.14x): %.2f V\n", 
                  voltage_fast, pin_fast, pin_fast * 9.14);
    Serial.println("\nSLOW reads (100µs delay - like sender code):");
    Serial.printf("  ADC: %.0f → Pin: %.4f V → Output (9.14x): %.2f V\n", 
                  voltage_slow, pin_slow, pin_slow * 9.14);
    Serial.printf("\nDifference: %.0f ADC (%.4f V)\n", 
                  voltage_fast - voltage_slow, pin_fast - pin_slow);
    Serial.println("================================\n");
    
    delay(2000);
}
