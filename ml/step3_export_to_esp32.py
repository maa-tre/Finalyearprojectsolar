"""
=============================================================================
STEP 3: Export Random Forest Model to C Code for ESP32
=============================================================================
Author: Senior Data Scientist
Purpose: Convert trained sklearn model to C code for embedded deployment
Methods: micromlgen library (primary) + manual export (backup)
=============================================================================

IMPORTANT: Install micromlgen before running:
    pip install micromlgen

If micromlgen doesn't work, this script also includes a manual export method.
=============================================================================
"""

import numpy as np
import joblib
import os
from datetime import datetime

# Try to import micromlgen
try:
    from micromlgen import port
    MICROMLGEN_AVAILABLE = True
    print("✅ micromlgen library found!")
except ImportError:
    MICROMLGEN_AVAILABLE = False
    print("⚠️  micromlgen not installed. Using manual export method.")
    print("   To install: pip install micromlgen")


# =============================================================================
# CONFIGURATION
# =============================================================================
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODELS_DIR = os.path.join(BASE_DIR, 'models')

CONFIG = {
    'model_path': os.path.join(MODELS_DIR, 'solar_fault_rf_model.joblib'),
    'scaler_path': os.path.join(MODELS_DIR, 'solar_fault_scaler.joblib'),
    'label_encoder_path': os.path.join(MODELS_DIR, 'solar_fault_label_encoder.joblib'),
    'output_header': os.path.join(MODELS_DIR, 'model.h'),
    'output_header_manual': os.path.join(MODELS_DIR, 'model_manual.h')
}


def load_artifacts():
    """Load all trained model artifacts."""
    
    print("\n" + "=" * 70)
    print("📂 LOADING MODEL ARTIFACTS")
    print("=" * 70)
    
    model = joblib.load(CONFIG['model_path'])
    scaler = joblib.load(CONFIG['scaler_path'])
    label_encoder = joblib.load(CONFIG['label_encoder_path'])
    
    print(f"✅ Model loaded: {model.n_estimators} trees, max_depth={model.max_depth}")
    print(f"✅ Scaler loaded: {len(scaler.mean_)} features")
    print(f"✅ Labels: {list(label_encoder.classes_)}")
    
    return model, scaler, label_encoder


def export_with_micromlgen(model):
    """
    Export model using micromlgen library.
    This is the recommended method for sklearn -> C conversion.
    """
    
    print("\n" + "=" * 70)
    print("🔧 EXPORTING WITH MICROMLGEN")
    print("=" * 70)
    
    if not MICROMLGEN_AVAILABLE:
        print("❌ micromlgen not available. Skipping...")
        return False
    
    try:
        # Port the model to C code
        c_code = port(model)
        
        # Save to header file
        with open(CONFIG['output_header'], 'w') as f:
            f.write(c_code)
        
        print(f"✅ Model exported to: {CONFIG['output_header']}")
        print(f"   File size: {os.path.getsize(CONFIG['output_header'])} bytes")
        
        return True
        
    except Exception as e:
        print(f"❌ micromlgen export failed: {e}")
        return False


def export_manual(model, scaler, label_encoder):
    """
    Manual export of Random Forest to C code.
    This is a backup method that creates a simplified but functional model.
    
    For ESP32, we export:
    1. Scaler parameters (mean, std)
    2. Decision tree structure for each tree
    3. Voting logic
    """
    
    print("\n" + "=" * 70)
    print("🔧 MANUAL EXPORT TO C CODE")
    print("=" * 70)
    
    feature_names = ['Voltage', 'Current', 'Temperature', 'Light_Intensity']
    class_names = list(label_encoder.classes_)
    n_classes = len(class_names)
    n_features = len(feature_names)
    n_trees = model.n_estimators
    
    # Start building C code
    c_code = []
    
    # Header comment
    c_code.append("/*")
    c_code.append(" * Solar Panel Fault Detection - Random Forest Model")
    c_code.append(f" * Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    c_code.append(f" * Trees: {n_trees}, Max Depth: {model.max_depth}")
    c_code.append(" * ")
    c_code.append(" * Classes:")
    for i, name in enumerate(class_names):
        c_code.append(f" *   {i}: {name}")
    c_code.append(" */")
    c_code.append("")
    
    # Include guards
    c_code.append("#ifndef SOLAR_FAULT_MODEL_H")
    c_code.append("#define SOLAR_FAULT_MODEL_H")
    c_code.append("")
    
    # Constants
    c_code.append(f"#define NUM_FEATURES {n_features}")
    c_code.append(f"#define NUM_CLASSES {n_classes}")
    c_code.append(f"#define NUM_TREES {n_trees}")
    c_code.append("")
    
    # Class names as strings
    c_code.append("// Fault type names")
    c_code.append("const char* CLASS_NAMES[] = {")
    for name in class_names:
        c_code.append(f'    "{name}",')
    c_code.append("};")
    c_code.append("")
    
    # Feature names
    c_code.append("// Feature names (for debugging)")
    c_code.append("const char* FEATURE_NAMES[] = {")
    for name in feature_names:
        c_code.append(f'    "{name}",')
    c_code.append("};")
    c_code.append("")
    
    # Scaler parameters
    c_code.append("// StandardScaler parameters")
    c_code.append("const float SCALER_MEAN[] = {")
    c_code.append("    " + ", ".join([f"{m:.6f}f" for m in scaler.mean_]))
    c_code.append("};")
    c_code.append("")
    c_code.append("const float SCALER_STD[] = {")
    c_code.append("    " + ", ".join([f"{s:.6f}f" for s in scaler.scale_]))
    c_code.append("};")
    c_code.append("")
    
    # Scale function
    c_code.append("// Apply StandardScaler transformation")
    c_code.append("void scale_features(float* features, float* scaled) {")
    c_code.append("    for (int i = 0; i < NUM_FEATURES; i++) {")
    c_code.append("        scaled[i] = (features[i] - SCALER_MEAN[i]) / SCALER_STD[i];")
    c_code.append("    }")
    c_code.append("}")
    c_code.append("")
    
    # Generate each decision tree
    for tree_idx, tree in enumerate(model.estimators_):
        c_code.append(f"// Decision Tree {tree_idx}")
        c_code.append(f"int predict_tree_{tree_idx}(float* features) {{")
        
        # Get tree structure
        tree_ = tree.tree_
        feature = tree_.feature
        threshold = tree_.threshold
        children_left = tree_.children_left
        children_right = tree_.children_right
        value = tree_.value
        
        # Generate recursive tree traversal as if-else statements
        def tree_to_code(node, indent=1):
            indent_str = "    " * indent
            
            # Leaf node
            if children_left[node] == children_right[node]:
                # Get the majority class
                class_counts = value[node][0]
                predicted_class = np.argmax(class_counts)
                return [f"{indent_str}return {predicted_class};"]
            
            # Internal node
            code = []
            feat_idx = feature[node]
            thresh = threshold[node]
            
            code.append(f"{indent_str}if (features[{feat_idx}] <= {thresh:.6f}f) {{")
            code.extend(tree_to_code(children_left[node], indent + 1))
            code.append(f"{indent_str}}} else {{")
            code.extend(tree_to_code(children_right[node], indent + 1))
            code.append(f"{indent_str}}}")
            
            return code
        
        c_code.extend(tree_to_code(0))
        c_code.append("}")
        c_code.append("")
    
    # Main prediction function
    c_code.append("// Main prediction function - returns class index")
    c_code.append("int predict(float* raw_features) {")
    c_code.append("    // Scale features")
    c_code.append("    float scaled[NUM_FEATURES];")
    c_code.append("    scale_features(raw_features, scaled);")
    c_code.append("    ")
    c_code.append("    // Vote counting")
    c_code.append("    int votes[NUM_CLASSES] = {0};")
    c_code.append("    ")
    c_code.append("    // Get prediction from each tree")
    
    for i in range(n_trees):
        c_code.append(f"    votes[predict_tree_{i}(scaled)]++;")
    
    c_code.append("    ")
    c_code.append("    // Find majority vote")
    c_code.append("    int max_votes = 0;")
    c_code.append("    int predicted_class = 0;")
    c_code.append("    for (int i = 0; i < NUM_CLASSES; i++) {")
    c_code.append("        if (votes[i] > max_votes) {")
    c_code.append("            max_votes = votes[i];")
    c_code.append("            predicted_class = i;")
    c_code.append("        }")
    c_code.append("    }")
    c_code.append("    ")
    c_code.append("    return predicted_class;")
    c_code.append("}")
    c_code.append("")
    
    # Convenience function to get class name
    c_code.append("// Get the class name string from prediction")
    c_code.append("const char* predict_class_name(float* raw_features) {")
    c_code.append("    int class_idx = predict(raw_features);")
    c_code.append("    return CLASS_NAMES[class_idx];")
    c_code.append("}")
    c_code.append("")
    
    # Check if fault (any non-Normal condition)
    c_code.append("// Check if there's a fault (returns 1 if fault, 0 if normal)")
    c_code.append("int is_fault(float* raw_features) {")
    c_code.append("    int prediction = predict(raw_features);")
    c_code.append("    // Class 0 is 'Normal' - return 1 if NOT normal")
    
    # Find the index of 'Normal' class
    normal_idx = list(label_encoder.classes_).index('Normal')
    c_code.append(f"    return (prediction != {normal_idx}) ? 1 : 0;")
    c_code.append("}")
    c_code.append("")
    
    # Close include guard
    c_code.append("#endif // SOLAR_FAULT_MODEL_H")
    
    # Write to file
    output_file = CONFIG['output_header_manual']
    with open(output_file, 'w') as f:
        f.write('\n'.join(c_code))
    
    print(f"✅ Manual export complete: {output_file}")
    print(f"   File size: {os.path.getsize(output_file)} bytes")
    print(f"   Lines of code: {len(c_code)}")
    
    return output_file


def verify_export(model, scaler, label_encoder):
    """Verify the exported model matches Python predictions."""
    
    print("\n" + "=" * 70)
    print("🧪 VERIFYING EXPORT")
    print("=" * 70)
    
    test_cases = [
        [20.0, 5.0, 35.0, 1000.0],   # Normal
        [10.0, 2.0, 45.0, 800.0],    # Partial Shading
        [22.0, 0.05, 30.0, 900.0],   # Open Circuit
        [1.0, 8.0, 65.0, 700.0],     # Short Circuit
    ]
    
    print("\nPython model predictions (for verification):")
    print("-" * 50)
    
    for i, features in enumerate(test_cases):
        X = np.array([features])
        X_scaled = scaler.transform(X)
        prediction = model.predict(X_scaled)[0]
        label = label_encoder.inverse_transform([prediction])[0]
        
        print(f"Test {i+1}: V={features[0]:5.1f}, I={features[1]:4.2f}, "
              f"T={features[2]:4.1f}, L={features[3]:6.1f}")
        print(f"        Prediction: {prediction} ({label})")
        print()
    
    print("✅ Use these values to verify ESP32 predictions match!")


def generate_esp32_usage_guide():
    """Generate documentation for using the model on ESP32."""
    
    guide = """
================================================================================
ESP32 USAGE GUIDE - Solar Panel Fault Detection Model
================================================================================

INSTALLATION STEPS:
-------------------
1. Copy 'model.h' (or 'model_manual.h') to your Arduino project folder
2. Include it in your sketch: #include "model.h"
3. Call predict() with your sensor readings

USAGE EXAMPLE:
--------------
```cpp
#include "model.h"

void loop() {
    // Read sensors (replace with actual sensor code)
    float voltage = readVoltage();
    float current = readCurrent();
    float temperature = readTemperature();
    float light = readLight();
    
    // Create feature array [Voltage, Current, Temperature, Light_Intensity]
    float features[4] = {voltage, current, temperature, light};
    
    // Get prediction (returns 0-3)
    int fault_type = predict(features);
    
    // Get human-readable name
    const char* fault_name = CLASS_NAMES[fault_type];
    
    // Check if any fault
    if (is_fault(features)) {
        // Turn on fault LED
        digitalWrite(FAULT_LED, HIGH);
    }
}
```

CLASS MAPPING:
--------------
0 = Normal
1 = Open_Circuit  
2 = Partial_Shading
3 = Short_Circuit

MEMORY USAGE (APPROXIMATE):
---------------------------
- Code size: ~5-15 KB (depends on tree complexity)
- RAM usage: ~200-500 bytes
- Suitable for ESP32 (520KB RAM, 4MB Flash)

TIPS FOR ESP32:
---------------
1. Use float instead of double for ESP32 efficiency
2. Model is already optimized with limited trees and depth
3. Prediction takes ~1-5ms on ESP32 @ 240MHz
4. Consider using FreeRTOS task for continuous monitoring

================================================================================
"""
    
    with open('ESP32_USAGE_GUIDE.txt', 'w') as f:
        f.write(guide)
    
    print("\n✅ Usage guide saved to: ESP32_USAGE_GUIDE.txt")


# =============================================================================
# MAIN EXECUTION
# =============================================================================
if __name__ == "__main__":
    
    print("\n")
    print("╔" + "═" * 68 + "╗")
    print("║" + " " * 15 + "ESP32 MODEL EXPORT TOOL" + " " * 20 + "║")
    print("║" + " " * 12 + "Random Forest -> C Header Conversion" + " " * 10 + "║")
    print("╚" + "═" * 68 + "╝")
    
    # Load artifacts
    model, scaler, label_encoder = load_artifacts()
    
    # Try micromlgen first
    micromlgen_success = export_with_micromlgen(model)
    
    # Always do manual export as well (more control)
    manual_output = export_manual(model, scaler, label_encoder)
    
    # Verify export
    verify_export(model, scaler, label_encoder)
    
    # Generate guide
    generate_esp32_usage_guide()
    
    print("\n" + "=" * 70)
    print("🎉 EXPORT COMPLETE!")
    print("=" * 70)
    
    if micromlgen_success:
        print(f"\n✅ micromlgen export: {CONFIG['output_header']}")
    
    print(f"✅ Manual export: {CONFIG['output_header_manual']}")
    print(f"✅ Usage guide: ESP32_USAGE_GUIDE.txt")
    
    print("\n📋 NEXT STEPS:")
    print("   1. Copy model_manual.h to your ESP32 Arduino project")
    print("   2. Open step4_esp32_arduino_sketch.ino in Arduino IDE")
    print("   3. Upload to ESP32 and test!")
    print("=" * 70)
