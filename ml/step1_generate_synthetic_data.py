"""
=============================================================================
STEP 1: Synthetic Data Generation for Solar Panel Fault Detection
=============================================================================
Author: Senior Data Scientist
Purpose: Generate realistic synthetic solar panel sensor data with fault labels
Output: solar_panel_dataset.csv (1600+ rows - 5 classes)
Features: Voltage, Current, Temperature, Light_Intensity, Efficiency
=============================================================================
"""

import numpy as np
import pandas as pd
import random
import os

# Set seed for reproducibility
np.random.seed(42)
random.seed(42)

# Panel specifications (for efficiency calculation)
PANEL_AREA = 1.6  # m² (typical residential panel)
SOLAR_CONSTANT = 1000  # W/m² at standard test conditions
PANEL_MAX_POWER = 300  # Watts (rated power)

def calculate_efficiency(voltage, current, light_intensity):
    """
    Calculate solar panel efficiency.
    Efficiency = (Power Output / Solar Input) * 100
    Solar Input = Light_Intensity * Panel_Area (simplified)
    """
    power_output = voltage * current
    # Convert lux to approximate W/m² (rough conversion: 1 lux ≈ 0.0079 W/m² for sunlight)
    irradiance = light_intensity * 0.0079
    solar_input = irradiance * PANEL_AREA
    
    if solar_input > 0:
        efficiency = (power_output / solar_input) * 100
        # Clip to realistic range (0-25% for silicon panels)
        efficiency = np.clip(efficiency, 0, 25)
    else:
        efficiency = 0
    
    return round(efficiency, 2)


def generate_solar_panel_data(n_samples=1500):
    """
    Generate synthetic solar panel sensor data mimicking real behavior.
    
    5 Fault Classes:
    - Normal: High Light + High Voltage + High Current + High Efficiency
    - Partial_Shading: LOW Light (blocked) + Reduced V/I + Lower Efficiency
    - Dust_Accumulation: Moderately reduced light + Gradual degradation
    - Open_Circuit: High Light + High Voltage + Zero Current
    - Short_Circuit: Low Voltage + High Current + High Temp
    
    Returns:
        DataFrame with Voltage, Current, Temperature, Light_Intensity, 
        Efficiency, Fault_Status
    """
    
    data = []
    
    # Define number of samples per class (balanced dataset)
    samples_per_class = n_samples // 5
    
    # =========================================================================
    # CLASS 1: NORMAL OPERATION
    # High Light (800-1200 lux) + High Voltage (17-22V) + High Current (4-6A)
    # High efficiency (15-22%)
    # =========================================================================
    for _ in range(samples_per_class):
        light_intensity = np.random.uniform(800, 1200)
        voltage = np.random.uniform(17, 22)
        current = np.random.uniform(4, 6)
        temperature = np.random.uniform(25, 45)
        efficiency = calculate_efficiency(voltage, current, light_intensity)
        # Add some realistic variation to efficiency
        efficiency = efficiency * np.random.uniform(0.9, 1.1)
        efficiency = np.clip(efficiency, 15, 22)
        
        data.append({
            'Voltage': voltage,
            'Current': current,
            'Temperature': temperature,
            'Light_Intensity': light_intensity,
            'Efficiency': round(efficiency, 2),
            'Fault_Status': 'Normal'
        })
    
    # =========================================================================
    # CLASS 2: PARTIAL SHADING
    # LOW Light (150-450 lux) - shadows blocking sunlight
    # Reduced Voltage (8-15V) + Reduced Current (1-3.5A)
    # Lower efficiency (5-12%) due to bypass diode losses
    # =========================================================================
    for _ in range(samples_per_class):
        light_intensity = np.random.uniform(150, 450)  # LOW - shading blocks sun!
        voltage = np.random.uniform(8, 15)
        current = np.random.uniform(1, 3.5)
        temperature = np.random.uniform(30, 55)  # Hot spots possible
        efficiency = np.random.uniform(5, 12)  # Reduced due to mismatch losses
        
        data.append({
            'Voltage': voltage,
            'Current': current,
            'Temperature': temperature,
            'Light_Intensity': light_intensity,
            'Efficiency': round(efficiency, 2),
            'Fault_Status': 'Partial_Shading'
        })
    
    # =========================================================================
    # CLASS 3: DUST ACCUMULATION
    # Moderately reduced light (400-700 lux) - dust layer blocks some light
    # Slightly reduced Voltage (14-19V) + Reduced Current (3-5A)
    # Reduced efficiency (10-16%) - gradual degradation
    # Higher temperature (35-55°C) - dust acts as insulator
    # =========================================================================
    for _ in range(samples_per_class):
        light_intensity = np.random.uniform(400, 700)  # Moderately reduced
        voltage = np.random.uniform(14, 19)
        current = np.random.uniform(3, 5)
        temperature = np.random.uniform(35, 55)  # Higher due to insulation
        efficiency = np.random.uniform(10, 16)  # Degraded performance
        
        data.append({
            'Voltage': voltage,
            'Current': current,
            'Temperature': temperature,
            'Light_Intensity': light_intensity,
            'Efficiency': round(efficiency, 2),
            'Fault_Status': 'Dust_Accumulation'
        })
    
    # =========================================================================
    # CLASS 4: OPEN CIRCUIT
    # High Light (700-1200 lux) + High Voltage (20-25V OCV) + Zero Current
    # Zero efficiency (no power output)
    # =========================================================================
    for _ in range(samples_per_class):
        light_intensity = np.random.uniform(700, 1200)
        voltage = np.random.uniform(20, 25)  # Open circuit voltage
        current = np.random.uniform(0, 0.15)  # Nearly zero
        temperature = np.random.uniform(25, 40)  # Normal (no power dissipation)
        efficiency = np.random.uniform(0, 2)  # Essentially zero
        
        data.append({
            'Voltage': voltage,
            'Current': current,
            'Temperature': temperature,
            'Light_Intensity': light_intensity,
            'Efficiency': round(efficiency, 2),
            'Fault_Status': 'Open_Circuit'
        })
    
    # =========================================================================
    # CLASS 5: SHORT CIRCUIT
    # Any light (500-1200 lux) + Very Low Voltage (0-4V) + High Current (6-10A)
    # Very low efficiency + Very high temperature (danger!)
    # =========================================================================
    for _ in range(samples_per_class):
        light_intensity = np.random.uniform(500, 1200)
        voltage = np.random.uniform(0, 4)  # Near zero
        current = np.random.uniform(6, 10)  # Short circuit current
        temperature = np.random.uniform(55, 85)  # Dangerous heat
        efficiency = np.random.uniform(0, 3)  # Minimal
        
        data.append({
            'Voltage': voltage,
            'Current': current,
            'Temperature': temperature,
            'Light_Intensity': light_intensity,
            'Efficiency': round(efficiency, 2),
            'Fault_Status': 'Short_Circuit'
        })
    
    # Create DataFrame
    df = pd.DataFrame(data)
    
    # Round numerical columns
    for col in ['Voltage', 'Current', 'Temperature', 'Light_Intensity']:
        df[col] = df[col].round(2)
    
    # Shuffle
    df = df.sample(frac=1, random_state=42).reset_index(drop=True)
    
    return df


def add_realistic_noise(df, noise_level=0.08):
    """
    Add realistic sensor noise to create overlapping regions between classes.
    This prevents overfitting by making boundaries less clear.
    
    Args:
        df: DataFrame with sensor data
        noise_level: Percentage of noise (8% default for realistic overlap)
    
    Returns:
        DataFrame with added noise
    """
    df_noisy = df.copy()
    
    numerical_cols = ['Voltage', 'Current', 'Temperature', 'Light_Intensity', 'Efficiency']
    
    for col in numerical_cols:
        # Add Gaussian noise
        noise = np.random.normal(0, noise_level * df_noisy[col].std(), len(df_noisy))
        df_noisy[col] = df_noisy[col] + noise
        df_noisy[col] = df_noisy[col].round(2)
        
        # Ensure no negative values
        df_noisy[col] = df_noisy[col].clip(lower=0)
    
    # Clip efficiency to realistic range
    df_noisy['Efficiency'] = df_noisy['Efficiency'].clip(0, 25)
    
    return df_noisy


def add_edge_cases(df, n_edge_cases=100):
    """
    Add edge cases that lie between fault types to prevent overfitting.
    These are ambiguous cases that make the model more robust.
    """
    edge_data = []
    
    # Edge case: Between Normal and Dust (borderline dusty)
    for _ in range(n_edge_cases // 5):
        light = np.random.uniform(680, 820)  # Overlapping region
        voltage = np.random.uniform(16, 19)
        current = np.random.uniform(4, 5)
        temp = np.random.uniform(40, 50)
        eff = np.random.uniform(14, 17)
        fault = np.random.choice(['Normal', 'Dust_Accumulation'])
        edge_data.append({
            'Voltage': voltage, 'Current': current, 'Temperature': temp,
            'Light_Intensity': light, 'Efficiency': eff, 'Fault_Status': fault
        })
    
    # Edge case: Between Dust and Partial Shading
    for _ in range(n_edge_cases // 5):
        light = np.random.uniform(420, 520)  # Overlapping region
        voltage = np.random.uniform(13, 16)
        current = np.random.uniform(3, 4)
        temp = np.random.uniform(45, 55)
        eff = np.random.uniform(10, 13)
        fault = np.random.choice(['Dust_Accumulation', 'Partial_Shading'])
        edge_data.append({
            'Voltage': voltage, 'Current': current, 'Temperature': temp,
            'Light_Intensity': light, 'Efficiency': eff, 'Fault_Status': fault
        })
    
    # Edge case: Between Normal and Open Circuit (high V, low I)
    for _ in range(n_edge_cases // 5):
        light = np.random.uniform(800, 1000)
        voltage = np.random.uniform(19, 22)
        current = np.random.uniform(0.1, 0.5)  # Ambiguous current
        temp = np.random.uniform(30, 40)
        eff = np.random.uniform(1, 5)
        fault = np.random.choice(['Normal', 'Open_Circuit'], p=[0.3, 0.7])
        edge_data.append({
            'Voltage': voltage, 'Current': current, 'Temperature': temp,
            'Light_Intensity': light, 'Efficiency': eff, 'Fault_Status': fault
        })
    
    # Edge case: Unusual conditions
    for _ in range(n_edge_cases // 5):
        light = np.random.uniform(600, 900)
        voltage = np.random.uniform(12, 18)
        current = np.random.uniform(2.5, 4.5)
        temp = np.random.uniform(35, 50)
        eff = np.random.uniform(8, 15)
        fault = np.random.choice(['Normal', 'Dust_Accumulation', 'Partial_Shading'])
        edge_data.append({
            'Voltage': voltage, 'Current': current, 'Temperature': temp,
            'Light_Intensity': light, 'Efficiency': eff, 'Fault_Status': fault
        })
    
    # More random edge cases
    for _ in range(n_edge_cases // 5):
        light = np.random.uniform(500, 800)
        voltage = np.random.uniform(10, 20)
        current = np.random.uniform(2, 5)
        temp = np.random.uniform(30, 55)
        eff = np.random.uniform(8, 18)
        fault = np.random.choice(['Normal', 'Dust_Accumulation', 'Partial_Shading'])
        edge_data.append({
            'Voltage': voltage, 'Current': current, 'Temperature': temp,
            'Light_Intensity': light, 'Efficiency': eff, 'Fault_Status': fault
        })
    
    edge_df = pd.DataFrame(edge_data)
    for col in ['Voltage', 'Current', 'Temperature', 'Light_Intensity', 'Efficiency']:
        edge_df[col] = edge_df[col].round(2)
    
    # Combine with original data
    combined = pd.concat([df, edge_df], ignore_index=True)
    combined = combined.sample(frac=1, random_state=42).reset_index(drop=True)
    
    return combined


def print_dataset_statistics(df):
    """Print comprehensive statistics about the generated dataset."""
    
    print("=" * 70)
    print("SYNTHETIC SOLAR PANEL DATASET STATISTICS")
    print("=" * 70)
    
    print(f"\n📊 Total Samples: {len(df)}")
    print(f"\n📋 Class Distribution:")
    print(df['Fault_Status'].value_counts())
    
    print(f"\n📈 Feature Statistics:")
    print(df.describe().round(2))
    
    print(f"\n🔍 Statistics by Fault Type:")
    for fault in sorted(df['Fault_Status'].unique()):
        subset = df[df['Fault_Status'] == fault]
        print(f"\n--- {fault} ({len(subset)} samples) ---")
        print(f"  Voltage:     {subset['Voltage'].mean():.2f} ± {subset['Voltage'].std():.2f} V")
        print(f"  Current:     {subset['Current'].mean():.2f} ± {subset['Current'].std():.2f} A")
        print(f"  Temperature: {subset['Temperature'].mean():.2f} ± {subset['Temperature'].std():.2f} °C")
        print(f"  Light:       {subset['Light_Intensity'].mean():.2f} ± {subset['Light_Intensity'].std():.2f} lux")
        print(f"  Efficiency:  {subset['Efficiency'].mean():.2f} ± {subset['Efficiency'].std():.2f} %")


# =============================================================================
# MAIN EXECUTION
# =============================================================================
if __name__ == "__main__":
    
    print("\n🔆 Generating Synthetic Solar Panel Fault Detection Dataset...")
    print("-" * 70)
    print("Features: Voltage, Current, Temperature, Light_Intensity, Efficiency")
    print("Classes: Normal, Partial_Shading, Dust_Accumulation, Open_Circuit, Short_Circuit")
    print("-" * 70)
    
    # Generate base data (5 classes × 300 samples = 1500)
    df = generate_solar_panel_data(n_samples=1500)
    
    # Add realistic sensor noise (8% for realistic overlap)
    df = add_realistic_noise(df, noise_level=0.08)
    
    # Add edge cases to prevent overfitting (100 edge cases)
    df = add_edge_cases(df, n_edge_cases=100)
    
    # Print statistics
    print_dataset_statistics(df)
    
    # Save to CSV in data folder
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(base_dir, 'data')
    os.makedirs(data_dir, exist_ok=True)
    output_file = os.path.join(data_dir, 'solar_panel_dataset.csv')
    df.to_csv(output_file, index=False)
    
    print(f"\n✅ Dataset saved to: {output_file}")
    print(f"📁 Shape: {df.shape}")
    
    # Display sample rows
    print("\n📝 Sample Data (First 10 rows):")
    print(df.head(10).to_string())
    
    print("\n" + "=" * 70)
    print("Data generation complete! Proceed to Step 2 for model training.")
    print("=" * 70)
