#!/usr/bin/env python
"""Fix the scaler to ensure it has 8 features"""

import pandas as pd
import joblib
from sklearn.preprocessing import StandardScaler

# Load the data
df = pd.read_csv('../data/solar_panel_dataset.csv')
feature_cols = ['Voltage', 'Current', 'Temperature', 'Light_Intensity', 
                'Humidity', 'ThermistorTemp', 'RelayStatus', 'Efficiency']

X = df[feature_cols].values
print(f'Data shape: {X.shape}')
print(f'Number of features: {X.shape[1]}')

# Create and fit a fresh scaler
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

print(f'\nFresh scaler with {len(scaler.mean_)} features')
print(f'Means: {scaler.mean_}')
print(f'Stds: {scaler.scale_}')

# Save to models directory
joblib.dump(scaler, '../models/solar_fault_scaler.joblib')
print(f'\n✅ Scaler saved to models/solar_fault_scaler.joblib')

# Also copy to backend
import shutil
shutil.copy('../models/solar_fault_scaler.joblib', '../backend/solar_fault_scaler.joblib')
print(f'✅ Scaler copied to backend/solar_fault_scaler.joblib')
