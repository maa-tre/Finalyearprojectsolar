"""
=============================================================================
STEP 2: Random Forest Training for Solar Panel Fault Detection
=============================================================================
Author: Senior Data Scientist
Purpose: Train, evaluate, and visualize a Random Forest classifier
Features: StandardScaler, Confusion Matrix, Decision Tree Visualization
=============================================================================
"""

import warnings
from typing import Tuple, List

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import joblib
from sklearn.model_selection import train_test_split, cross_val_score, StratifiedKFold
from sklearn.preprocessing import StandardScaler, LabelEncoder
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix, f1_score
from sklearn.tree import plot_tree, export_text

warnings.filterwarnings('ignore')


# =============================================================================
# CONFIGURATION
# =============================================================================
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

CONFIG = {
    'data_file': os.path.join(BASE_DIR, 'data', 'solar_panel_dataset.csv'),
    'models_dir': os.path.join(BASE_DIR, 'models'),
    'assets_dir': os.path.join(BASE_DIR, 'assets'),
    'test_size': 0.20,
    'random_state': 42,
    'n_estimators': 15,      # Number of trees (keep small for ESP32)
    'max_depth': 6,          # Limit depth for embedded deployment
    'min_samples_split': 10, # Increased to prevent overfitting
    'min_samples_leaf': 5,   # Increased to prevent overfitting
    'cv_folds': 5            # Cross-validation folds
}


def load_and_explore_data(filepath: str) -> pd.DataFrame:
    """Load dataset and perform basic exploration."""
    print("\n" + "=" * 70)
    print("LOADING DATA")
    print("=" * 70)
    
    df = pd.read_csv(filepath)
    
    print(f"\n✅ Loaded {len(df)} samples")
    print(f"\n📋 Columns: {list(df.columns)}")
    print(f"\n🎯 Target Distribution:")
    print(df['Fault_Status'].value_counts())
    
    # Check for missing values
    missing = df.isnull().sum().sum()
    print(f"\n❓ Missing Values: {missing}")
    
    return df


def preprocess_data(df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, StandardScaler, LabelEncoder, List[str]]:
    """Preprocess the data: encode labels, scale features, split data."""
    print("\n" + "=" * 70)
    print("PREPROCESSING DATA")
    print("=" * 70)
    
    # Define features and target (now includes Efficiency)
    feature_cols = ['Voltage', 'Current', 'Temperature', 'Light_Intensity', 'Efficiency']
    target_col = 'Fault_Status'
    
    X = df[feature_cols].values
    y = df[target_col].values
    
    # Encode labels
    label_encoder = LabelEncoder()
    y_encoded = label_encoder.fit_transform(y)
    
    print(f"\n🏷️  Label Encoding Mapping:")
    for i, label in enumerate(label_encoder.classes_):
        print(f"   {i} -> {label}")
    
    # Split data (80/20)
    X_train, X_test, y_train, y_test = train_test_split(
        X, y_encoded, 
        test_size=CONFIG['test_size'], 
        random_state=CONFIG['random_state'],
        stratify=y_encoded  # Maintain class balance
    )
    
    print(f"\n📊 Train/Test Split:")
    print(f"   Training samples: {len(X_train)}")
    print(f"   Testing samples:  {len(X_test)}")
    
    # Scale features using StandardScaler
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)
    
    print(f"\n📐 Feature Scaling Applied (StandardScaler)")
    print(f"   Means: {scaler.mean_.round(2)}")
    print(f"   Stds:  {scaler.scale_.round(2)}")
    
    return (X_train_scaled, X_test_scaled, y_train, y_test, 
            scaler, label_encoder, feature_cols)


def train_random_forest(X_train: np.ndarray, y_train: np.ndarray) -> RandomForestClassifier:
    """Train a Random Forest Classifier optimized for ESP32 embedded deployment."""
    print("\n" + "=" * 70)
    print("TRAINING RANDOM FOREST CLASSIFIER")
    print("=" * 70)
    
    # Initialize classifier with regularization to prevent overfitting
    rf_classifier = RandomForestClassifier(
        n_estimators=CONFIG['n_estimators'],
        max_depth=CONFIG['max_depth'],
        min_samples_split=CONFIG['min_samples_split'],
        min_samples_leaf=CONFIG['min_samples_leaf'],
        max_features='sqrt',  # Regularization: use sqrt of features
        bootstrap=True,
        oob_score=True,       # Out-of-bag score for validation
        random_state=CONFIG['random_state'],
        n_jobs=-1,
        verbose=1
    )
    
    print(f"\n⚙️  Model Parameters (with regularization):")
    print(f"   Number of Trees: {CONFIG['n_estimators']}")
    print(f"   Max Depth: {CONFIG['max_depth']}")
    print(f"   Min Samples Split: {CONFIG['min_samples_split']}")
    print(f"   Min Samples Leaf: {CONFIG['min_samples_leaf']}")
    print(f"   Max Features: sqrt (regularization)")
    
    # Cross-validation to check for overfitting
    print(f"\n📊 Performing {CONFIG['cv_folds']}-Fold Cross-Validation...")
    cv = StratifiedKFold(n_splits=CONFIG['cv_folds'], shuffle=True, random_state=CONFIG['random_state'])
    cv_scores = cross_val_score(rf_classifier, X_train, y_train, cv=cv, scoring='accuracy')
    
    print(f"   CV Scores: {cv_scores.round(4)}")
    print(f"   CV Mean: {cv_scores.mean():.4f} (+/- {cv_scores.std() * 2:.4f})")
    
    # Check for overfitting (high variance in CV scores)
    if cv_scores.std() > 0.05:
        print(f"   ⚠️  Warning: High variance in CV scores may indicate overfitting")
    else:
        print(f"   ✅ CV scores are stable - good generalization")
    
    # Train the model on full training data
    print(f"\n🚀 Training on full training set...")
    rf_classifier.fit(X_train, y_train)
    
    print(f"\n✅ Training Complete!")
    print(f"   OOB Score: {rf_classifier.oob_score_:.4f}")
    
    return rf_classifier


def evaluate_model(model: RandomForestClassifier, X_test: np.ndarray, y_test: np.ndarray, label_encoder: LabelEncoder) -> Tuple[float, np.ndarray]:
    """Evaluate model performance with multiple metrics."""
    print("\n" + "=" * 70)
    print("MODEL EVALUATION")
    print("=" * 70)
    
    # Predictions
    y_pred = model.predict(X_test)
    
    # Accuracy
    accuracy = accuracy_score(y_test, y_pred)
    print(f"\n🎯 Overall Accuracy: {accuracy * 100:.2f}%")
    
    # F1 Score (weighted)
    f1 = f1_score(y_test, y_pred, average='weighted')
    print(f"📈 Weighted F1-Score: {f1:.4f}")
    
    # Classification Report
    print(f"\n📋 Classification Report:")
    print("-" * 50)
    print(classification_report(
        y_test, y_pred, 
        target_names=label_encoder.classes_
    ))
    
    # Confusion Matrix
    cm = confusion_matrix(y_test, y_pred)
    
    return accuracy, cm


def plot_confusion_matrix(cm: np.ndarray, labels: np.ndarray, save_path: str = 'confusion_matrix.png') -> None:
    """Generate and save a confusion matrix visualization."""
    print("\n" + "=" * 70)
    print("GENERATING CONFUSION MATRIX VISUALIZATION")
    print("=" * 70)
    
    plt.figure(figsize=(10, 8))
    
    # Create heatmap
    sns.heatmap(
        cm, 
        annot=True, 
        fmt='d', 
        cmap='Blues',
        xticklabels=labels,
        yticklabels=labels,
        annot_kws={'size': 14}
    )
    
    plt.title('Solar Panel Fault Detection\nConfusion Matrix', fontsize=16, fontweight='bold')
    plt.ylabel('True Label', fontsize=12)
    plt.xlabel('Predicted Label', fontsize=12)
    plt.xticks(rotation=45, ha='right')
    plt.yticks(rotation=0)
    plt.tight_layout()
    
    # Save figure
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n✅ Confusion matrix saved to: {save_path}")
    
    plt.show()


def visualize_decision_tree(model: RandomForestClassifier, feature_names: List[str], class_names: np.ndarray, 
                            tree_index: int = 0, save_path: str = 'decision_tree.png') -> None:
    """Visualize one decision tree from the Random Forest to show classification rules."""
    print("\n" + "=" * 70)
    print(f"VISUALIZING DECISION TREE #{tree_index}")
    print("=" * 70)
    
    # Get one tree from the forest
    tree = model.estimators_[tree_index]
    
    # Create figure
    plt.figure(figsize=(25, 15))
    
    # Plot the tree (class_names must be a list)
    plot_tree(
        tree,
        feature_names=feature_names,
        class_names=list(class_names),
        filled=True,
        rounded=True,
        fontsize=10,
        proportion=True,
        precision=2
    )
    
    plt.title(f'Decision Tree #{tree_index} from Random Forest\n'
              f'(Solar Panel Fault Detection)', 
              fontsize=18, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n✅ Decision tree visualization saved to: {save_path}")
    
    plt.show()
    
    # Also print text-based rules
    print(f"\n📜 Decision Rules (Text Format):")
    print("-" * 50)
    rules = export_text(tree, feature_names=feature_names, max_depth=5)
    print(rules)


def plot_feature_importance(model: RandomForestClassifier, feature_names: List[str], save_path: str = 'feature_importance.png') -> None:
    """Visualize feature importance from the Random Forest."""
    print("\n" + "=" * 70)
    print("FEATURE IMPORTANCE ANALYSIS")
    print("=" * 70)
    
    # Get feature importances
    importances = model.feature_importances_
    indices = np.argsort(importances)[::-1]
    
    print("\n🔍 Feature Ranking:")
    for i, idx in enumerate(indices):
        print(f"   {i+1}. {feature_names[idx]}: {importances[idx]:.4f}")
    
    # Create visualization
    plt.figure(figsize=(10, 6))
    
    colors = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4']
    
    bars = plt.barh(range(len(importances)), importances[indices], color=colors)
    plt.yticks(range(len(importances)), [feature_names[i] for i in indices])
    plt.xlabel('Importance Score', fontsize=12)
    plt.title('Feature Importance for Solar Panel Fault Detection', 
              fontsize=14, fontweight='bold')
    
    # Add value labels
    for bar, val in zip(bars, importances[indices]):
        plt.text(val + 0.01, bar.get_y() + bar.get_height()/2, 
                 f'{val:.3f}', va='center', fontsize=10)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"\n✅ Feature importance plot saved to: {save_path}")
    
    plt.show()


def save_model_artifacts(model: RandomForestClassifier, scaler: StandardScaler, 
                         label_encoder: LabelEncoder, feature_names: List[str]) -> None:
    """Save all model artifacts for later use and ESP32 conversion."""
    print("\n" + "=" * 70)
    print("SAVING MODEL ARTIFACTS")
    print("=" * 70)
    
    # Save the trained model
    joblib.dump(model, 'solar_fault_rf_model.joblib')
    print("✅ Model saved: solar_fault_rf_model.joblib")
    
    # Save the scaler
    joblib.dump(scaler, 'solar_fault_scaler.joblib')
    print("✅ Scaler saved: solar_fault_scaler.joblib")
    
    # Save label encoder
    joblib.dump(label_encoder, 'solar_fault_label_encoder.joblib')
    print("✅ Label encoder saved: solar_fault_label_encoder.joblib")
    
    # Save feature names
    joblib.dump(feature_names, 'solar_fault_feature_names.joblib')
    print("✅ Feature names saved: solar_fault_feature_names.joblib")
    
    # Save model info as text
    with open('model_info.txt', 'w') as f:
        f.write("Solar Panel Fault Detection - Model Info\n")
        f.write("=" * 50 + "\n\n")
        f.write(f"Model Type: Random Forest Classifier\n")
        f.write(f"Number of Trees: {model.n_estimators}\n")
        f.write(f"Max Depth: {model.max_depth}\n")
        f.write(f"\nFeatures:\n")
        for i, feat in enumerate(feature_names):
            f.write(f"  {i}: {feat}\n")
        f.write(f"\nClasses:\n")
        for i, cls in enumerate(label_encoder.classes_):
            f.write(f"  {i}: {cls}\n")
        f.write(f"\nScaler Parameters:\n")
        f.write(f"  Means: {scaler.mean_}\n")
        f.write(f"  Scales: {scaler.scale_}\n")
    
    print("✅ Model info saved: model_info.txt")


def test_single_prediction(model: RandomForestClassifier, scaler: StandardScaler, 
                           label_encoder: LabelEncoder, feature_names: List[str]) -> None:
    """Test the model with sample predictions for each fault type (5 classes)."""
    print("\n" + "=" * 70)
    print("TESTING SINGLE PREDICTION")
    print("=" * 70)
    
    # Test cases representing each fault type (now includes Efficiency)
    test_cases = [
        {'Voltage': 20.0, 'Current': 5.0, 'Temperature': 35, 'Light_Intensity': 1000, 
         'Efficiency': 18.0, 'Expected': 'Normal'},
        {'Voltage': 12.0, 'Current': 2.5, 'Temperature': 45, 'Light_Intensity': 300, 
         'Efficiency': 8.0, 'Expected': 'Partial_Shading'},
        {'Voltage': 16.5, 'Current': 4.0, 'Temperature': 45, 'Light_Intensity': 550, 
         'Efficiency': 13.0, 'Expected': 'Dust_Accumulation'},
        {'Voltage': 22.0, 'Current': 0.05, 'Temperature': 30, 'Light_Intensity': 900, 
         'Efficiency': 1.0, 'Expected': 'Open_Circuit'},
        {'Voltage': 1.5, 'Current': 8.0, 'Temperature': 70, 'Light_Intensity': 800, 
         'Efficiency': 1.5, 'Expected': 'Short_Circuit'},
    ]
    
    for i, test in enumerate(test_cases):
        # Prepare input (now includes Efficiency)
        X_sample = np.array([[test['Voltage'], test['Current'], 
                              test['Temperature'], test['Light_Intensity'],
                              test['Efficiency']]])
        
        # Scale input
        X_scaled = scaler.transform(X_sample)
        
        # Predict
        prediction = model.predict(X_scaled)[0]
        proba = model.predict_proba(X_scaled)[0]
        predicted_label = label_encoder.inverse_transform([prediction])[0]
        
        print(f"\n📍 Test Case {i+1}:")
        print(f"   Input: V={test['Voltage']}V, I={test['Current']}A, "
              f"T={test['Temperature']}°C, Light={test['Light_Intensity']}lux")
        print(f"   Expected: {test['Expected']}")
        print(f"   Predicted: {predicted_label}")
        print(f"   Confidence: {max(proba)*100:.1f}%")
        
        match = "✅" if predicted_label == test['Expected'] else "❌"
        print(f"   Result: {match}")


# =============================================================================
# MAIN EXECUTION
# =============================================================================
if __name__ == "__main__":
    
    print("\n")
    print("╔" + "═" * 68 + "╗")
    print("║" + " " * 15 + "SOLAR PANEL FAULT DETECTION" + " " * 16 + "║")
    print("║" + " " * 15 + "Random Forest Training Pipeline" + " " * 12 + "║")
    print("╚" + "═" * 68 + "╝")
    
    # Step 1: Load Data
    df = load_and_explore_data(CONFIG['data_file'])
    
    # Step 2: Preprocess Data
    (X_train, X_test, y_train, y_test, 
     scaler, label_encoder, feature_names) = preprocess_data(df)
    
    # Step 3: Train Model
    model = train_random_forest(X_train, y_train)
    
    # Step 4: Evaluate Model
    accuracy, cm = evaluate_model(model, X_test, y_test, label_encoder)
    
    # Step 5: Visualizations
    plot_confusion_matrix(cm, label_encoder.classes_)
    visualize_decision_tree(model, feature_names, label_encoder.classes_, tree_index=0)
    plot_feature_importance(model, feature_names)
    
    # Step 6: Save Artifacts
    save_model_artifacts(model, scaler, label_encoder, feature_names)
    
    # Step 7: Test Predictions
    test_single_prediction(model, scaler, label_encoder, feature_names)
    
    print("\n" + "=" * 70)
    print("🎉 TRAINING PIPELINE COMPLETE!")
    print("=" * 70)
    print("\n📁 Generated Files:")
    print("   - solar_fault_rf_model.joblib (trained model)")
    print("   - solar_fault_scaler.joblib (feature scaler)")
    print("   - solar_fault_label_encoder.joblib (label mapping)")
    print("   - confusion_matrix.png (evaluation visual)")
    print("   - decision_tree.png (tree visualization)")
    print("   - feature_importance.png (feature analysis)")
    print("   - model_info.txt (model parameters)")
    print("\n➡️  Next: Run step3_export_to_esp32.py to convert model to C code")
    print("=" * 70)
