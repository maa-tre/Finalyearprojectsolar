/*
 * Solar Panel Fault Detection - Random Forest Model
 * Generated: 2026-01-02 15:41:16
 * Trees: 10, Max Depth: 5
 * 
 * Classes:
 *   0: Normal
 *   1: Open_Circuit
 *   2: Partial_Shading
 *   3: Short_Circuit
 */

#ifndef SOLAR_FAULT_MODEL_H
#define SOLAR_FAULT_MODEL_H

#define NUM_FEATURES 4
#define NUM_CLASSES 4
#define NUM_TREES 10

// Fault type names
const char* CLASS_NAMES[] = {
    "Normal",
    "Open_Circuit",
    "Partial_Shading",
    "Short_Circuit",
};

// Feature names (for debugging)
const char* FEATURE_NAMES[] = {
    "Voltage",
    "Current",
    "Temperature",
    "Light_Intensity",
};

// StandardScaler parameters
const float SCALER_MEAN[] = {
    13.530488f, 3.746800f, 43.080350f, 900.020213f
};

const float SCALER_STD[] = {
    8.256446f, 3.049463f, 14.431640f, 169.050730f
};

// Apply StandardScaler transformation
void scale_features(float* features, float* scaled) {
    for (int i = 0; i < NUM_FEATURES; i++) {
        scaled[i] = (features[i] - SCALER_MEAN[i]) / SCALER_STD[i];
    }
}

// Decision Tree 0
int predict_tree_0(float* features) {
    if (features[1] <= 0.756920f) {
        if (features[0] <= 0.285778f) {
            return 2;
        } else {
            if (features[0] <= 1.016722f) {
                if (features[0] <= 0.755108f) {
                    return 0;
                } else {
                    if (features[1] <= -0.546588f) {
                        return 1;
                    } else {
                        return 0;
                    }
                }
            } else {
                if (features[3] <= 1.355272f) {
                    if (features[0] <= 1.037918f) {
                        return 1;
                    } else {
                        return 1;
                    }
                } else {
                    if (features[2] <= -0.499967f) {
                        return 1;
                    } else {
                        return 0;
                    }
                }
            }
        }
    } else {
        return 3;
    }
}

// Decision Tree 1
int predict_tree_1(float* features) {
    if (features[2] <= 0.530407f) {
        if (features[3] <= -0.691717f) {
            if (features[0] <= 0.454132f) {
                return 2;
            } else {
                return 1;
            }
        } else {
            if (features[2] <= -0.198546f) {
                if (features[0] <= 0.281539f) {
                    return 2;
                } else {
                    if (features[0] <= 0.781149f) {
                        return 0;
                    } else {
                        return 1;
                    }
                }
            } else {
                if (features[1] <= -0.107166f) {
                    return 2;
                } else {
                    if (features[3] <= -0.526737f) {
                        return 0;
                    } else {
                        return 0;
                    }
                }
            }
        }
    } else {
        return 3;
    }
}

// Decision Tree 2
int predict_tree_2(float* features) {
    if (features[0] <= -0.939325f) {
        return 3;
    } else {
        if (features[1] <= -1.025361f) {
            return 1;
        } else {
            if (features[1] <= -0.066176f) {
                return 2;
            } else {
                return 0;
            }
        }
    }
}

// Decision Tree 3
int predict_tree_3(float* features) {
    if (features[1] <= -1.025361f) {
        return 1;
    } else {
        if (features[0] <= 0.285778f) {
            if (features[1] <= 0.276508f) {
                return 2;
            } else {
                return 3;
            }
        } else {
            return 0;
        }
    }
}

// Decision Tree 4
int predict_tree_4(float* features) {
    if (features[1] <= -1.031919f) {
        return 1;
    } else {
        if (features[2] <= 0.530407f) {
            if (features[1] <= -0.066176f) {
                return 2;
            } else {
                if (features[1] <= 0.773317f) {
                    return 0;
                } else {
                    return 0;
                }
            }
        } else {
            return 3;
        }
    }
}

// Decision Tree 5
int predict_tree_5(float* features) {
    if (features[2] <= 0.541841f) {
        if (features[0] <= 0.285778f) {
            if (features[0] <= -0.694668f) {
                return 3;
            } else {
                return 2;
            }
        } else {
            if (features[0] <= 0.755108f) {
                return 0;
            } else {
                if (features[1] <= -0.540030f) {
                    return 1;
                } else {
                    return 0;
                }
            }
        }
    } else {
        return 3;
    }
}

// Decision Tree 6
int predict_tree_6(float* features) {
    if (features[1] <= -1.030280f) {
        return 1;
    } else {
        if (features[0] <= 0.274272f) {
            if (features[1] <= 0.278147f) {
                return 2;
            } else {
                return 3;
            }
        } else {
            return 0;
        }
    }
}

// Decision Tree 7
int predict_tree_7(float* features) {
    if (features[0] <= -0.953254f) {
        return 3;
    } else {
        if (features[1] <= -1.025361f) {
            return 1;
        } else {
            if (features[1] <= -0.056338f) {
                return 2;
            } else {
                return 0;
            }
        }
    }
}

// Decision Tree 8
int predict_tree_8(float* features) {
    if (features[0] <= -0.960521f) {
        return 3;
    } else {
        if (features[1] <= -0.057977f) {
            if (features[1] <= -1.031919f) {
                return 1;
            } else {
                return 2;
            }
        } else {
            return 0;
        }
    }
}

// Decision Tree 9
int predict_tree_9(float* features) {
    if (features[1] <= -0.066176f) {
        if (features[1] <= -1.030280f) {
            return 1;
        } else {
            return 2;
        }
    } else {
        if (features[2] <= 0.339854f) {
            return 0;
        } else {
            return 3;
        }
    }
}

// Main prediction function - returns class index
int predict(float* raw_features) {
    // Scale features
    float scaled[NUM_FEATURES];
    scale_features(raw_features, scaled);
    
    // Vote counting
    int votes[NUM_CLASSES] = {0};
    
    // Get prediction from each tree
    votes[predict_tree_0(scaled)]++;
    votes[predict_tree_1(scaled)]++;
    votes[predict_tree_2(scaled)]++;
    votes[predict_tree_3(scaled)]++;
    votes[predict_tree_4(scaled)]++;
    votes[predict_tree_5(scaled)]++;
    votes[predict_tree_6(scaled)]++;
    votes[predict_tree_7(scaled)]++;
    votes[predict_tree_8(scaled)]++;
    votes[predict_tree_9(scaled)]++;
    
    // Find majority vote
    int max_votes = 0;
    int predicted_class = 0;
    for (int i = 0; i < NUM_CLASSES; i++) {
        if (votes[i] > max_votes) {
            max_votes = votes[i];
            predicted_class = i;
        }
    }
    
    return predicted_class;
}

// Get the class name string from prediction
const char* predict_class_name(float* raw_features) {
    int class_idx = predict(raw_features);
    return CLASS_NAMES[class_idx];
}

// Check if there's a fault (returns 1 if fault, 0 if normal)
int is_fault(float* raw_features) {
    int prediction = predict(raw_features);
    // Class 0 is 'Normal' - return 1 if NOT normal
    return (prediction != 0) ? 1 : 0;
}

#endif // SOLAR_FAULT_MODEL_H