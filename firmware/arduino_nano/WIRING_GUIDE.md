# Arduino Nano Solar Monitor - Wiring Guide

## Required Components
- Arduino Nano (or compatible clone)
- Voltage Sensor Module (0-25V) or DIY voltage divider
- ACS712-05B Current Sensor (5A version)
- LM35 Temperature Sensor
- LDR (Light Dependent Resistor) + 10kΩ resistor
- USB cable for connection to computer

## Pin Connections

### Voltage Sensor (A0)
```
Solar Panel (+) ──┬── 100kΩ ──┬── Arduino A0
                  │           │
                  └── 10kΩ  ──┴── GND
```
This creates an 11:1 voltage divider for measuring up to ~30V

### Current Sensor - ACS712 (A1)
```
ACS712 Module:
- VCC → Arduino 5V
- GND → Arduino GND
- OUT → Arduino A1
- IP+ → Solar Panel positive line (pass through)
- IP- → Load positive connection
```

### Temperature Sensor - LM35 (A2)
```
LM35:
- VCC → Arduino 5V
- GND → Arduino GND
- OUT → Arduino A2
```

### Light Sensor - LDR (A3)
```
5V ── LDR ──┬── Arduino A3
            │
            └── 10kΩ ── GND
```

### Status LED (D13)
Built-in LED, no external wiring needed

## Complete Wiring Diagram
```
                    ┌─────────────────┐
                    │   Arduino Nano  │
                    │                 │
    Voltage Div ────┤ A0          D13 ├──── Status LED (built-in)
    ACS712 OUT  ────┤ A1              │
    LM35 OUT    ────┤ A2          5V  ├──── Sensors VCC
    LDR Div     ────┤ A3         GND  ├──── Sensors GND
                    │                 │
                    │    [USB]        │
                    └───────┬─────────┘
                            │
                      To Computer
```

## Serial Communication
- Baud Rate: 115200
- Data Format: JSON
- Commands:
  - `START` - Begin continuous monitoring
  - `STOP` - Stop monitoring
  - `READ` - Single sensor reading
  - `STATUS` - Get device status
  - `CALIB` - Read raw ADC values for calibration
  - `PING` - Check connection

## Testing
1. Upload the firmware to Arduino Nano
2. Open Serial Monitor at 115200 baud
3. Send `PING` to verify connection
4. Send `READ` to get a single reading
5. Send `START` to begin continuous monitoring
