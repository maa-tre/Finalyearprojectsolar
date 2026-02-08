"""
=============================================================================
🔆 SOLAR PANEL FAULT DETECTION - FastAPI Backend
=============================================================================
Features:
- 5 Fault Classes: Normal, Partial_Shading, Dust_Accumulation, Open_Circuit, Short_Circuit
- Efficiency tracking
- WhatsApp notifications via pywhatkit (uses WhatsApp Web)
- Simulator mode with fault injection
- Serial and WiFi ESP32 support
=============================================================================
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Response
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List, Dict
import numpy as np
import joblib
import json
import asyncio
import random
from datetime import datetime
from contextlib import asynccontextmanager
import serial
import serial.tools.list_ports
import os
import threading

# pywhatkit for WhatsApp (uses WhatsApp Web)
try:
    import pywhatkit as pwk
    PYWHATKIT_AVAILABLE = True
except ImportError:
    PYWHATKIT_AVAILABLE = False
    print("WARNING: pywhatkit not installed. WhatsApp notifications disabled.")

# Fault-specific recommendations
FAULT_RECOMMENDATIONS = {
    "Normal": {
        "message": "✅ System operating normally",
        "action": "No action required. Continue monitoring.",
        "severity": "info"
    },
    "Partial_Shading": {
        "message": "🌤️ Partial Shading Detected!",
        "action": "🔧 ACTION REQUIRED: Remove obstacles (trees, buildings, debris) blocking sunlight from panel. Check for shadows during peak sun hours.",
        "severity": "warning"
    },
    "Dust_Accumulation": {
        "message": "🌫️ Dust Accumulation Detected!",
        "action": "🧹 ACTION REQUIRED: Clean the solar panel surface. Use water and soft cloth. Schedule regular cleaning (weekly in dusty areas).",
        "severity": "warning"
    },
    "Open_Circuit": {
        "message": "🔌 Open Circuit Fault Detected!",
        "action": "⚡ URGENT: Check all cable connections. Inspect junction box. Look for broken wires or loose terminals. Call technician if issue persists.",
        "severity": "critical"
    },
    "Short_Circuit": {
        "message": "⚡ SHORT CIRCUIT DETECTED!",
        "action": "🚨 CRITICAL: Immediately disconnect the panel! Fire hazard. Do NOT touch. Call professional electrician immediately. Check for melted wires or damaged cells.",
        "severity": "danger"
    }
}

# =============================================================================
# MODELS
# =============================================================================
class SensorData(BaseModel):
    voltage: float
    current: float
    temperature: float
    light_intensity: float
    humidity: Optional[float] = 50.0
    thermistor_temp: Optional[float] = 25.0
    efficiency: Optional[float] = None
    relay_status: bool = False

class PredictionResponse(BaseModel):
    fault_type: str
    fault_index: int
    confidence: float
    is_fault: bool
    power: float
    efficiency: float
    timestamp: str
    recommendation: Optional[str] = None

class WiFiNetwork(BaseModel):
    ssid: str
    password: str

class ConnectionConfig(BaseModel):
    mode: str  # "simulator", "serial", "wifi"
    port: Optional[str] = None
    baudrate: Optional[int] = 115200
    esp32_ip: Optional[str] = None

class WhatsAppConfig(BaseModel):
    phone_number: str
    enabled: bool = True

class GatewayRecord(BaseModel):
    senderId: int
    ldrValue: int
    dhtTemp: float
    humidity: float
    thermistorTemp: float
    voltage: float
    current: float
    relayStatus: bool = False
    valid: bool
    gateway_timestamp_ms: int


# =============================================================================
# GLOBAL STATE
# =============================================================================
class AppState:
    def __init__(self):
        self.model = None
        self.scaler = None
        self.label_encoder = None
        self.model_loaded = False
        self.connection_mode = "simulator"
        self.serial_connection = None
        self.esp32_ip = None
        self.simulation_fault_type = 0  # 0=Normal, 1=Open, 2=Partial, 3=Short, 4=Dust
        self.connected_clients: List[WebSocket] = []
        self.is_monitoring = False
        self.simulated_relay_states: Dict[int, bool] = {1: False, 2: False}
        
        # WhatsApp notification tracking
        self.whatsapp_enabled = False
        self.whatsapp_number = ""
        self.last_notified_fault = None  # Track last fault to avoid spam
        self.last_notification_time = None
        self.last_notified_fault = None  # Track last fault to avoid spam
        self.last_notification_time = None
        self.notification_cooldown = 60  # Minimum seconds between same fault notifications
        
        # Command Queue for Gateway
        self.pending_commands: Dict[str, str] = {}
        self.commands_lock = threading.Lock()
        
    def load_model(self):
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        models_dir = os.path.join(base_dir, 'models')
        try:
            self.model = joblib.load(os.path.join(models_dir, 'solar_fault_rf_model.joblib'))
            self.scaler = joblib.load(os.path.join(models_dir, 'solar_fault_scaler.joblib'))
            self.label_encoder = joblib.load(os.path.join(models_dir, 'solar_fault_label_encoder.joblib'))
            self.model_loaded = True
            print("✅ Model loaded successfully")
            print(f"   Classes: {list(self.label_encoder.classes_)}")
        except Exception as e:
            print(f"❌ Failed to load model: {e}")
            self.model_loaded = False

state = AppState()

# =============================================================================
# LIFESPAN
# =============================================================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    state.load_model()
    yield
    if state.serial_connection:
        state.serial_connection.close()

# =============================================================================
# APP INITIALIZATION
# =============================================================================
app = FastAPI(
    title="Solar Panel Fault Detection API",
    description="Real-time ML-powered fault detection with WhatsApp notifications",
    version="2.0.0",
    lifespan=lifespan
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# =============================================================================
# SIMULATOR - 5 FAULT PROFILES
# =============================================================================
FAULT_PROFILES = {
    0: {"name": "Normal", "voltage": (17, 22), "current": (4, 6), "temp": (25, 45), "light": (800, 1200), "efficiency": (15, 22)},
    1: {"name": "Open_Circuit", "voltage": (20, 25), "current": (0, 0.15), "temp": (25, 40), "light": (700, 1200), "efficiency": (0, 2)},
    2: {"name": "Partial_Shading", "voltage": (8, 15), "current": (1, 3.5), "temp": (30, 55), "light": (150, 450), "efficiency": (5, 12)},
    3: {"name": "Short_Circuit", "voltage": (0, 4), "current": (6, 10), "temp": (55, 85), "light": (500, 1200), "efficiency": (0, 3)},
    4: {"name": "Dust_Accumulation", "voltage": (14, 19), "current": (3, 5), "temp": (35, 55), "light": (400, 700), "efficiency": (10, 16)},
}

def generate_simulated_data(fault_type: int = 0) -> dict:
    """Generate realistic simulated sensor data for a given fault type."""
    profile = FAULT_PROFILES.get(fault_type, FAULT_PROFILES[0])
    
    voltage = random.uniform(*profile["voltage"]) + random.gauss(0, 0.5)
    current = max(0, random.uniform(*profile["current"]) + random.gauss(0, 0.1))
    temperature = random.uniform(*profile["temp"]) + random.gauss(0, 1)
    light = max(0, random.uniform(*profile["light"]) + random.gauss(0, 20))
    efficiency = random.uniform(*profile["efficiency"]) + random.gauss(0, 0.5)
    efficiency = max(0, min(25, efficiency))  # Clip to realistic range
    
    return {
        "voltage": round(voltage, 2),
        "current": round(current, 2),
        "temperature": round(temperature, 2),
        "light_intensity": round(light, 2),
        "humidity": round(random.uniform(30, 80), 1),
        "thermistor_temp": round(temperature + random.uniform(-2, 2), 2),
        "efficiency": round(efficiency, 2),
        "relay_status": False
    }

# =============================================================================
# WHATSAPP NOTIFICATION (using pywhatkit - WhatsApp Web)
# =============================================================================
# Global lock to prevent multiple simultaneous WhatsApp sends
whatsapp_send_lock = threading.Lock()
whatsapp_sending = False

def send_whatsapp_sync(phone: str, message: str):
    """
    Send WhatsApp message synchronously using pywhatkit.
    This runs in a separate thread to not block the async event loop.
    
    IMPORTANT: You must be logged into WhatsApp Web in your browser!
    First time: Go to web.whatsapp.com and scan QR code with your phone.
    """
    global whatsapp_sending
    
    if not PYWHATKIT_AVAILABLE:
        print("❌ pywhatkit not available")
        return False
    
    # Prevent multiple sends at once
    with whatsapp_send_lock:
        if whatsapp_sending:
            print("⏳ WhatsApp send already in progress, skipping...")
            return False
        whatsapp_sending = True
    
    try:
        # sendwhatmsg_instantly sends message immediately
        # wait_time: seconds to wait for WhatsApp Web to load
        # tab_close: DON'T close tab - let user manage it
        # close_time: not used since tab_close=False
        pwk.sendwhatmsg_instantly(
            phone_no=phone,
            message=message,
            wait_time=20,       # Wait 20 seconds for WhatsApp Web to load
            tab_close=False,    # DON'T close tab - less intrusive
            close_time=5
        )
        print(f"✅ WhatsApp message sent to {phone}")
        return True
    except Exception as e:
        print(f"❌ WhatsApp error: {e}")
        return False
    finally:
        whatsapp_sending = False

async def send_whatsapp_notification(fault_type: str, sensor_data: dict, is_simulator: bool = False) -> bool:
    """
    Send WhatsApp notification using pywhatkit (WhatsApp Web).
    
    PREREQUISITE: Must be logged into WhatsApp Web in your default browser!
    Go to web.whatsapp.com and scan QR code with your phone first.
    
    Args:
        fault_type: The detected fault type
        sensor_data: Current sensor readings
        is_simulator: If True, only send once per fault type change
    
    Returns:
        True if message sent successfully
    """
    global whatsapp_sending
    
    # Skip if notifications disabled or no phone number
    if not state.whatsapp_enabled or not state.whatsapp_number:
        print(f"⚠️ WhatsApp skipped: enabled={state.whatsapp_enabled}, number={bool(state.whatsapp_number)}")
        return False
    
    # Skip if already sending a message
    if whatsapp_sending:
        print("⏳ Already sending WhatsApp, skipping...")
        return False
    
    # Skip Normal - no notification needed
    if fault_type == "Normal":
        # Reset last notified when back to normal
        state.last_notified_fault = None
        return False
    
    # For simulator mode: only notify once per fault type change
    if is_simulator and state.last_notified_fault == fault_type:
        return False
    
    # Cooldown: 30 seconds between SAME fault type notifications (not all notifications)
    if state.last_notification_time and state.last_notified_fault == fault_type:
        elapsed = (datetime.now() - state.last_notification_time).total_seconds()
        if elapsed < 30:  # 30 second cooldown for same fault
            return False
    
    print(f"📱 Sending WhatsApp notification for: {fault_type}")
    
    # Get recommendation
    rec = FAULT_RECOMMENDATIONS.get(fault_type, FAULT_RECOMMENDATIONS["Normal"])
    
    # Build message (shorter for WhatsApp)
    message = f"""🔆 SOLAR PANEL ALERT 🔆

{rec['message']}

📊 Readings:
• Voltage: {sensor_data.get('voltage', 'N/A')} V
• Current: {sensor_data.get('current', 'N/A')} A
• Temp: {sensor_data.get('temperature', 'N/A')} °C
• Light: {sensor_data.get('light_intensity', 'N/A')} lux
• Efficiency: {sensor_data.get('efficiency', 'N/A')}%

{rec['action']}

⏰ {datetime.now().strftime('%H:%M:%S')}"""
    
    # Send in background thread (pywhatkit is blocking)
    phone = state.whatsapp_number
    if not phone.startswith("+"):
        phone = "+" + phone
    
    # Run in thread to not block async loop
    thread = threading.Thread(target=send_whatsapp_sync, args=(phone, message))
    thread.start()
    
    # Update state
    state.last_notified_fault = fault_type
    state.last_notification_time = datetime.now()
    print(f"📱 WhatsApp notification queued for: {fault_type}")
    
    return True

# =============================================================================
# PREDICTION
# =============================================================================
def calculate_efficiency(voltage: float, current: float, light: float) -> float:
    """Calculate panel efficiency from sensor readings."""
    power_output = voltage * current
    irradiance = light * 0.0079  # lux to W/m² approximation
    solar_input = irradiance * 1.6  # 1.6 m² panel area
    
    if solar_input > 0:
        efficiency = (power_output / solar_input) * 100
        return round(min(25, max(0, efficiency)), 2)
    return 0.0

def predict_fault(data: SensorData) -> PredictionResponse:
    """Make fault prediction using the ML model."""
    if not state.model_loaded:
        raise HTTPException(status_code=500, detail="Model not loaded")
    
    # Calculate efficiency if not provided
    efficiency = data.efficiency
    if efficiency is None:
        efficiency = calculate_efficiency(data.voltage, data.current, data.light_intensity)
    
    # Prepare features (now includes efficiency)
    features = np.array([[
        data.voltage, 
        data.current, 
        data.temperature, 
        data.light_intensity,
        efficiency
    ]])
    features_scaled = state.scaler.transform(features)
    
    prediction = state.model.predict(features_scaled)[0]
    proba = state.model.predict_proba(features_scaled)[0]
    
    fault_type = state.label_encoder.inverse_transform([prediction])[0]
    confidence = float(max(proba) * 100)
    
    # Get recommendation
    rec = FAULT_RECOMMENDATIONS.get(fault_type, FAULT_RECOMMENDATIONS["Normal"])
    
    return PredictionResponse(
        fault_type=fault_type,
        fault_index=int(prediction),
        confidence=confidence,
        is_fault=(fault_type != "Normal"),
        power=round(data.voltage * data.current, 2),
        efficiency=efficiency,
        timestamp=datetime.now().isoformat(),
        recommendation=rec["action"]
    )

# =============================================================================
# REST ENDPOINTS
# =============================================================================
@app.get("/")
async def root():
    return {"message": "Solar Panel Fault Detection API v2.0", "status": "running"}

@app.get("/api/status")
async def get_status():
    return {
        "model_loaded": state.model_loaded,
        "connection_mode": state.connection_mode,
        "is_monitoring": state.is_monitoring,
        "connected_clients": len(state.connected_clients),
        "simulation_fault_type": state.simulation_fault_type,
        "whatsapp_enabled": state.whatsapp_enabled,
        "whatsapp_configured": bool(state.whatsapp_number)
    }

@app.post("/api/predict")
async def predict(data: SensorData):
    return predict_fault(data)

@app.get("/api/simulate")
async def get_simulated_data(fault_type: int = 0):
    data = generate_simulated_data(fault_type)
    sensor_data = SensorData(**data)
    prediction = predict_fault(sensor_data)
    return {"sensor_data": data, "prediction": prediction}

@app.post("/api/set-simulation-mode")
async def set_simulation_mode(fault_type: int):
    if fault_type < 0 or fault_type > 4:  # Now 5 fault types (0-4)
        raise HTTPException(status_code=400, detail="Invalid fault type (0-4)")
    state.simulation_fault_type = fault_type
    return {"status": "ok", "fault_type": fault_type, "name": FAULT_PROFILES[fault_type]["name"]}

@app.post("/api/gateway-data")
async def receive_gateway_data(records: List[GatewayRecord]):
    """
    Endpoint to receive aggregated data from ESP32 Gateway.
    Processes multiple records, runs ML predictions, and broadcasts via WebSocket.
    """
    processed_count = 0
    
    for record in records:
        if not record.valid:
            continue
            
        # Convert to standard SensorData
        # Calculate efficiency dynamically
        efficiency = calculate_efficiency(record.voltage, record.current, float(record.ldrValue))
        
        sensor_data = SensorData(
            voltage=record.voltage,
            current=record.current,
            temperature=record.dhtTemp,
            light_intensity=float(record.ldrValue),
            humidity=record.humidity,
            thermistor_temp=record.thermistorTemp,
            efficiency=efficiency,
            relay_status=record.relayStatus
        )
        
        # Run Prediction
        prediction = predict_fault(sensor_data)
        
        # Send WhatsApp if fault detected (and enabled)
        if prediction.is_fault:
            await send_whatsapp_notification(
                prediction.fault_type, 
                sensor_data.model_dump(), 
                is_simulator=False
            )
            
        # Broadcast to WebSocket Clients
        # alerting frontend that this is from a specific sender
        payload = {
            "type": "gateway_data",
            "sender_id": record.senderId,
            "sensor_data": sensor_data.model_dump(),
            "prediction": prediction.model_dump(),
            "timestamp": record.gateway_timestamp_ms
        }
        
        # Broadcast
        for client in state.connected_clients:
            try:
                await client.send_json(payload)
            except:
                pass # Handle disconnected clients
                
        processed_count += 1
        
    return {"status": "success", "processed": processed_count}

@app.get("/api/serial-ports")
async def list_serial_ports():
    ports = [{"port": p.device, "description": p.description} for p in serial.tools.list_ports.comports()]
    return {"ports": ports}

@app.post("/api/connect")
async def connect(config: ConnectionConfig):
    state.connection_mode = config.mode
    
    if config.mode == "serial" and config.port:
        try:
            if state.serial_connection:
                state.serial_connection.close()
            state.serial_connection = serial.Serial(config.port, config.baudrate, timeout=1)
            return {"status": "connected", "mode": "serial", "port": config.port}
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))
    
    elif config.mode == "wifi" and config.esp32_ip:
        state.esp32_ip = config.esp32_ip
        return {"status": "connected", "mode": "wifi", "ip": config.esp32_ip}
    
    elif config.mode == "simulator":
        return {"status": "connected", "mode": "simulator"}
    
    return {"status": "ok", "mode": config.mode}

@app.post("/api/disconnect")
async def disconnect():
    if state.serial_connection:
        state.serial_connection.close()
        state.serial_connection = None
    state.connection_mode = "simulator"
    return {"status": "disconnected"}

@app.get("/api/fault-types")
async def get_fault_types():
    return {
        "fault_types": [
            {"id": 0, "name": "Normal", "color": "#22c55e", "icon": "✅", "description": "System operating normally"},
            {"id": 1, "name": "Open_Circuit", "color": "#a855f7", "icon": "🔌", "description": "Circuit connection broken"},
            {"id": 2, "name": "Partial_Shading", "color": "#f59e0b", "icon": "🌤️", "description": "Shadows blocking sunlight"},
            {"id": 3, "name": "Short_Circuit", "color": "#ef4444", "icon": "⚡", "description": "Critical short circuit"},
            {"id": 4, "name": "Dust_Accumulation", "color": "#78716c", "icon": "🌫️", "description": "Dust reducing efficiency"}
        ]
    }

@app.get("/api/recommendations")
async def get_recommendations():
    return {"recommendations": FAULT_RECOMMENDATIONS}

# =============================================================================
# WHATSAPP ENDPOINTS
# =============================================================================
@app.post("/api/whatsapp/configure")
async def configure_whatsapp(config: WhatsAppConfig):
    """Configure WhatsApp notifications."""
    # Clean phone number (remove spaces, dashes)
    phone = config.phone_number.replace(" ", "").replace("-", "")
    if not phone.startswith("+"):
        phone = "+" + phone
    
    state.whatsapp_number = phone
    state.whatsapp_enabled = config.enabled
    state.last_notified_fault = None  # Reset notification tracking
    
    return {
        "status": "configured",
        "phone_number": phone,
        "enabled": config.enabled
    }

# =============================================================================
# COMMAND API (For Bi-directional Communication)
# =============================================================================
class CommandRequest(BaseModel):
    station_id: int
    command: str

@app.post("/api/command")
async def queue_command(request: CommandRequest):
    """Queue a command for a specific station (e.g., TOGGLE_RELAY)."""
    station_id = str(request.station_id)
    
    with state.commands_lock:
        state.pending_commands[station_id] = request.command
    
    # If in simulator mode, apply command immediately to simulated state
    if state.connection_mode == "simulator":
        if request.command == "ACTIVATE_RELAY":
            state.simulated_relay_states[request.station_id] = True
        elif request.command == "DEACTIVATE_RELAY":
            state.simulated_relay_states[request.station_id] = False
        elif request.command == "TOGGLE_RELAY":
            state.simulated_relay_states[request.station_id] = not state.simulated_relay_states.get(request.station_id, False)

    print(f"🕹️ Command Queued for Station {station_id}: {request.command}")
    return {"status": "success", "message": "Command queued", "station_id": station_id}

@app.get("/api/get-command/{station_id}")
async def get_pending_command(station_id: str):
    """Poll endpoint for Gateway to check for pending commands."""
    command = None
    with state.commands_lock:
        if station_id in state.pending_commands:
            command = state.pending_commands.pop(station_id)
            print(f"🚀 Command Sent to Gateway for Station {station_id}: {command}")
    
    if command:
        return {"station_id": int(station_id), "command": command}
    else:
        # Return 204 No Content if no command
        return Response(status_code=204)    


@app.post("/api/whatsapp/test")
async def test_whatsapp():
    """Send a test WhatsApp message - doesn't affect cooldown for real notifications."""
    global whatsapp_sending
    
    if not state.whatsapp_number:
        raise HTTPException(status_code=400, detail="WhatsApp number not configured")
    
    if whatsapp_sending:
        raise HTTPException(status_code=429, detail="Already sending a message, please wait")
    
    test_data = {
        "voltage": 20.0,
        "current": 5.0,
        "temperature": 35.0,
        "light_intensity": 1000.0,
        "efficiency": 18.0
    }
    
    # Build test message
    message = f"""🧪 TEST MESSAGE 🧪

🔆 Solar Panel Monitoring Active!

📊 Sample Readings:
• Voltage: {test_data['voltage']} V
• Current: {test_data['current']} A
• Temp: {test_data['temperature']} °C
• Light: {test_data['light_intensity']} lux
• Efficiency: {test_data['efficiency']}%

✅ WhatsApp notifications are working!

⏰ {datetime.now().strftime('%H:%M:%S')}"""
    
    phone = state.whatsapp_number
    if not phone.startswith("+"):
        phone = "+" + phone
    
    # Send test directly without affecting notification state
    thread = threading.Thread(target=send_whatsapp_sync, args=(phone, message))
    thread.start()
    
    # DON'T update last_notified_fault or last_notification_time for test
    print(f"📱 Test WhatsApp message queued to {phone}")
    
    return {"status": "sent", "message": "Test notification sent"}

@app.get("/api/whatsapp/status")
async def whatsapp_status():
    """Get WhatsApp notification status."""
    return {
        "enabled": state.whatsapp_enabled,
        "configured": bool(state.whatsapp_number),
        "phone_number": state.whatsapp_number[:4] + "****" + state.whatsapp_number[-4:] if state.whatsapp_number else None,
        "last_notified_fault": state.last_notified_fault,
        "last_notification_time": state.last_notification_time.isoformat() if state.last_notification_time else None,
        "pywhatkit_available": PYWHATKIT_AVAILABLE,
        "note": "Make sure you're logged into WhatsApp Web (web.whatsapp.com) in your browser!"
    }

# =============================================================================
# WEBSOCKET FOR REAL-TIME DATA
# =============================================================================
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    state.connected_clients.append(websocket)
    
    try:
        while True:
            # Receive commands from client
            try:
                message = await asyncio.wait_for(websocket.receive_text(), timeout=0.1)
                data = json.loads(message)
                
                if data.get("command") == "start":
                    state.is_monitoring = True
                elif data.get("command") == "stop":
                    state.is_monitoring = False
                elif data.get("command") == "set_fault":
                    new_fault = data.get("fault_type", 0)
                    # If fault type changes in simulator, reset notification
                    if new_fault != state.simulation_fault_type:
                        state.last_notified_fault = None
                    state.simulation_fault_type = new_fault
                elif data.get("command") == "configure_whatsapp":
                    state.whatsapp_number = data.get("phone", "")
                    state.whatsapp_enabled = data.get("enabled", True)
                    
            except asyncio.TimeoutError:
                pass
            
            # Send data if monitoring
            if state.is_monitoring:
                if state.connection_mode == "simulator":
                    # Broadcast for both Station 1 and Station 2
                    for station_id in [1, 2]:
                        sensor_data = generate_simulated_data(state.simulation_fault_type)
                        # Use simulated relay state
                        sensor_data["relay_status"] = state.simulated_relay_states.get(station_id, False)
                        
                        prediction = predict_fault(SensorData(**sensor_data))
                        
                        # Send WhatsApp if fault detected (limited to last notified logic)
                        if prediction.is_fault:
                            await send_whatsapp_notification(prediction.fault_type, sensor_data, is_simulator=True)
                        elif not prediction.is_fault:
                            state.last_notified_fault = None
                        
                        await websocket.send_json({
                            "type": "data",
                            "sender_id": station_id,
                            "sensor_data": sensor_data,
                            "prediction": prediction.model_dump()
                        })
                elif state.connection_mode == "serial" and state.serial_connection:
                    sensor_data = read_serial_data()
                    prediction = predict_fault(SensorData(**sensor_data))
                    await websocket.send_json({
                        "type": "data",
                        "sender_id": 1,
                        "sensor_data": sensor_data,
                        "prediction": prediction.model_dump()
                    })
            
            await asyncio.sleep(0.5)  # 2 updates per second
            
    except WebSocketDisconnect:
        state.connected_clients.remove(websocket)

def read_serial_data() -> dict:
    """Read data from serial connection."""
    if not state.serial_connection:
        return generate_simulated_data(0)
    
    try:
        state.serial_connection.write(b"GET_DATA\n")
        line = state.serial_connection.readline().decode().strip()
        
        if line.startswith("DATA:"):
            parts = line[5:].split(",")
            voltage = float(parts[0])
            current = float(parts[1])
            temp = float(parts[2])
            light = float(parts[3])
            efficiency = float(parts[4]) if len(parts) > 4 else calculate_efficiency(voltage, current, light)
            
            return {
                "voltage": voltage,
                "current": current,
                "temperature": temp,
                "light_intensity": light,
                "efficiency": efficiency
            }
    except:
        pass
    
    return generate_simulated_data(0)

# =============================================================================
# RUN SERVER
# =============================================================================
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
