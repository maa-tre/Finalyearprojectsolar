import requests
import json
import time
import threading
import websocket

# Configuration
BACKEND_URL = "http://localhost:8000"
WS_URL = "ws://localhost:8000/ws"

# Mock Data for Sender 1 (Normal)
sender1_data = {
    "senderId": 1,
    "ldrValue": 1000,
    "dhtTemp": 25.5,
    "humidity": 45.0,
    "thermistorTemp": 25.0,
    "voltage": 19.5,
    "current": 5.2,
    "valid": True,
    "gateway_timestamp_ms": int(time.time() * 1000)
}

# Mock Data for Sender 2 (Faulty - Partial Shading)
sender2_data = {
    "senderId": 2,
    "ldrValue": 200,   # Low light
    "dhtTemp": 28.0,
    "humidity": 40.0,
    "thermistorTemp": 30.0,
    "voltage": 12.0,   # Low voltage
    "current": 3.0,
    "valid": True,
    "gateway_timestamp_ms": int(time.time() * 1000)
}

payload = [sender1_data, sender2_data]

def on_message(ws, message):
    data = json.loads(message)
    if data.get("type") == "gateway_data":
        print(f"\n✅ WebSocket Received Data for Station {data.get('sender_id')}")
        print(f"   Prediction: {data.get('prediction', {}).get('fault_type')}")
        
def on_error(ws, error):
    print(f"❌ WebSocket Error: {error}")

def on_close(ws, close_status_code, close_msg):
    print("WebSocket Closed")

def on_open(ws):
    print("✅ WebSocket Connected")
    # Start the POST request in a separate thread after a short delay
    threading.Thread(target=send_post_request).start()

def send_post_request():
    time.sleep(1)
    print("\n🚀 Sending Mock Gateway Data via POST...")
    try:
        response = requests.post(f"{BACKEND_URL}/api/gateway-data", json=payload)
        print(f"POST Response: {response.status_code} - {response.text}")
        
        if response.status_code == 200:
             print("✅ POST Request Successful")
        else:
             print("❌ POST Request Failed")
             
    except Exception as e:
        print(f"❌ Error sending POST: {e}")
        
    # Close WS after a brief wait to allow messages to arrive
    time.sleep(2)
    ws.close()

if __name__ == "__main__":
    print(f"Starting Verification Script...")
    print(f"Backend: {BACKEND_URL}")
    
    # Check if backend is running
    try:
        requests.get(f"{BACKEND_URL}/")
    except:
        print("❌ Backend is not running! Please start 'backend/main.py' first.")
        exit(1)

    # Start WebSocket Client
    ws = websocket.WebSocketApp(WS_URL,
                              on_open=on_open,
                              on_message=on_message,
                              on_error=on_error,
                              on_close=on_close)
    
    ws.run_forever()
