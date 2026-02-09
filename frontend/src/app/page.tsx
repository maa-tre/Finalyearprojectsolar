'use client'

import { useState, useEffect, useCallback } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import {
  Sun, Zap, Thermometer, Eye, Activity, Wifi, Usb,
  Play, Pause, Settings, AlertTriangle, CheckCircle,
  TrendingUp, Battery, Radio, RefreshCw, MessageCircle, Percent
} from 'lucide-react'
import { LineChart, Line, XAxis, YAxis, ResponsiveContainer, AreaChart, Area, PieChart, Pie, Cell } from 'recharts'

// Types
interface SensorData {
  voltage: number
  current: number
  temperature: number
  light_intensity: number
  humidity: number
  thermistor_temp: number
  efficiency: number
  relay_status: boolean
}

interface Prediction {
  fault_type: string
  fault_index: number
  confidence: number
  is_fault: boolean
  power: number
  efficiency: number
  timestamp: string
  recommendation?: string
}

interface HistoryEntry {
  time: string
  voltage: number
  current: number
  temperature: number
  light: number
  humidity: number
  thermistor: number
  power: number
  efficiency: number
  relay: boolean
  fault: string
}

// Fault type colors and info (5 classes including Dust)
const FAULT_INFO: Record<string, { color: string, gradient: string, icon: string, description: string }> = {
  'Normal': {
    color: '#22c55e',
    gradient: 'from-green-500 to-emerald-500',
    icon: '✅',
    description: 'System operating normally'
  },
  'Open_Circuit': {
    color: '#a855f7',
    gradient: 'from-purple-500 to-violet-500',
    icon: '🔌',
    description: 'Circuit connection broken'
  },
  'Partial_Shading': {
    color: '#f59e0b',
    gradient: 'from-amber-500 to-orange-500',
    icon: '🌤️',
    description: 'Shadows blocking sunlight'
  },
  'Short_Circuit': {
    color: '#ef4444',
    gradient: 'from-red-500 to-rose-500',
    icon: '⚡',
    description: 'Critical: Short circuit detected'
  },
  'Dust_Accumulation': {
    color: '#78716c',
    gradient: 'from-stone-500 to-stone-600',
    icon: '🌫️',
    description: 'Dust reducing panel efficiency'
  },
}

// Gauge Component
function Gauge({ value, min, max, label, unit, color, icon: Icon }: {
  value: number
  min: number
  max: number
  label: string
  unit: string
  color: string
  icon: any
}) {
  const percentage = Math.min(100, Math.max(0, ((value - min) / (max - min)) * 100))
  const circumference = 2 * Math.PI * 45
  const strokeDasharray = `${(percentage / 100) * circumference} ${circumference}`

  return (
    <motion.div
      className="glass-card p-6 flex flex-col items-center"
      whileHover={{ scale: 1.02 }}
      transition={{ type: "spring", stiffness: 300 }}
    >
      <div className="relative w-32 h-32">
        <svg className="w-full h-full -rotate-90" viewBox="0 0 100 100">
          {/* Background circle */}
          <circle
            cx="50"
            cy="50"
            r="45"
            stroke="rgba(255,255,255,0.1)"
            strokeWidth="8"
            fill="none"
          />
          {/* Progress circle */}
          <motion.circle
            cx="50"
            cy="50"
            r="45"
            stroke={color}
            strokeWidth="8"
            fill="none"
            strokeLinecap="round"
            initial={{ strokeDasharray: `0 ${circumference}` }}
            animate={{ strokeDasharray }}
            transition={{ duration: 0.5, ease: "easeOut" }}
            style={{ filter: `drop-shadow(0 0 10px ${color})` }}
          />
        </svg>
        <div className="absolute inset-0 flex flex-col items-center justify-center">
          <Icon className="w-5 h-5 mb-1" style={{ color }} />
          <span className="text-2xl font-bold">{value.toFixed(1)}</span>
          <span className="text-xs text-gray-400">{unit}</span>
        </div>
      </div>
      <span className="mt-3 text-sm font-medium text-gray-300">{label}</span>
    </motion.div>
  )
}

// Status Card Component
function StatusCard({ prediction }: { prediction: Prediction | null }) {
  if (!prediction) return null

  const info = FAULT_INFO[prediction.fault_type] || FAULT_INFO['Normal']

  return (
    <motion.div
      className={`glass-card p-8 border-2 ${prediction.is_fault ? 'status-fault' : 'status-normal'}`}
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      key={prediction.fault_type}
    >
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-4">
          <motion.div
            className={`w-20 h-20 rounded-2xl bg-gradient-to-br ${info.gradient} flex items-center justify-center text-4xl`}
            animate={prediction.is_fault ? { scale: [1, 1.1, 1] } : {}}
            transition={{ repeat: Infinity, duration: 1 }}
          >
            {info.icon}
          </motion.div>
          <div>
            <h2 className="text-3xl font-bold">{prediction.fault_type.replace('_', ' ')}</h2>
            <p className="text-gray-400 mt-1">{info.description}</p>
          </div>
        </div>
        <div className="text-right">
          <div className="text-5xl font-bold gradient-text">{prediction.confidence.toFixed(1)}%</div>
          <div className="text-gray-400 mt-1">Confidence</div>
        </div>
      </div>

      <div className="mt-6 flex gap-8">
        <div className="flex items-center gap-2">
          <Battery className="w-5 h-5 text-solar-400" />
          <span className="text-gray-300">Power Output:</span>
          <span className="font-bold text-white">{prediction.power.toFixed(1)} W</span>
        </div>
        <div className="flex items-center gap-2">
          <Activity className="w-5 h-5 text-green-400" />
          <span className="text-gray-300">Status:</span>
          <span className={`font-bold ${prediction.is_fault ? 'text-red-400' : 'text-green-400'}`}>
            {prediction.is_fault ? 'FAULT DETECTED' : 'OPERATIONAL'}
          </span>
        </div>
      </div>
    </motion.div>
  )
}

// Connection Panel Component
function ConnectionPanel({
  mode,
  setMode,
  isConnected,
  simulationFault,
  setSimulationFault
}: {
  mode: string
  setMode: (m: string) => void
  isConnected: boolean
  simulationFault: number
  setSimulationFault: (f: number) => void
}) {
  const modes = [
    { id: 'simulator', label: 'Simulator', icon: RefreshCw, color: 'text-blue-400' },
    { id: 'serial', label: 'USB/Serial', icon: Usb, color: 'text-green-400' },
    { id: 'wifi', label: 'WiFi/ESP32', icon: Wifi, color: 'text-purple-400' },
  ]

  const faultTypes = [
    { id: 0, name: 'Normal', color: 'bg-green-500' },
    { id: 1, name: 'Open Circuit', color: 'bg-purple-500' },
    { id: 2, name: 'Partial Shading', color: 'bg-amber-500' },
    { id: 3, name: 'Short Circuit', color: 'bg-red-500' },
    { id: 4, name: 'Dust Accumulation', color: 'bg-stone-500' },
  ]

  return (
    <div className="glass-card p-6">
      <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
        <Settings className="w-5 h-5 text-solar-400" />
        Connection Mode
      </h3>

      <div className="grid grid-cols-3 gap-3 mb-6">
        {modes.map(m => (
          <button
            key={m.id}
            onClick={() => setMode(m.id)}
            className={`p-4 rounded-xl border transition-all ${mode === m.id
              ? 'bg-white/10 border-solar-500 shadow-lg shadow-solar-500/20'
              : 'border-white/10 hover:bg-white/5'
              }`}
          >
            <m.icon className={`w-6 h-6 mx-auto mb-2 ${m.color}`} />
            <div className="text-sm font-medium">{m.label}</div>
          </button>
        ))}
      </div>

      {mode === 'simulator' && (
        <div>
          <h4 className="text-sm font-medium text-gray-400 mb-3">Simulate Fault Type</h4>
          <div className="grid grid-cols-2 gap-2">
            {faultTypes.map(f => (
              <button
                key={f.id}
                onClick={() => setSimulationFault(f.id)}
                className={`p-3 rounded-lg border transition-all flex items-center gap-2 ${simulationFault === f.id
                  ? 'bg-white/10 border-white/30'
                  : 'border-white/10 hover:bg-white/5'
                  }`}
              >
                <div className={`w-3 h-3 rounded-full ${f.color}`} />
                <span className="text-sm">{f.name}</span>
              </button>
            ))}
          </div>
        </div>
      )}

      {mode === 'serial' && (
        <div>
          <label className="text-sm font-medium text-gray-400">COM Port</label>
          <select className="w-full mt-2 p-3 rounded-lg bg-white/5 border border-white/10 text-white">
            <option value="">Select Port...</option>
            <option value="COM3">COM3 - Arduino Nano</option>
            <option value="COM4">COM4 - USB Device</option>
          </select>
        </div>
      )}

      {mode === 'wifi' && (
        <div className="space-y-4">
          <div className="p-4 rounded-xl bg-white/5 border border-white/10">
            <div className="flex items-center gap-2 mb-2">
              <Radio className="w-4 h-4 text-purple-400" />
              <label className="text-xs font-medium text-gray-400 uppercase tracking-wider">Central Node MAC</label>
            </div>
            <div className="text-lg font-mono font-bold text-white tracking-widest">
              88:57:21:8E:C2:BC
            </div>
            <p className="text-[10px] text-gray-500 mt-2">ESP-NOW protocol active on Channel 1</p>
          </div>
        </div>
      )}
    </div>
  )
}

// Mini Chart Component
function MiniChart({ data, dataKey, color, title }: {
  data: HistoryEntry[]
  dataKey: string
  color: string
  title: string
}) {
  return (
    <div className="glass-card p-4">
      <h4 className="text-sm font-medium text-gray-400 mb-3">{title}</h4>
      <div className="h-24">
        <ResponsiveContainer width="100%" height="100%">
          <AreaChart data={data.slice(-20)}>
            <defs>
              <linearGradient id={`gradient-${dataKey}`} x1="0" y1="0" x2="0" y2="1">
                <stop offset="0%" stopColor={color} stopOpacity={0.3} />
                <stop offset="100%" stopColor={color} stopOpacity={0} />
              </linearGradient>
            </defs>
            <Area
              type="monotone"
              dataKey={dataKey}
              stroke={color}
              strokeWidth={2}
              fill={`url(#gradient-${dataKey})`}
            />
          </AreaChart>
        </ResponsiveContainer>
      </div>
    </div>
  )
}

interface StationState {
  sensorData: SensorData | null
  prediction: Prediction | null
  history: HistoryEntry[]
  faultCounts: Record<string, number>
  lastUpdate: Date
}

// Main Page Component
export default function Home() {
  const [isMonitoring, setIsMonitoring] = useState(false)
  const [connectionMode, setConnectionMode] = useState('simulator')
  const [isConnected, setIsConnected] = useState(false)
  const [simulationFault, setSimulationFault] = useState(0)

  // Multi-station state
  const [selectedStation, setSelectedStation] = useState<number>(1)
  const [availableStations, setAvailableStations] = useState<Set<number>>(new Set([1]))
  const [stationData, setStationData] = useState<Record<number, StationState>>({})

  // Debug: Track relay command status
  const [lastRelayCommand, setLastRelayCommand] = useState<{time: string, station: number, command: string, status: string}>({
    time: '-',
    station: 1,
    command: '-',
    status: 'idle'
  })

  // Compute current display data from selected station
  const currentStation = stationData[selectedStation] || {
    sensorData: null,
    prediction: null,
    history: [],
    faultCounts: {},
    lastUpdate: new Date()
  }

  const { sensorData, prediction, history, faultCounts } = currentStation

  // WhatsApp notification state
  const [whatsappNumber, setWhatsappNumber] = useState('')
  const [whatsappEnabled, setWhatsappEnabled] = useState(false)
  const [whatsappStatus, setWhatsappStatus] = useState<string>('')

  // Reset data when mode or monitoring changes
  useEffect(() => {
    if (!isMonitoring) {
      setStationData({})
      setAvailableStations(new Set([1]))
    }
  }, [connectionMode, isMonitoring])

  // WebSocket connection and Stale Data Handling
  useEffect(() => {
    if (!isMonitoring) return

    const ws = new WebSocket('ws://localhost:8000/ws')

    // Interval to check for stale data (Timeout: 5s)
    const staleCheck = setInterval(() => {
      setStationData(prev => {
        const now = new Date().getTime()
        const updatedState = { ...prev }
        let changed = false

        Object.keys(updatedState).forEach(id => {
          const sId = parseInt(id)
          const lastUpdate = updatedState[sId]?.lastUpdate?.getTime() || 0
          if (now - lastUpdate > 5000 && updatedState[sId].sensorData !== null) {
            updatedState[sId] = { ...updatedState[sId], sensorData: null, prediction: null }
            changed = true
          }
        })
        return changed ? updatedState : prev
      })
    }, 1000)

    ws.onopen = () => {
      console.log('WebSocket connected')
      ws.send(JSON.stringify({ command: 'start' }))
    }

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data)

      // --- IMMEDIATE RELAY FEEDBACK (All modes) ---
      if (data.type === 'relay_command_acknowledged') {
        console.log(`✓ Relay command acknowledged for Station ${data.station_id}: ${data.command}`)
        console.log(`  → New state: ${data.status_text}`)
        
        setStationData(prev => {
          const current = prev[data.station_id] || {
            sensorData: null,
            prediction: null,
            history: [],
            faultCounts: {},
            lastUpdate: new Date()
          }
          
          // Update sensor data with new relay status
          const updatedSensorData = current.sensorData ? { ...current.sensorData } : null
          if (updatedSensorData) {
            updatedSensorData.relay_status = data.relay_status
          }

          return {
            ...prev,
            [data.station_id]: {
              ...current,
              sensorData: updatedSensorData || current.sensorData
            }
          }
        })
        return // Don't process as normal data
      }

      // --- STRICT MODE FILTERING ---
      let targetStationId = 1
      let incomingSensorData = data.sensor_data
      let incomingPrediction = data.prediction

      if (connectionMode === 'simulator') {
        if (data.type !== 'data') return
        targetStationId = data.sender_id || 1
      }
      else if (connectionMode === 'wifi') {
        if (data.type !== 'gateway_data') return
        targetStationId = data.sender_id
      }
      else return

      setAvailableStations(prev => {
        const newSet = new Set(prev)
        if (!newSet.has(targetStationId)) newSet.add(targetStationId)
        return newSet
      })

      setIsConnected(true)

      setStationData(prev => {
        const currentStationState = prev[targetStationId] || {
          sensorData: null,
          prediction: null,
          history: [],
          faultCounts: {},
          lastUpdate: new Date()
        }

        const newEntry: HistoryEntry = {
          time: new Date().toLocaleTimeString(),
          voltage: incomingSensorData.voltage,
          current: incomingSensorData.current,
          temperature: incomingSensorData.temperature,
          light: incomingSensorData.light_intensity,
          humidity: incomingSensorData.humidity || 0,
          thermistor: incomingSensorData.thermistor_temp || 0,
          power: incomingPrediction.power,
          efficiency: incomingPrediction.efficiency || incomingSensorData.efficiency || 0,
          relay: incomingSensorData.relay_status || false,
          fault: incomingPrediction.fault_type
        }

        const newFaultCounts = { ...currentStationState.faultCounts }
        newFaultCounts[incomingPrediction.fault_type] = (newFaultCounts[incomingPrediction.fault_type] || 0) + 1

        return {
          ...prev,
          [targetStationId]: {
            sensorData: incomingSensorData,
            prediction: incomingPrediction,
            history: [...currentStationState.history.slice(-100), newEntry],
            faultCounts: newFaultCounts,
            lastUpdate: new Date()
          }
        }
      })
    }

    ws.onclose = () => {
      console.log('WebSocket disconnected')
      setIsMonitoring(false)
      setIsConnected(false)
    }

    ws.onerror = () => {
      console.error('WebSocket error')
      setIsMonitoring(false)
      setIsConnected(false)
    }

    return () => {
      clearInterval(staleCheck)
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ command: 'stop' }))
      }
      ws.close()
    }
  }, [isMonitoring, connectionMode])

  // Update simulation fault type
  useEffect(() => {
    if (connectionMode === 'simulator' && isMonitoring) {
      fetch(`http://localhost:8000/api/set-simulation-mode?fault_type=${simulationFault}`, {
        method: 'POST'
      })
    }
  }, [simulationFault, connectionMode, isMonitoring])

  // Configure WhatsApp notifications
  const configureWhatsApp = async () => {
    if (!whatsappNumber || whatsappNumber.length !== 10) {
      setWhatsappStatus('❌ Enter valid 10-digit number')
      setTimeout(() => setWhatsappStatus(''), 3000)
      return
    }
    try {
      const fullNumber = '+91' + whatsappNumber  // Add India country code
      const response = await fetch('http://localhost:8000/api/whatsapp/configure', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          phone_number: fullNumber,
          enabled: whatsappEnabled
        })
      })
      const data = await response.json()
      setWhatsappStatus(data.status === 'configured' ? '✅ Saved! Will notify: ' + fullNumber : '❌ Failed')
      setTimeout(() => setWhatsappStatus(''), 5000)
    } catch (error) {
      setWhatsappStatus('❌ Connection error')
      setTimeout(() => setWhatsappStatus(''), 3000)
    }
  }

  // Test WhatsApp notification
  const testWhatsApp = async () => {
    if (!whatsappNumber || whatsappNumber.length !== 10) {
      setWhatsappStatus('❌ Save config first!')
      setTimeout(() => setWhatsappStatus(''), 3000)
      return
    }
    try {
      setWhatsappStatus('📤 Opening WhatsApp Web...')
      const response = await fetch('http://localhost:8000/api/whatsapp/test', { method: 'POST' })
      const data = await response.json()
      setWhatsappStatus(data.status === 'sent' ? '✅ Check your browser!' : '❌ ' + (data.detail || 'Failed'))
      setTimeout(() => setWhatsappStatus(''), 5000)
    } catch (error) {
      setWhatsappStatus('❌ Connection error')
      setTimeout(() => setWhatsappStatus(''), 3000)
    }
  }

  const pieData = Object.entries(faultCounts).map(([name, value]) => ({
    name,
    value,
    color: FAULT_INFO[name]?.color || '#666'
  }))

  return (
    <main className="min-h-screen grid-bg">
      {/* Header */}
      <header className="border-b border-white/10 backdrop-blur-xl bg-dark-900/50 sticky top-0 z-50">
        <div className="max-w-7xl mx-auto px-6 py-4">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-4">
              <motion.div
                className="w-12 h-12 rounded-xl bg-gradient-to-br from-solar-400 to-amber-500 flex items-center justify-center"
                animate={{ rotate: isMonitoring ? 360 : 0 }}
                transition={{ duration: 10, repeat: Infinity, ease: "linear" }}
              >
                <Sun className="w-7 h-7 text-dark-900" />
              </motion.div>
              <div>
                <h1 className="text-2xl font-bold gradient-text">Solar Panel Monitor</h1>
                <p className="text-sm text-gray-400">AI-Powered Fault Detection System</p>
              </div>
            </div>

            <div className="flex items-center gap-4">
              <div className="flex items-center gap-2 px-4 py-2 rounded-full bg-white/5 border border-white/10">
                <div className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-500' : 'bg-gray-500'}`} />
                <span className="text-sm">{connectionMode === 'simulator' ? 'Simulator' : connectionMode === 'serial' ? 'USB' : 'WiFi'}</span>
              </div>

              {/* Station Selector */}
              {availableStations.size > 0 && (
                <div className="flex bg-white/5 rounded-xl p-1 border border-white/10">
                  {Array.from(availableStations).sort().map(id => (
                    <button
                      key={id}
                      onClick={() => setSelectedStation(id)}
                      className={`px-3 py-1.5 rounded-lg text-sm font-medium transition-all ${selectedStation === id
                        ? 'bg-solar-500 text-dark-900 shadow-lg'
                        : 'text-gray-400 hover:text-white'
                        }`}
                    >
                      Station {id}
                    </button>
                  ))}
                </div>
              )}

              <motion.button
                onClick={() => setIsMonitoring(!isMonitoring)}
                className={`px-6 py-3 rounded-xl font-semibold flex items-center gap-2 transition-all ${isMonitoring
                  ? 'bg-red-500/20 border border-red-500/50 text-red-400 hover:bg-red-500/30'
                  : 'bg-gradient-to-r from-solar-500 to-amber-500 text-dark-900 hover:shadow-lg hover:shadow-solar-500/30'
                  }`}
                whileHover={{ scale: 1.02 }}
                whileTap={{ scale: 0.98 }}
              >
                {isMonitoring ? <Pause className="w-5 h-5" /> : <Play className="w-5 h-5" />}
                {isMonitoring ? 'Stop' : 'Start'} Monitoring
              </motion.button>
            </div>
          </div>
        </div>
      </header>

      <div className="max-w-7xl mx-auto px-6 py-8">
        {/* Status Banner */}
        <motion.div
          initial={{ opacity: 0, y: -20 }}
          animate={{ opacity: 1, y: 0 }}
          className="mb-8"
        >
          <StatusCard prediction={prediction} />
        </motion.div>

        <div className="grid grid-cols-12 gap-6">
          {/* Left Column - Gauges */}
          <div className="col-span-12 lg:col-span-8">
            {/* Environmental Sensor Gauges */}
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-4">
              <Gauge
                value={sensorData?.humidity || 0}
                min={0} max={100}
                label="Humidity"
                unit="%"
                color="#06b6d4"
                icon={Sun}
              />
              <Gauge
                value={sensorData?.temperature || 0}
                min={0} max={100}
                label="Air Temperature"
                unit="°C"
                color="#f59e0b"
                icon={Thermometer}
              />
              <Gauge
                value={sensorData?.thermistor_temp || 0}
                min={0} max={100}
                label="Case Temp"
                unit="°C"
                color="#ec4899"
                icon={Thermometer}
              />
            </div>

            {/* Electrical & Solar Gauges */}
            <div className="grid grid-cols-2 lg:grid-cols-4 gap-4 mb-6">
              <Gauge
                value={sensorData?.voltage || 0}
                min={0} max={30}
                label="Voltage"
                unit="V"
                color="#3b82f6"
                icon={Zap}
              />
              <Gauge
                value={sensorData?.current || 0}
                min={0} max={12}
                label="Current"
                unit="A"
                color="#ef4444"
                icon={Activity}
              />
              <Gauge
                value={sensorData?.light_intensity || 0}
                min={0} max={1500}
                label="Light"
                unit="lux"
                color="#22c55e"
                icon={Eye}
              />
              <Gauge
                value={sensorData?.efficiency || 0}
                min={0} max={25}
                label="Efficiency"
                unit="%"
                color="#8b5cf6"
                icon={Percent}
              />
            </div>

            {/* Charts */}
            <div className="grid grid-cols-2 md:grid-cols-3 gap-4 mb-6">
              <MiniChart data={history} dataKey="power" color="#fbbf24" title="Power Output (W)" />
              <MiniChart data={history} dataKey="voltage" color="#3b82f6" title="Voltage (V)" />
              <MiniChart data={history} dataKey="current" color="#ef4444" title="Current (A)" />
              <MiniChart data={history} dataKey="temperature" color="#f59e0b" title="Temperature (°C)" />
              <MiniChart data={history} dataKey="light" color="#22c55e" title="Light (lux)" />
              <MiniChart data={history} dataKey="efficiency" color="#8b5cf6" title="Efficiency (%)" />
              <MiniChart data={history} dataKey="humidity" color="#06b6d4" title="Humidity (%)" />
              <MiniChart data={history} dataKey="thermistor" color="#ec4899" title="Case Temperature (°C)" />
            </div>

            {/* Main Chart */}
            <div className="glass-card p-6">
              <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
                <TrendingUp className="w-5 h-5 text-solar-400" />
                Real-time Power Output
              </h3>
              <div className="h-64">
                <ResponsiveContainer width="100%" height="100%">
                  <AreaChart data={history.slice(-50)}>
                    <defs>
                      <linearGradient id="powerGradient" x1="0" y1="0" x2="0" y2="1">
                        <stop offset="0%" stopColor="#fbbf24" stopOpacity={0.4} />
                        <stop offset="100%" stopColor="#fbbf24" stopOpacity={0} />
                      </linearGradient>
                    </defs>
                    <XAxis dataKey="time" stroke="#64748b" fontSize={12} />
                    <YAxis stroke="#64748b" fontSize={12} />
                    <Area
                      type="monotone"
                      dataKey="power"
                      stroke="#fbbf24"
                      strokeWidth={2}
                      fill="url(#powerGradient)"
                    />
                  </AreaChart>
                </ResponsiveContainer>
              </div>
            </div>
          </div>

          {/* Right Column - Controls & Stats */}
          <div className="col-span-12 lg:col-span-4 space-y-6">
            {/* Connection Panel */}
            <ConnectionPanel
              mode={connectionMode}
              setMode={setConnectionMode}
              isConnected={isConnected}
              simulationFault={simulationFault}
              setSimulationFault={setSimulationFault}
            />

            {/* Relay Control Panel */}
            <motion.div 
              className="glass-card p-6 overflow-hidden relative"
              layout
            >
              <div className="flex items-center justify-between mb-4">
                <h3 className="text-lg font-semibold flex items-center gap-2">
                  <Zap className={`w-5 h-5 transition-colors ${sensorData?.relay_status ? 'text-yellow-400 animate-pulse' : 'text-gray-500'}`} />
                  Relay Control (Station {selectedStation})
                </h3>
                <motion.div 
                  className={`px-3 py-1 rounded-full text-[10px] font-bold uppercase tracking-widest transition-all ${sensorData?.relay_status 
                    ? 'bg-yellow-500/30 text-yellow-300 border border-yellow-500/50' 
                    : 'bg-gray-500/20 text-gray-400 border border-gray-500/30'
                  }`}
                  animate={{
                    scale: sensorData?.relay_status ? 1.05 : 1,
                  }}
                >
                  {sensorData?.relay_status ? '✓ ACTIVATED' : '✗ DEACTIVATED'}
                </motion.div>
              </div>

              <motion.div 
                className="p-4 rounded-xl bg-gradient-to-br mb-4 text-center border transition-all"
                animate={{
                  background: sensorData?.relay_status 
                    ? 'linear-gradient(135deg, rgba(234, 179, 8, 0.1), rgba(168, 85, 247, 0.1))'
                    : 'linear-gradient(135deg, rgba(107, 114, 128, 0.05), rgba(75, 85, 99, 0.05))',
                  borderColor: sensorData?.relay_status ? 'rgba(234, 179, 8, 0.5)' : 'rgba(107, 114, 128, 0.2)'
                }}
              >
                <p className="text-xs text-gray-400 mb-2">
                  Pin Status: GPIO {selectedStation === 1 ? '13' : selectedStation === 2 ? '2' : '?'}
                </p>
                <div className={`text-3xl font-black tracking-wide transition-all ${sensorData?.relay_status 
                  ? 'text-yellow-400 drop-shadow-lg drop-shadow-yellow-500/50' 
                  : 'text-gray-600 drop-shadow-none'
                }`}>
                  {sensorData?.relay_status ? '⚡ ON' : '⊘ OFF'}
                </div>
                <p className="text-[10px] text-gray-500 mt-2">
                  {sensorData?.relay_status ? 'Pin pulled LOW (Active)' : 'Pin released HIGH (Inactive)'}
                </p>
                <p className="text-[9px] text-gray-600 mt-2">
                  {selectedStation === 1 ? 'Station 1 (sender_node)' : selectedStation === 2 ? 'Station 2 (sender_node_A)' : 'Unknown Station'}
                </p>
              </motion.div>

              <div className="space-y-2">
                <motion.button
                  onClick={async () => {
                    const nextCmd = sensorData?.relay_status ? 'DEACTIVATE_RELAY' : 'ACTIVATE_RELAY';
                    setLastRelayCommand({
                      time: new Date().toLocaleTimeString(),
                      station: selectedStation,
                      command: nextCmd,
                      status: 'sending...'
                    })
                    try {
                      const resp = await fetch('http://localhost:8000/api/command', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ station_id: selectedStation, command: nextCmd })
                      })
                      if (resp.ok) {
                        const data = await resp.json()
                        console.log(`✓ ${nextCmd} command accepted by backend`)
                        setLastRelayCommand(prev => ({...prev, status: 'queued (waiting for ESP response)'}))
                      } else {
                        setLastRelayCommand(prev => ({...prev, status: 'failed'}))
                      }
                    } catch (err) {
                      console.error('Failed to send relay command:', err)
                      setLastRelayCommand(prev => ({...prev, status: 'error: ' + String(err).substring(0, 30)}))
                    }
                  }}
                  className={`w-full p-4 rounded-xl font-bold flex items-center justify-center gap-3 transition-all duration-300 ${sensorData?.relay_status
                      ? 'bg-red-500/20 border border-red-500/50 text-red-400 hover:bg-red-500/30 hover:border-red-500/70 active:scale-95'
                      : 'bg-green-500/20 border border-green-500/50 text-green-400 hover:bg-green-500/30 hover:border-green-500/70 active:scale-95'
                    }`}
                  whileHover={{ scale: 1.02 }}
                  whileTap={{ scale: 0.96 }}
                >
                  <RefreshCw className="w-5 h-5" />
                  <span>{sensorData?.relay_status ? 'Turn Off' : 'Turn On'}</span>
                </motion.button>

                <motion.button
                  onClick={async () => {
                    setLastRelayCommand({
                      time: new Date().toLocaleTimeString(),
                      station: selectedStation,
                      command: 'TOGGLE_RELAY',
                      status: 'sending...'
                    })
                    try {
                      const resp = await fetch('http://localhost:8000/api/command', {
                        method: 'POST',
                        headers: { 'Content-Type': 'application/json' },
                        body: JSON.stringify({ station_id: selectedStation, command: 'TOGGLE_RELAY' })
                      })
                      if (resp.ok) {
                        console.log(`✓ TOGGLE command accepted by backend`)
                        setLastRelayCommand(prev => ({...prev, status: 'queued (waiting for ESP response)'}))
                      } else {
                        setLastRelayCommand(prev => ({...prev, status: 'failed'}))
                      }
                    } catch (err) {
                      console.error('Failed to send toggle command:', err)
                      setLastRelayCommand(prev => ({...prev, status: 'error: ' + String(err).substring(0, 30)}))
                    }
                  }}
                  className="w-full p-3 rounded-xl font-semibold flex items-center justify-center gap-2 bg-white/5 border border-white/20 text-white/70 hover:bg-white/10 active:scale-95 transition-all"
                  whileHover={{ scale: 1.02 }}
                  whileTap={{ scale: 0.96 }}
                >
                  <Zap className="w-4 h-4" />
                  <span>Quick Toggle</span>
                </motion.button>
              </div>

              {/* DEBUG INFO PANEL */}
              <div className="mt-4 p-3 rounded-lg bg-white/5 border border-white/10 space-y-2">
                <div className="text-[10px] font-mono text-gray-400">
                  <div>Mode: <span className="text-blue-300">{connectionMode.toUpperCase()}</span></div>
                  <div>Last Cmd: <span className="text-yellow-300">{lastRelayCommand.command}</span> @ {lastRelayCommand.time}</div>
                  <div>Status: <span className={`font-bold ${
                    lastRelayCommand.status === 'queued (waiting for ESP response)' ? 'text-amber-300' :
                    lastRelayCommand.status.includes('error') ? 'text-red-300' :
                    lastRelayCommand.status === 'idle' ? 'text-gray-400' :
                    'text-green-300'
                  }`}>{lastRelayCommand.status}</span></div>
                  <div>Station: <span className="text-purple-300">S{lastRelayCommand.station} (GPIO {selectedStation === 1 ? '13' : '2'})</span></div>
                </div>
              </div>

              <motion.p 
                className="text-[10px] text-gray-500 mt-3 text-center italic"
                animate={{ opacity: [0.5, 1, 0.5] }}
                transition={{ duration: 3, repeat: Infinity }}
              >
                Real pin status: {sensorData ? '✓ Receiving updates' : '⏳ Waiting for data...'}
              </motion.p>
            </motion.div>

            {/* Fault Distribution */}
            <div className="glass-card p-6">
              <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
                <AlertTriangle className="w-5 h-5 text-solar-400" />
                Fault Distribution
              </h3>
              {pieData.length > 0 ? (
                <div className="h-48">
                  <ResponsiveContainer width="100%" height="100%">
                    <PieChart>
                      <Pie
                        data={pieData}
                        cx="50%"
                        cy="50%"
                        innerRadius={40}
                        outerRadius={70}
                        dataKey="value"
                        stroke="none"
                      >
                        {pieData.map((entry, index) => (
                          <Cell key={index} fill={entry.color} />
                        ))}
                      </Pie>
                    </PieChart>
                  </ResponsiveContainer>
                </div>
              ) : (
                <div className="h-48 flex items-center justify-center text-gray-500">
                  Start monitoring to see distribution
                </div>
              )}
              <div className="mt-4 space-y-2">
                {Object.entries(FAULT_INFO).map(([name, info]) => (
                  <div key={name} className="flex items-center justify-between text-sm">
                    <div className="flex items-center gap-2">
                      <div className="w-3 h-3 rounded-full" style={{ backgroundColor: info.color }} />
                      <span className="text-gray-300">{name.replace('_', ' ')}</span>
                    </div>
                    <span className="font-medium">{faultCounts[name] || 0}</span>
                  </div>
                ))}
              </div>
            </div>

            {/* System Info */}
            <div className="glass-card p-6">
              <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
                <Radio className="w-5 h-5 text-solar-400" />
                System Info
              </h3>
              <div className="space-y-3 text-sm">
                <div className="flex justify-between">
                  <span className="text-gray-400">Model</span>
                  <span>Random Forest (10 trees)</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Accuracy</span>
                  <span className="text-green-400">100%</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Update Rate</span>
                  <span>500ms</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-gray-400">Total Readings</span>
                  <span>{history.length}</span>
                </div>
              </div>
            </div>

            {/* WhatsApp Notifications */}
            <div className="glass-card p-6">
              <h3 className="text-lg font-semibold mb-4 flex items-center gap-2">
                <MessageCircle className="w-5 h-5 text-green-400" />
                WhatsApp Alerts
              </h3>
              <div className="space-y-4">
                <div>
                  <label className="text-sm text-gray-400 block mb-2">Recipient Phone Number</label>
                  <div className="flex gap-2">
                    <span className="p-3 rounded-lg bg-white/10 border border-white/10 text-gray-300 text-sm">+91</span>
                    <input
                      type="text"
                      placeholder="9945697063"
                      value={whatsappNumber}
                      onChange={(e) => setWhatsappNumber(e.target.value.replace(/\D/g, ''))}
                      className="flex-1 p-3 rounded-lg bg-white/5 border border-white/10 text-white placeholder-gray-500 text-sm"
                      maxLength={10}
                    />
                  </div>
                  <p className="text-xs text-gray-500 mt-1">Enter 10-digit number (without country code)</p>
                </div>
                <label className="flex items-center gap-3 cursor-pointer">
                  <input
                    type="checkbox"
                    checked={whatsappEnabled}
                    onChange={(e) => setWhatsappEnabled(e.target.checked)}
                    className="w-4 h-4 rounded border-gray-600 bg-white/5 text-green-500 focus:ring-green-500"
                  />
                  <span className="text-sm text-gray-300">Enable fault notifications</span>
                </label>
                <div className="grid grid-cols-2 gap-2">
                  <button
                    onClick={configureWhatsApp}
                    className="p-2 rounded-lg bg-green-500/20 border border-green-500/50 text-green-400 text-sm hover:bg-green-500/30 transition"
                  >
                    Save Config
                  </button>
                  <button
                    onClick={testWhatsApp}
                    className="p-2 rounded-lg bg-white/5 border border-white/10 text-gray-300 text-sm hover:bg-white/10 transition"
                  >
                    Send Test
                  </button>
                </div>
                {whatsappStatus && (
                  <div className="text-sm text-center text-gray-400">{whatsappStatus}</div>
                )}
                <div className="text-xs text-amber-400/80 mt-2 p-2 rounded bg-amber-500/10 border border-amber-500/20">
                  ⚠️ First time? Open <a href="https://web.whatsapp.com" target="_blank" className="underline">web.whatsapp.com</a> and scan QR code with your phone!
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* Footer */}
      <footer className="border-t border-white/10 mt-12 py-6">
        <div className="max-w-7xl mx-auto px-6 text-center text-gray-500 text-sm">
          <p>Solar Panel Fault Detection System • Powered by Machine Learning</p>
        </div>
      </footer>
    </main>
  )
}
