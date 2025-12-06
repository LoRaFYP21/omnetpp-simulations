# Quick Start Guide - Hardware Integration

## Prerequisites
- LilyGO LORA32 device (923MHz)
- USB cable
- Arduino IDE installed
- LoRa library installed (via Arduino Library Manager)

## 5-Minute Setup

### 1. Flash Firmware to LilyGO Device
```
1. Open Arduino IDE
2. Install "TTGO LoRa32-OLED" board support (ESP32)
3. Install "LoRa" library by Sandeep Mistry
4. Open: firmware/lilygo_lora32_omnetpp.ino
5. Select Board: TTGO LoRa32-OLED V1/V2
6. Select Port: (your COM port)
7. Click Upload
```

### 2. Find Your COM Port
```
Windows: Device Manager → Ports (COM & LPT)
Look for: "Silicon Labs CP210x" or "USB Serial"
Note the COM number (e.g., COM3)
```

### 3. Configure OMNeT++
```
1. Edit: simulations/hardware_integration.ini
2. Change line: *.hardwareNode[*].serialPort = "COM3"
   Replace COM3 with your actual COM port
3. Save file
```

### 4. Build Project
```
OMNeT++ IDE:
1. Project → Clean
2. Project → Build Project
3. Wait for successful build
```

### 5. Run Simulation
```
1. Close Arduino Serial Monitor (if open)
2. In OMNeT++: Select hardware_integration.ini
3. Click Run button
4. Check console for: "Serial port initialized"
```

## Verify It's Working

### Arduino Serial Monitor (115200 baud):
```
LilyGO LORA32 - OMNeT++ HIL Interface
Initializing...
LoRa initialized successfully
Frequency: 923.00 MHz
SF: 12
Initialized - Node ID: X, SF: 12, TP: 14 dBm
```

### OMNeT++ Console:
```
Serial port initialized: COM3 @ 115200 baud
LoRaHardwareApp initialized for node X
```

## Test Communication

### Send Test Packet
In simulation, configure a node to send to hardware node ID.
Watch Arduino Serial Monitor for:
```
TX LoRa: Src=X Dst=Y Type=1
LoRa packet transmitted
```

### Receive Test Packet
If another LoRa device transmits, watch for:
```
RX LoRa: 15 bytes
Forwarded to OMNeT++: Src=Y Dst=X RSSI=-85 dBm
```

## Common Issues

### "Cannot open serial port"
→ Close Arduino Serial Monitor
→ Check COM port number
→ Try different USB port

### "LoRa initialization failed"
→ Check antenna is connected
→ Verify board selection in Arduino IDE
→ Re-upload firmware

### No packets received
→ Check frequency: 923 MHz
→ Verify spreading factor: 12
→ Check antenna connection

## Next Steps

1. Review `README_HARDWARE_INTEGRATION.md` for detailed documentation
2. Customize LoRa parameters in .ini file
3. Add more hardware nodes
4. Test with real-world scenarios

## Support Files

- Full documentation: `README_HARDWARE_INTEGRATION.md`
- Configuration file: `simulations/hardware_integration.ini`
- Firmware: `firmware/lilygo_lora32_omnetpp.ino`
- Network definition: `simulations/networks/HardwareNetwork.ned`
