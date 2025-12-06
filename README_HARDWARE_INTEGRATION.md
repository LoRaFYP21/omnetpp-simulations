# Hardware-in-the-Loop Integration Guide

This guide explains how to integrate your LilyGO LORA32 923MHz device with the OMNeT++ simulation.

## Overview

The hardware integration creates a bridge between your physical LilyGO LORA32 device and the OMNeT++ simulation environment. The physical device becomes a node in the simulated LoRa network, allowing real-world hardware testing within the simulation.

## Architecture

```
┌─────────────────────────────────────────┐
│        OMNeT++ Simulation               │
│                                         │
│  ┌──────────┐  ┌──────────┐           │
│  │Simulated │  │Simulated │           │
│  │  Node 1  │  │  Node 2  │  ...      │
│  └──────────┘  └──────────┘           │
│                                         │
│  ┌─────────────────────────┐          │
│  │  LoRaHardwareNode       │          │
│  │  ┌─────────────────┐   │          │
│  │  │LoRaHardwareApp  │   │          │
│  │  └────────┬────────┘   │          │
│  │           │             │          │
│  │  ┌────────▼────────┐   │          │
│  │  │SerialPortInterface│  │          │
│  │  └────────┬────────┘   │          │
│  └───────────┼─────────────┘          │
│              │ Serial/USB             │
└──────────────┼─────────────────────────┘
               │
               ▼
    ┌──────────────────────┐
    │  LilyGO LORA32       │
    │  Physical Device     │
    │  (923 MHz)           │
    └──────────────────────┘
```

## Components

### 1. OMNeT++ Side

- **SerialPortInterface**: Low-level serial communication (Windows)
- **LoRaHardwareApp**: Application layer that bridges simulation and hardware
- **LoRaHardwareNode**: Complete node module with serial interface

### 2. Hardware Side

- **Arduino Firmware**: Runs on LilyGO LORA32, handles serial protocol and LoRa transmission

## Setup Instructions

### Step 1: Hardware Setup

1. **Connect LilyGO LORA32 to PC**:
   - Use USB cable to connect device
   - Note the COM port (e.g., COM3, COM4)
   - Check in Windows Device Manager → Ports (COM & LPT)

2. **Install Arduino IDE and Libraries**:
   ```
   - Arduino IDE 1.8.19 or later
   - Board: ESP32 Dev Module or TTGO LoRa32-OLED
   - Library: LoRa by Sandeep Mistry (install via Library Manager)
   ```

3. **Upload Firmware**:
   - Open `firmware/lilygo_lora32_omnetpp.ino` in Arduino IDE
   - Select board: Tools → Board → ESP32 Arduino → TTGO LoRa32-OLED
   - Select port: Tools → Port → (your COM port)
   - Upload Speed: 921600
   - Click Upload
   - Verify successful upload in Serial Monitor (115200 baud)

### Step 2: OMNeT++ Configuration

1. **Update Makefile**:
   Add new source files to your project. In `src/Makefile`, ensure these files are included:
   ```makefile
   SOURCES += \
       misc/SerialPortInterface.cc \
       LoRaApp/LoRaHardwareApp.cc
   ```

2. **Rebuild Project**:
   - In OMNeT++ IDE: Project → Clean → Clean all projects
   - Project → Build Project
   - Verify no compilation errors

3. **Configure Serial Port**:
   Edit `simulations/hardware_integration.ini`:
   ```ini
   *.hardwareNode[*].serialPort = "COM3"  # Change to your actual COM port
   *.hardwareNode[*].baudRate = 115200
   ```

4. **Create Network**:
   Create or modify a network file (e.g., `simulations/networks/HardwareNetwork.ned`):
   ```ned
   import loranetwork.LoraNode.LoRaHardwareNode;
   import loranetwork.LoraNode.LoRaNode;
   import loranetwork.LoraNode.LoRaGW;
   import loranetwork.LoRaPhy.LoRaMedium;
   
   network LoRaMeshWithHardware {
       parameters:
           int numberOfNodes = default(5);
           int numberOfGateways = default(1);
           int numberOfHardwareNodes = default(1);
       
       submodules:
           N[numberOfNodes]: LoRaNode;
           loRaGW[numberOfGateways]: LoRaGW;
           hardwareNode[numberOfHardwareNodes]: LoRaHardwareNode;
           LoRaMedium: LoRaMedium;
   }
   ```

### Step 3: Running the Simulation

1. **Start Hardware Device**:
   - Ensure LilyGO LORA32 is connected and powered
   - Open Arduino Serial Monitor (115200 baud) to verify it's running
   - You should see: "Waiting for configuration from OMNeT++..."

2. **Run Simulation**:
   - In OMNeT++, select `hardware_integration.ini`
   - Click Run
   - The simulation will:
     - Initialize the serial connection
     - Send configuration to the hardware device
     - Begin normal simulation with hardware node

3. **Verify Connection**:
   - Check OMNeT++ console for: "Serial port initialized"
   - Check Arduino Serial Monitor for: "Initialized - Node ID: X"

## Communication Protocol

### Serial Packet Format

```
[START][START][LEN][TYPE][SRC_H][SRC_L][DST_H][DST_L][VIA_H][VIA_L][TTL][SF][TP][CF_H][CF_L][RSSI][PAYLOAD...][CRC]

- START: 0xAA 0x55 (2 bytes)
- LEN: Payload length + 13 (1 byte)
- TYPE: Message type (1 byte)
- SRC: Source address (2 bytes, big-endian)
- DST: Destination address (2 bytes, big-endian)
- VIA: Via/next hop (2 bytes, big-endian)
- TTL: Time to live (1 byte)
- SF: Spreading factor (1 byte)
- TP: Transmit power (1 byte)
- CF: Center frequency in MHz (2 bytes, big-endian)
- RSSI: Received signal strength + 128 (1 byte)
- PAYLOAD: Variable length data
- CRC: XOR checksum (1 byte)
```

### Message Types

- `0xFF`: Initialization/configuration
- `0x01`: DATA packet
- `0x05`: ACK packet
- `0x06`: ROUTING packet

## Troubleshooting

### Serial Port Issues

**Error: "Cannot open serial port"**
- Verify correct COM port in .ini file
- Check device is connected and drivers installed
- Close Arduino Serial Monitor (can't share port)
- Try different USB cable/port

**Error: "Access Denied"**
- Close any program using the COM port
- Restart OMNeT++ IDE
- Check Windows permissions

### Communication Issues

**Hardware not receiving packets**
- Verify baud rate matches (115200)
- Check Arduino Serial Monitor for errors
- Verify firmware uploaded correctly
- Check packet format in protocol

**Hardware not sending packets**
- Verify LoRa antenna is connected
- Check LoRa frequency matches (923 MHz)
- Check spreading factor and bandwidth
- Monitor RSSI values

### Simulation Issues

**Compilation errors**
- Ensure all source files are in Makefile
- Clean and rebuild project
- Check include paths

**Runtime errors**
- Verify NED files are correct
- Check .ini configuration
- Enable debug output: `cmdenv-express-mode = false`

## Advanced Configuration

### Multiple Hardware Nodes

You can connect multiple LilyGO devices:

```ini
*.numberOfHardwareNodes = 2
*.hardwareNode[0].serialPort = "COM3"
*.hardwareNode[1].serialPort = "COM4"
```

### Custom LoRa Parameters

Configure different LoRa settings per node:

```ini
*.hardwareNode[0].LoRaHardwareApp.initialLoRaSF = 7
*.hardwareNode[0].LoRaHardwareApp.initialLoRaTP = 20dBm
*.hardwareNode[0].LoRaHardwareApp.initialLoRaCF = 923.2MHz
```

### Polling Interval

Adjust serial port polling rate (trade-off between latency and CPU usage):

```ini
*.hardwareNode[*].LoRaHardwareApp.pollInterval = 0.01s  # 10ms (default)
*.hardwareNode[*].LoRaHardwareApp.pollInterval = 0.001s # 1ms (low latency)
*.hardwareNode[*].LoRaHardwareApp.pollInterval = 0.1s   # 100ms (low CPU)
```

## Testing

### Basic Connectivity Test

1. Upload firmware to LilyGO device
2. Run simulation with 1 hardware node + 1 simulated node
3. Configure simulated node to send packets to hardware node
4. Observe packet reception in Arduino Serial Monitor
5. Verify packet forwarding in OMNeT++ log

### Range Test

1. Place second LilyGO device at various distances
2. Monitor RSSI values in simulation
3. Compare with real-world measurements

## Performance Considerations

- **Polling Interval**: 10ms provides good balance (default)
- **Serial Baud Rate**: 115200 is sufficient for most scenarios
- **Packet Buffer**: Hardware has limited memory, avoid burst traffic
- **Synchronization**: Serial communication adds ~10-50ms latency

## References

- LilyGO LORA32 Documentation: https://github.com/LilyGO/TTGO-LORA32
- LoRa Library: https://github.com/sandeepmistry/arduino-LoRa
- OMNeT++ Manual: https://doc.omnetpp.org/
- INET Framework: https://inet.omnetpp.org/

## Support

For issues or questions:
1. Check Arduino Serial Monitor output
2. Enable OMNeT++ debug logging
3. Verify hardware connections
4. Review protocol implementation

## License

This integration follows the same license as the main project (LGPL v3).
