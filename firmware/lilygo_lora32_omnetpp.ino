/**
 * LilyGO LORA32 Firmware for OMNeT++ Hardware-in-the-Loop Integration
 * 
 * This firmware enables the LilyGO LORA32 device to communicate with
 * an OMNeT++ simulation via serial port.
 * 
 * Hardware: LilyGO TTGO LORA32 V2.1 (923MHz)
 * Chip: SX1276/SX1278
 * 
 * Board: TTGO LoRa32-OLED V1/V2
 * Upload Speed: 921600
 * 
 * Required Libraries:
 * - LoRa by Sandeep Mistry (install via Arduino Library Manager)
 */

#include <SPI.h>
#include <LoRa.h>

// LilyGO LORA32 Pin Definitions
#define SCK     5
#define MISO    19
#define MOSI    27
#define SS      18
#define RST     14
#define DIO0    26

// LoRa Configuration
#define LORA_FREQUENCY  923E6  // 923 MHz
#define LORA_BANDWIDTH  125E3  // 125 kHz
#define LORA_SF         12     // Spreading Factor 12
#define LORA_TX_POWER   14     // 14 dBm

// Serial Protocol Constants
#define SERIAL_BAUD     115200
#define START_BYTE_1    0xAA
#define START_BYTE_2    0x55
#define MAX_PAYLOAD     255

// Packet Structure
struct SerialPacket {
  uint8_t msgType;
  uint16_t source;
  uint16_t destination;
  uint16_t via;
  uint8_t ttl;
  uint8_t sf;
  uint8_t tp;
  uint16_t cf;
  uint8_t rssi;
  uint8_t payloadLen;
  uint8_t payload[MAX_PAYLOAD];
};

// Global variables
uint16_t nodeId = 0;
bool loraInitialized = false;
unsigned long lastPacketTime = 0;

void setup() {
  // Initialize Serial
  Serial.begin(SERIAL_BAUD);
  while (!Serial);
  
  Serial.println("LilyGO LORA32 - OMNeT++ HIL Interface");
  Serial.println("Initializing...");
  
  // Initialize LoRa
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("ERROR: LoRa initialization failed!");
    while (1);
  }
  
  // Configure LoRa
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.enableCrc();
  
  loraInitialized = true;
  Serial.println("LoRa initialized successfully");
  Serial.print("Frequency: ");
  Serial.print(LORA_FREQUENCY / 1E6);
  Serial.println(" MHz");
  Serial.print("SF: ");
  Serial.println(LORA_SF);
  
  // Set up LoRa receive callback
  LoRa.onReceive(onLoRaReceive);
  LoRa.receive();
  
  Serial.println("Waiting for configuration from OMNeT++...");
}

void loop() {
  // Check for serial data from OMNeT++
  if (Serial.available() >= 2) {
    uint8_t byte1 = Serial.peek();
    if (byte1 == START_BYTE_1) {
      processSerialPacket();
    } else {
      // Discard invalid byte
      Serial.read();
    }
  }
  
  // Periodic status update
  if (millis() - lastPacketTime > 10000) {
    Serial.print("Status: Ready (Node ID: ");
    Serial.print(nodeId);
    Serial.println(")");
    lastPacketTime = millis();
  }
}

void processSerialPacket() {
  // Wait for complete packet header
  if (Serial.available() < 3) {
    return;
  }
  
  uint8_t start1 = Serial.read();
  uint8_t start2 = Serial.read();
  
  if (start1 != START_BYTE_1 || start2 != START_BYTE_2) {
    Serial.println("ERROR: Invalid start bytes");
    return;
  }
  
  uint8_t len = Serial.read();
  
  // Wait for complete packet
  unsigned long timeout = millis() + 1000;
  while (Serial.available() < len + 1 && millis() < timeout) {
    delay(1);
  }
  
  if (Serial.available() < len + 1) {
    Serial.println("ERROR: Packet timeout");
    return;
  }
  
  // Read packet data
  uint8_t buffer[260];
  buffer[0] = len;
  for (int i = 1; i <= len + 1; i++) {
    buffer[i] = Serial.read();
  }
  
  // Verify CRC
  uint8_t crc = len;
  for (int i = 1; i < len + 1; i++) {
    crc ^= buffer[i];
  }
  
  if (crc != buffer[len + 1]) {
    Serial.println("ERROR: CRC mismatch");
    return;
  }
  
  // Parse packet
  SerialPacket pkt;
  pkt.msgType = buffer[1];
  pkt.source = (buffer[2] << 8) | buffer[3];
  pkt.destination = (buffer[4] << 8) | buffer[5];
  pkt.via = (buffer[6] << 8) | buffer[7];
  pkt.ttl = buffer[8];
  pkt.sf = buffer[9];
  pkt.tp = buffer[10];
  pkt.cf = (buffer[11] << 8) | buffer[12];
  pkt.rssi = buffer[13];
  pkt.payloadLen = len - 13;
  
  for (int i = 0; i < pkt.payloadLen; i++) {
    pkt.payload[i] = buffer[14 + i];
  }
  
  // Handle packet
  if (pkt.msgType == 0xFF) {
    // Initialization packet
    nodeId = pkt.source;
    
    // Update LoRa parameters if specified
    if (pkt.sf >= 7 && pkt.sf <= 12) {
      LoRa.setSpreadingFactor(pkt.sf);
    }
    if (pkt.tp > 0 && pkt.tp <= 20) {
      LoRa.setTxPower(pkt.tp);
    }
    
    Serial.print("Initialized - Node ID: ");
    Serial.print(nodeId);
    Serial.print(", SF: ");
    Serial.print(pkt.sf);
    Serial.print(", TP: ");
    Serial.print(pkt.tp);
    Serial.println(" dBm");
  } else {
    // Data packet - transmit via LoRa
    transmitLoRaPacket(&pkt);
  }
}

void transmitLoRaPacket(SerialPacket* pkt) {
  Serial.print("TX LoRa: Src=");
  Serial.print(pkt->source);
  Serial.print(" Dst=");
  Serial.print(pkt->destination);
  Serial.print(" Type=");
  Serial.println(pkt->msgType);
  
  // Update LoRa parameters if needed
  if (pkt->sf >= 7 && pkt->sf <= 12) {
    LoRa.setSpreadingFactor(pkt->sf);
  }
  if (pkt->tp > 0 && pkt->tp <= 20) {
    LoRa.setTxPower(pkt->tp);
  }
  
  // Begin LoRa packet
  LoRa.beginPacket();
  
  // Write header
  LoRa.write(pkt->msgType);
  LoRa.write((pkt->source >> 8) & 0xFF);
  LoRa.write(pkt->source & 0xFF);
  LoRa.write((pkt->destination >> 8) & 0xFF);
  LoRa.write(pkt->destination & 0xFF);
  LoRa.write((pkt->via >> 8) & 0xFF);
  LoRa.write(pkt->via & 0xFF);
  LoRa.write(pkt->ttl);
  
  // Write payload
  for (int i = 0; i < pkt->payloadLen; i++) {
    LoRa.write(pkt->payload[i]);
  }
  
  // End and transmit
  LoRa.endPacket();
  
  // Return to receive mode
  LoRa.receive();
  
  Serial.println("LoRa packet transmitted");
  lastPacketTime = millis();
}

void onLoRaReceive(int packetSize) {
  if (packetSize == 0) return;
  
  Serial.print("RX LoRa: ");
  Serial.print(packetSize);
  Serial.println(" bytes");
  
  // Read packet
  SerialPacket pkt;
  
  if (packetSize >= 8) {
    pkt.msgType = LoRa.read();
    pkt.source = (LoRa.read() << 8) | LoRa.read();
    pkt.destination = (LoRa.read() << 8) | LoRa.read();
    pkt.via = (LoRa.read() << 8) | LoRa.read();
    pkt.ttl = LoRa.read();
    
    // Read remaining payload
    pkt.payloadLen = 0;
    while (LoRa.available() && pkt.payloadLen < MAX_PAYLOAD) {
      pkt.payload[pkt.payloadLen++] = LoRa.read();
    }
    
    // Get RSSI and SNR
    pkt.rssi = LoRa.packetRssi() + 128; // Convert to unsigned
    pkt.sf = LORA_SF;
    pkt.tp = LORA_TX_POWER;
    pkt.cf = LORA_FREQUENCY / 1E6;
    
    // Forward to OMNeT++ via serial
    sendSerialPacket(&pkt);
  }
}

void sendSerialPacket(SerialPacket* pkt) {
  uint8_t buffer[270];
  int idx = 0;
  
  buffer[idx++] = START_BYTE_1;
  buffer[idx++] = START_BYTE_2;
  
  uint8_t len = 13 + pkt->payloadLen;
  buffer[idx++] = len;
  
  buffer[idx++] = pkt->msgType;
  buffer[idx++] = (pkt->source >> 8) & 0xFF;
  buffer[idx++] = pkt->source & 0xFF;
  buffer[idx++] = (pkt->destination >> 8) & 0xFF;
  buffer[idx++] = pkt->destination & 0xFF;
  buffer[idx++] = (pkt->via >> 8) & 0xFF;
  buffer[idx++] = pkt->via & 0xFF;
  buffer[idx++] = pkt->ttl;
  buffer[idx++] = pkt->sf;
  buffer[idx++] = pkt->tp;
  buffer[idx++] = (pkt->cf >> 8) & 0xFF;
  buffer[idx++] = pkt->cf & 0xFF;
  buffer[idx++] = pkt->rssi;
  
  for (int i = 0; i < pkt->payloadLen; i++) {
    buffer[idx++] = pkt->payload[i];
  }
  
  // Calculate CRC
  uint8_t crc = 0;
  for (int i = 2; i < idx; i++) {
    crc ^= buffer[i];
  }
  buffer[idx++] = crc;
  
  // Send to serial
  Serial.write(buffer, idx);
  Serial.flush();
  
  Serial.print("Forwarded to OMNeT++: Src=");
  Serial.print(pkt->source);
  Serial.print(" Dst=");
  Serial.print(pkt->destination);
  Serial.print(" RSSI=");
  Serial.print((int)pkt->rssi - 128);
  Serial.println(" dBm");
  
  lastPacketTime = millis();
}
