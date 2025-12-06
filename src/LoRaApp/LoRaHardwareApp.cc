//
// LoRaHardwareApp.cc - Implementation of hardware-in-the-loop interface
//

#include "LoRaHardwareApp.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/NodeOperations.h"
#include <algorithm>

namespace inet {

Define_Module(LoRaHardwareApp);

LoRaHardwareApp::~LoRaHardwareApp() {
    cancelAndDelete(pollTimer);
    if (serialPort) {
        serialPort->closePort();
        delete serialPort;
    }
}

void LoRaHardwareApp::initialize(int stage) {
    if (stage == INITSTAGE_LOCAL) {
        // Initialize parameters
        portName = par("serialPort").stdstringValue();
        baudRate = par("baudRate");
        pollInterval = par("pollInterval");
        
        nodeId = getContainingNode(this)->getIndex();
        
        // Initialize LoRa parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        
        // Initialize statistics
        LoRa_HW_PacketSent = registerSignal("LoRa_HW_PacketSent");
        LoRa_HW_PacketReceived = registerSignal("LoRa_HW_PacketReceived");
        
        packetsSentToHW = 0;
        packetsReceivedFromHW = 0;
        packetsForwardedToSim = 0;
        packetsForwardedToHW = 0;
        
        // Create poll timer
        pollTimer = new cMessage("pollTimer");
        
        EV << "LoRaHardwareApp initialized for node " << nodeId << endl;
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        // Initialize serial port
        if (!initializeSerialPort()) {
            EV_ERROR << "Failed to initialize serial port " << portName << endl;
            error("Cannot open serial port for hardware device");
        }
        
        // Start polling
        scheduleAt(simTime() + pollInterval, pollTimer);
    }
}

bool LoRaHardwareApp::initializeSerialPort() {
    serialPort = new SerialPortInterface();
    
    if (!serialPort->openPort(portName, baudRate)) {
        EV_ERROR << "Failed to open serial port: " << portName << endl;
        return false;
    }
    
    // Send initialization command to hardware
    SerialPacket initPkt;
    initPkt.msgType = 0xFF; // INIT command
    initPkt.source = nodeId;
    initPkt.destination = 0;
    initPkt.via = 0;
    initPkt.ttl = 0;
    initPkt.sf = loRaSF;
    initPkt.tp = loRaTP;
    initPkt.cf = loRaCF.get() / 1e6; // Convert to MHz
    initPkt.rssi = 0;
    
    std::vector<uint8_t> initData = encodePacket(initPkt);
    serialPort->writeData(initData);
    
    EV << "Serial port initialized: " << portName << " @ " << baudRate << " baud" << endl;
    return true;
}

void LoRaHardwareApp::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage()) {
        if (msg == pollTimer) {
            // Poll serial port for incoming data
            pollSerialPort();
            
            // Reschedule poll timer
            scheduleAt(simTime() + pollInterval, pollTimer);
        }
    }
    else {
        // Message from simulation - forward to hardware
        LoRaAppPacket* pkt = check_and_cast<LoRaAppPacket*>(msg);
        
        EV << "Received packet from simulation - Source: " << pkt->getSource() 
           << " Dest: " << pkt->getDestination() << endl;
        
        sendPacketToHardware(pkt);
        delete pkt;
    }
}

void LoRaHardwareApp::pollSerialPort() {
    if (!serialPort || !serialPort->isConnected()) {
        return;
    }
    
    // Check if data is available
    if (serialPort->dataAvailable()) {
        processReceivedSerialData();
    }
}

void LoRaHardwareApp::processReceivedSerialData() {
    std::vector<uint8_t> buffer;
    int bytesRead = serialPort->readData(buffer, 256);
    
    if (bytesRead > 0) {
        EV << "Received " << bytesRead << " bytes from hardware" << endl;
        
        SerialPacket serialPkt;
        if (decodePacket(buffer, serialPkt)) {
            // Convert to LoRa packet and send to simulation
            LoRaAppPacket* loraPkt = convertToLoRaPacket(serialPkt);
            
            if (loraPkt) {
                EV << "Forwarding packet to simulation - Source: " << loraPkt->getSource()
                   << " Dest: " << loraPkt->getDestination() << endl;
                
                send(loraPkt, "appOut");
                packetsForwardedToSim++;
                emit(LoRa_HW_PacketReceived, (long)loraPkt->getSource());
            }
        }
    }
}

void LoRaHardwareApp::sendPacketToHardware(LoRaAppPacket* packet) {
    if (!serialPort || !serialPort->isConnected()) {
        EV_WARN << "Serial port not connected, dropping packet" << endl;
        return;
    }
    
    SerialPacket serialPkt = convertToSerialPacket(packet);
    std::vector<uint8_t> encodedData = encodePacket(serialPkt);
    
    int bytesWritten = serialPort->writeData(encodedData);
    
    if (bytesWritten > 0) {
        EV << "Sent " << bytesWritten << " bytes to hardware device" << endl;
        packetsForwardedToHW++;
        emit(LoRa_HW_PacketSent, (long)packet->getDestination());
    }
}

LoRaHardwareApp::SerialPacket LoRaHardwareApp::convertToSerialPacket(LoRaAppPacket* packet) {
    SerialPacket serialPkt;
    
    serialPkt.msgType = packet->getMsgType();
    serialPkt.source = packet->getSource();
    serialPkt.destination = packet->getDestination();
    serialPkt.via = packet->getVia();
    serialPkt.ttl = packet->getTtl();
    
    // Get LoRa parameters from options
    const LoRaOptions& options = packet->getOptions();
    serialPkt.sf = options.getLoRaSF() >= 0 ? options.getLoRaSF() : loRaSF;
    serialPkt.tp = options.getLoRaTP() >= 0 ? options.getLoRaTP() : loRaTP;
    serialPkt.cf = options.getLoRaCF().get() > 0 ? options.getLoRaCF().get() / 1e6 : loRaCF.get() / 1e6;
    serialPkt.rssi = options.getRSSI();
    
    // Add payload (data int for now)
    int dataInt = packet->getDataInt();
    serialPkt.payload.push_back((dataInt >> 24) & 0xFF);
    serialPkt.payload.push_back((dataInt >> 16) & 0xFF);
    serialPkt.payload.push_back((dataInt >> 8) & 0xFF);
    serialPkt.payload.push_back(dataInt & 0xFF);
    
    return serialPkt;
}

LoRaAppPacket* LoRaHardwareApp::convertToLoRaPacket(const SerialPacket& serialPkt) {
    LoRaAppPacket* packet = new LoRaAppPacket("HW_LoRaPacket");
    
    packet->setMsgType(serialPkt.msgType);
    packet->setSource(serialPkt.source);
    packet->setDestination(serialPkt.destination);
    packet->setVia(serialPkt.via);
    packet->setTtl(serialPkt.ttl);
    
    // Set LoRa options
    LoRaOptions options;
    options.setLoRaSF(serialPkt.sf);
    options.setLoRaTP(serialPkt.tp);
    options.setLoRaCF(units::values::Hz(serialPkt.cf * 1e6));
    options.setRSSI(serialPkt.rssi);
    packet->setOptions(options);
    
    // Extract data int from payload
    if (serialPkt.payload.size() >= 4) {
        int dataInt = (serialPkt.payload[0] << 24) | 
                     (serialPkt.payload[1] << 16) |
                     (serialPkt.payload[2] << 8) | 
                     serialPkt.payload[3];
        packet->setDataInt(dataInt);
    }
    
    packet->setByteLength(20 + serialPkt.payload.size()); // Header + payload
    packet->setDepartureTime(simTime());
    
    return packet;
}

std::vector<uint8_t> LoRaHardwareApp::encodePacket(const SerialPacket& pkt) {
    std::vector<uint8_t> data;
    
    // Protocol: [START][LEN][TYPE][SRC_H][SRC_L][DST_H][DST_L][VIA_H][VIA_L][TTL][SF][TP][CF_H][CF_L][RSSI][PAYLOAD...][CRC]
    
    data.push_back(0xAA); // START byte
    data.push_back(0x55); // START byte 2
    
    uint8_t payloadLen = pkt.payload.size();
    data.push_back(payloadLen + 13); // Length (excluding START, LEN, and CRC)
    
    data.push_back(pkt.msgType);
    data.push_back((pkt.source >> 8) & 0xFF);
    data.push_back(pkt.source & 0xFF);
    data.push_back((pkt.destination >> 8) & 0xFF);
    data.push_back(pkt.destination & 0xFF);
    data.push_back((pkt.via >> 8) & 0xFF);
    data.push_back(pkt.via & 0xFF);
    data.push_back(pkt.ttl);
    data.push_back(pkt.sf);
    data.push_back((uint8_t)pkt.tp);
    uint16_t cf = (uint16_t)pkt.cf;
    data.push_back((cf >> 8) & 0xFF);
    data.push_back(cf & 0xFF);
    data.push_back((uint8_t)(pkt.rssi + 128)); // Convert signed to unsigned
    
    // Add payload
    for (uint8_t byte : pkt.payload) {
        data.push_back(byte);
    }
    
    // Calculate CRC (simple XOR checksum)
    uint8_t crc = 0;
    for (size_t i = 2; i < data.size(); i++) {
        crc ^= data[i];
    }
    data.push_back(crc);
    
    return data;
}

bool LoRaHardwareApp::decodePacket(const std::vector<uint8_t>& data, SerialPacket& pkt) {
    if (data.size() < 18) { // Minimum packet size
        EV_WARN << "Packet too short: " << data.size() << " bytes" << endl;
        return false;
    }
    
    // Check START bytes
    if (data[0] != 0xAA || data[1] != 0x55) {
        EV_WARN << "Invalid START bytes" << endl;
        return false;
    }
    
    uint8_t len = data[2];
    if (data.size() < len + 4) { // START(2) + LEN(1) + DATA(len) + CRC(1)
        EV_WARN << "Incomplete packet" << endl;
        return false;
    }
    
    // Verify CRC
    uint8_t crc = 0;
    for (size_t i = 2; i < len + 3; i++) {
        crc ^= data[i];
    }
    if (crc != data[len + 3]) {
        EV_WARN << "CRC mismatch" << endl;
        return false;
    }
    
    // Decode packet
    pkt.msgType = data[3];
    pkt.source = (data[4] << 8) | data[5];
    pkt.destination = (data[6] << 8) | data[7];
    pkt.via = (data[8] << 8) | data[9];
    pkt.ttl = data[10];
    pkt.sf = data[11];
    pkt.tp = data[12];
    pkt.cf = (data[13] << 8) | data[14];
    pkt.rssi = data[15] - 128; // Convert unsigned to signed
    
    // Extract payload
    pkt.payload.clear();
    for (size_t i = 16; i < len + 3; i++) {
        pkt.payload.push_back(data[i]);
    }
    
    return true;
}

void LoRaHardwareApp::finish() {
    EV << "LoRaHardwareApp statistics:" << endl;
    EV << "  Packets sent to HW: " << packetsSentToHW << endl;
    EV << "  Packets received from HW: " << packetsReceivedFromHW << endl;
    EV << "  Packets forwarded to sim: " << packetsForwardedToSim << endl;
    EV << "  Packets forwarded to HW: " << packetsForwardedToHW << endl;
    
    recordScalar("packetsSentToHW", packetsSentToHW);
    recordScalar("packetsReceivedFromHW", packetsReceivedFromHW);
    recordScalar("packetsForwardedToSim", packetsForwardedToSim);
    recordScalar("packetsForwardedToHW", packetsForwardedToHW);
}

bool LoRaHardwareApp::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback) {
    Enter_Method_Silent();
    
    if (dynamic_cast<NodeStartOperation *>(operation)) {
        if ((NodeStartOperation::Stage)stage == NodeStartOperation::STAGE_APPLICATION_LAYER) {
            if (serialPort && !serialPort->isConnected()) {
                initializeSerialPort();
            }
        }
    }
    else if (dynamic_cast<NodeShutdownOperation *>(operation)) {
        if ((NodeShutdownOperation::Stage)stage == NodeShutdownOperation::STAGE_APPLICATION_LAYER) {
            cancelEvent(pollTimer);
            if (serialPort) {
                serialPort->closePort();
            }
        }
    }
    else if (dynamic_cast<NodeCrashOperation *>(operation)) {
        if ((NodeCrashOperation::Stage)stage == NodeCrashOperation::STAGE_CRASH) {
            cancelEvent(pollTimer);
            if (serialPort) {
                serialPort->closePort();
            }
        }
    }
    
    return true;
}

} // namespace inet
