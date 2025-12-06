//
// LoRaHardwareApp.h - Application layer for hardware-in-the-loop LoRa device
//

#ifndef __LORA_OMNET_LORAHARDWAREAPP_H_
#define __LORA_OMNET_LORAHARDWAREAPP_H_

#include <omnetpp.h>
#include <string>
#include <queue>
#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "LoRaAppPacket_m.h"
#include "../misc/SerialPortInterface.h"

using namespace omnetpp;

namespace inet {

/**
 * Hardware-in-the-Loop LoRa Application
 * Bridges physical LilyGO LORA32 device with OMNeT++ simulation
 */
class INET_API LoRaHardwareApp : public cSimpleModule, public ILifecycle
{
protected:
    // Serial communication
    SerialPortInterface* serialPort;
    std::string portName;
    int baudRate;
    
    // Node identification
    int nodeId;
    
    // Timers
    cMessage* pollTimer;
    simtime_t pollInterval;
    
    // Statistics
    simsignal_t LoRa_HW_PacketSent;
    simsignal_t LoRa_HW_PacketReceived;
    int packetsSentToHW;
    int packetsReceivedFromHW;
    int packetsForwardedToSim;
    int packetsForwardedToHW;
    
    // LoRa parameters
    double loRaTP;
    units::values::Hz loRaCF;
    int loRaSF;
    
    // Packet processing
    struct SerialPacket {
        uint8_t msgType;
        uint16_t source;
        uint16_t destination;
        uint16_t via;
        uint8_t ttl;
        uint8_t sf;
        double tp;
        double cf;
        double rssi;
        std::vector<uint8_t> payload;
    };
    
    std::queue<SerialPacket> txQueue;
    
protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    virtual bool handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback) override;
    
    // Serial communication methods
    virtual bool initializeSerialPort();
    virtual void pollSerialPort();
    virtual void sendPacketToHardware(LoRaAppPacket* packet);
    virtual void processReceivedSerialData();
    
    // Packet conversion methods
    virtual SerialPacket convertToSerialPacket(LoRaAppPacket* packet);
    virtual LoRaAppPacket* convertToLoRaPacket(const SerialPacket& serialPkt);
    
    // Protocol methods
    virtual std::vector<uint8_t> encodePacket(const SerialPacket& pkt);
    virtual bool decodePacket(const std::vector<uint8_t>& data, SerialPacket& pkt);
    
public:
    LoRaHardwareApp() : serialPort(nullptr), pollTimer(nullptr) {}
    virtual ~LoRaHardwareApp();
};

} // namespace inet

#endif
