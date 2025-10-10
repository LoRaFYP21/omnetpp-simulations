// Minimal reliable fragmentation sender app mirroring Arduino logic

#ifndef __LORA_OMNET_RELIABLE_FRAG_APP_H_
#define __LORA_OMNET_RELIABLE_FRAG_APP_H_

#include <omnetpp.h>
#include <string>
#include "LoRaAppPacket_m.h"
#include <inet/common/Units.h>

using namespace omnetpp;

namespace inet {

class ReliableFragApp : public cSimpleModule
{
  protected:
    // Parameters
    int destId = -1;
    int payloadBytes = 0;
    simtime_t sendInterval = 5;
    int fragmentSize = 200;
    int fragMaxTries = 8;
    simtime_t fragAckTimeout = 1;
    int msgMaxTries = 3;
    simtime_t finalAckBaseTimeout = 1.8;
    simtime_t perFragSpacing = 0.015;

    // LoRa control info defaults
    double loRaTP = 17; // dBm
    units::values::Hz loRaCF = units::values::Hz(923e6);
    int loRaSF = 8;
    units::values::Hz loRaBW = units::values::Hz(125e3);
    int loRaCR = 5;
    bool loRaUseHeader = true;

    // State
    int nodeId = -1;
    uint32_t txSeq = 0;
    bool inFlight = false;
    int currentMsgTry = 0;
    int totalFrags = 0;
    int fragIdx = 0;
    int fragTry = 0;
    int singleLen = 0;

    // Counters (like sketch)
    uint64_t txDataPktsTotal = 0;
    uint64_t txBytesTotal = 0;
    simtime_t sessionStart = SIMTIME_ZERO;

    // Timers
    cMessage* sendTimer = nullptr;
    cMessage* fragAckTimer = nullptr;
    cMessage* finalAckTimer = nullptr;

    // Helpers
    void scheduleNextSend();
    void startNewMessage();
    void sendCurrentFragment();
    void handleAck(const LoRaAppPacket* pkt);

  protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
};

}

#endif
