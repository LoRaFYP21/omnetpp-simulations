// Simple sink that ACKs fragments and final message, mirroring Arduino logic

#ifndef __LORA_OMNET_ACKING_SINK_APP_H_
#define __LORA_OMNET_ACKING_SINK_APP_H_

#include <omnetpp.h>
#include <vector>
#include "LoRaAppPacket_m.h"
#include <inet/common/Units.h>

using namespace omnetpp;

namespace inet {

class AckingSinkApp : public cSimpleModule
{
  protected:
    int nodeId = -1;
    // LoRa control info defaults for ACKs
    double loRaTP = 17;
    units::values::Hz loRaCF = units::values::Hz(923e6);
    int loRaSF = 8;
    units::values::Hz loRaBW = units::values::Hz(125e3);
    int loRaCR = 5;
    bool loRaUseHeader = true;
    // Reassembly state (one in-flight per peer)
    int reSrc = -1;
    int reSeq = -1;
    int reTot = 0;
    int reGot = 0;
    std::vector<int> fragBytes;  // bytes per fragment
    std::vector<bool> have;

    // Counters for reporting back
    uint64_t rxDataPktsTotal = 0; // counts MSG + unique MSGF frags
    uint64_t rxBytesTotal = 0;    // app TEXT bytes totals

    void resetReasm();
    void startReasm(int src, int seq, int tot);
    bool addFrag(int idx, int chunkLen);
    void sendFragAck(int dst, int seq, int idx);
    void sendFinalAck(int dst, int seq);

  protected:
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
};

}

#endif
