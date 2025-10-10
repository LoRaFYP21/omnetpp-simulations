// Minimal reliable fragmentation sender app mirroring Arduino logic

#include "ReliableFragApp.h"
#include "inet/common/ModuleAccess.h"
#include "LoRa/LoRaMacControlInfo_m.h"

namespace inet {

Define_Module(ReliableFragApp);

static const int MSGKIND_SEND = 1001;
static const int MSGKIND_FRAG_ACK_TIMEOUT = 1002;
static const int MSGKIND_FINAL_ACK_TIMEOUT = 1003;

void ReliableFragApp::initialize(int stage) {
    if (stage != INITSTAGE_LOCAL) return;

    nodeId = getContainingNode(this)->getIndex();

    destId = par("destId");
    payloadBytes = par("payloadBytes");
    sendInterval = par("sendInterval");
    fragmentSize = par("fragmentSize");
    fragMaxTries = par("fragMaxTries");
    fragAckTimeout = par("fragAckTimeout");
    msgMaxTries = par("msgMaxTries");
    finalAckBaseTimeout = par("finalAckBaseTimeout");
    perFragSpacing = par("perFragSpacing");
    sendOnce = par("sendOnce");

    loRaTP = par("initialLoRaTP").doubleValue();
    loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
    loRaSF = par("initialLoRaSF");
    loRaBW = units::values::Hz(par("initialLoRaBW").doubleValue());
    loRaCR = par("initialLoRaCR");
    loRaUseHeader = par("initialUseHeader");

    sendTimer = new cMessage("sendTimer", MSGKIND_SEND);
    fragAckTimer = new cMessage("fragAckTimer", MSGKIND_FRAG_ACK_TIMEOUT);
    finalAckTimer = new cMessage("finalAckTimer", MSGKIND_FINAL_ACK_TIMEOUT);

    sessionStart = simTime();
    scheduleNextSend();
}

void ReliableFragApp::scheduleNextSend() {
    if (!sendTimer->isScheduled())
        scheduleAt(simTime() + sendInterval, sendTimer);
}

void ReliableFragApp::startNewMessage() {
    inFlight = true;
    currentMsgTry = 1;
    txSeq++;
    // compute fragments
    if (payloadBytes <= fragmentSize) {
        totalFrags = 1;
        singleLen = payloadBytes;
    } else {
        totalFrags = (payloadBytes + fragmentSize - 1) / fragmentSize;
        singleLen = -1;
    }
    fragIdx = 0;
    fragTry = 0;

    // kick off first fragment or single packet
    sendCurrentFragment();
}

void ReliableFragApp::sendCurrentFragment() {
    bool single = (totalFrags == 1 && singleLen >= 0);
    int chunkLen;
    if (single) {
        chunkLen = singleLen;
    } else {
        int L = payloadBytes;
        int off = fragIdx * fragmentSize;
        int end = std::min(L, off + fragmentSize);
        chunkLen = std::max(0, end - off);
    }

    auto *pkt = new LoRaAppPacket("DATA");
    pkt->setMsgType(AppPacketType::DATA);
    pkt->setSource(nodeId);
    pkt->setDestination(destId);
    pkt->setTtl(0);
    pkt->setVia(0);
    pkt->setSeq((int)txSeq);
    pkt->setIsFragment(!single);
    pkt->setFragIdx(single ? -1 : fragIdx);
    pkt->setFragTot(single ? -1 : totalFrags);
    pkt->setPayloadBytes(chunkLen);

    // attach LoRa control info
    auto *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);
    cInfo->setLoRaUseHeader(loRaUseHeader);
    pkt->setControlInfo(cInfo);

    send(pkt, "appOut");

    txDataPktsTotal++;
    txBytesTotal += chunkLen;

    // wait for appropriate ACK
    if (!single) {
        fragTry++;
        if (fragAckTimer->isScheduled()) cancelEvent(fragAckTimer);
        scheduleAt(simTime() + fragAckTimeout, fragAckTimer);
    } else {
        if (finalAckTimer->isScheduled()) cancelEvent(finalAckTimer);
        scheduleAt(simTime() + finalAckBaseTimeout, finalAckTimer);
    }
}

void ReliableFragApp::handleAck(const LoRaAppPacket* pkt) {
    int seq = pkt->getSeq();
    if (!inFlight || seq != (int)txSeq) return; // not for current message

    if (pkt->getIsFinalAck()) {
        // delivery complete
        inFlight = false;
        if (finalAckTimer->isScheduled()) cancelEvent(finalAckTimer);
        // Emit scalars
        recordScalar("txDataPktsTotal", (double)txDataPktsTotal);
        recordScalar("txBytesTotal", (double)txBytesTotal);
        recordScalar("ack_peerRxBytes", (double)pkt->getRxTotBytes());
        recordScalar("ack_peerRxPkts", (double)pkt->getRxTotPkts());
        double pdr = (txDataPktsTotal > 0) ? ((double)pkt->getRxTotPkts() / (double)txDataPktsTotal) : 0.0;
        recordScalar("pdr", pdr);
        double elapsedMs = (simTime() - sessionStart).dbl() * 1000.0;
        double bps = (elapsedMs > 0) ? (pkt->getRxTotBytes() * 8.0 * 1000.0 / elapsedMs) : 0.0;
        recordScalar("goodput_bps", bps);
        // schedule next message only if allowed
        messagesSent++;
        if (!sendOnce)
            scheduleNextSend();
        return;
    }

    // fragment ACKF
    if (pkt->getFragIdx() == fragIdx) {
        // got expected ACKF
        if (fragAckTimer->isScheduled()) cancelEvent(fragAckTimer);
        fragIdx++;
        fragTry = 0;
        if (fragIdx >= totalFrags) {
            // all fragments sent; wait for final ACK (longer, depends on N)
            simtime_t finalWait = finalAckBaseTimeout + totalFrags * 0.5;
            if (finalAckTimer->isScheduled()) cancelEvent(finalAckTimer);
            scheduleAt(simTime() + finalWait, finalAckTimer);
        } else {
            // send next fragment immediately
            sendCurrentFragment();
        }
    }
}

void ReliableFragApp::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage()) {
        if (msg->getKind() == MSGKIND_SEND) {
            if (!inFlight)
                startNewMessage();
        }
        else if (msg->getKind() == MSGKIND_FRAG_ACK_TIMEOUT) {
            // retry current fragment
            if (!inFlight) return;
            if (fragTry < fragMaxTries) {
                scheduleAt(simTime() + perFragSpacing, fragAckTimer); // reuse timer as guard between tries
                sendCurrentFragment();
            } else {
                // fragment failed => retry whole message if possible
                if (currentMsgTry < msgMaxTries) {
                    currentMsgTry++;
                    fragIdx = 0;
                    fragTry = 0;
                    // restart the same message (keep seq)
                    sendCurrentFragment();
                } else {
                    // give up this message
                    inFlight = false;
                    recordScalar("msgFailed", 1);
                    if (!sendOnce)
                        scheduleNextSend();
                }
            }
        }
        else if (msg->getKind() == MSGKIND_FINAL_ACK_TIMEOUT) {
            // retry whole message on final ack timeout
            if (!inFlight) return;
            if (currentMsgTry < msgMaxTries) {
                currentMsgTry++;
                fragIdx = 0;
                fragTry = 0;
                // restart the same message (keep seq)
                sendCurrentFragment();
            } else {
                inFlight = false;
                recordScalar("msgFailed", 1);
                if (!sendOnce)
                    scheduleNextSend();
            }
        }
        return;
    }

    // Incoming from lower layer
    auto *pkt = check_and_cast<LoRaAppPacket*>(msg);
    if (pkt->getMsgType() == AppPacketType::ACK) {
        handleAck(pkt);
    }
    delete pkt;
}

void ReliableFragApp::finish() {
    recordScalar("txDataPktsTotal", (double)txDataPktsTotal);
    recordScalar("txBytesTotal", (double)txBytesTotal);
}

} // namespace inet
