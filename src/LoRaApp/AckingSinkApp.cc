// Simple sink that ACKs fragments and final message, mirroring Arduino logic

#include "AckingSinkApp.h"
#include "LoRa/LoRaMacControlInfo_m.h"
#include "inet/common/ModuleAccess.h"

namespace inet {

Define_Module(AckingSinkApp);

void AckingSinkApp::initialize(int stage) {
    if (stage != INITSTAGE_LOCAL) return;
    nodeId = getContainingNode(this)->getIndex();
    loRaTP = par("initialLoRaTP").doubleValue();
    loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
    loRaSF = par("initialLoRaSF");
    loRaBW = units::values::Hz(par("initialLoRaBW").doubleValue());
    loRaCR = par("initialLoRaCR");
    loRaUseHeader = par("initialUseHeader");
    resetReasm();
}

void AckingSinkApp::resetReasm() {
    reSrc = -1; reSeq = -1; reTot = 0; reGot = 0; fragBytes.clear(); have.clear();
}

void AckingSinkApp::startReasm(int src, int seq, int tot) {
    reSrc = src; reSeq = seq; reTot = tot; reGot = 0; fragBytes.assign(tot, 0); have.assign(tot, false);
}

bool AckingSinkApp::addFrag(int idx, int chunkLen) {
    if (idx < 0 || idx >= reTot) return false;
    if (!have[idx]) { have[idx] = true; fragBytes[idx] = chunkLen; reGot++; return true; }
    return false;
}

void AckingSinkApp::sendFragAck(int dst, int seq, int idx) {
    auto *ack = new LoRaAppPacket("ACKF");
    ack->setMsgType(AppPacketType::ACK);
    ack->setSource(nodeId);
    ack->setDestination(dst);
    ack->setSeq(seq);
    ack->setIsFinalAck(false);
    ack->setFragIdx(idx);
    auto *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);
    cInfo->setLoRaUseHeader(loRaUseHeader);
    ack->setControlInfo(cInfo);
    send(ack, "appOut");
}

void AckingSinkApp::sendFinalAck(int dst, int seq) {
    auto *ack = new LoRaAppPacket("ACK");
    ack->setMsgType(AppPacketType::ACK);
    ack->setSource(nodeId);
    ack->setDestination(dst);
    ack->setSeq(seq);
    ack->setIsFinalAck(true);
    ack->setRxTotBytes(rxBytesTotal);
    ack->setRxTotPkts(rxDataPktsTotal);
    auto *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);
    cInfo->setLoRaUseHeader(loRaUseHeader);
    ack->setControlInfo(cInfo);
    send(ack, "appOut");
}

void AckingSinkApp::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage()) { delete msg; return; }
    auto *pkt = check_and_cast<LoRaAppPacket*>(msg);

    if (pkt->getMsgType() == AppPacketType::DATA) {
        // Single or fragment
        if (!pkt->getIsFragment()) {
            rxDataPktsTotal++;
            rxBytesTotal += pkt->getPayloadBytes();
            // final ACK
            sendFinalAck(pkt->getSource(), pkt->getSeq());
        } else {
            // start or continue reassembly
            if (pkt->getSource() != reSrc || pkt->getSeq() != reSeq) {
                startReasm(pkt->getSource(), pkt->getSeq(), pkt->getFragTot());
            }
            bool fresh = addFrag(pkt->getFragIdx(), pkt->getPayloadBytes());
            if (fresh) { rxDataPktsTotal++; rxBytesTotal += pkt->getPayloadBytes(); }
            // per-fragment ACK
            sendFragAck(pkt->getSource(), pkt->getSeq(), pkt->getFragIdx());

            // if complete, send final ACK
            bool all = (reGot == reTot);
            if (all) {
                sendFinalAck(pkt->getSource(), pkt->getSeq());
                resetReasm();
            }
        }
    }

    delete pkt;
}

void AckingSinkApp::finish() {
    recordScalar("rxDataPktsTotal", (double)rxDataPktsTotal);
    recordScalar("rxBytesTotal", (double)rxBytesTotal);
}

} // namespace inet
