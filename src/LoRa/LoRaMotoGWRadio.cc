//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "LoRaMotoGWRadio.h"
#include "LoRaPhy/LoRaMedium.h"

namespace inet {

namespace physicallayer {

Define_Module(LoRaMotoGWRadio);

void LoRaMotoGWRadio::initialize(int stage)
{
    FlatRadioBase::initialize(stage);
    iAmGateway = par("iAmGateway").boolValue();
    if (stage == INITSTAGE_LAST) {
        setRadioMode(RADIO_MODE_TRANSCEIVER);
        LoRaMotoGWRadioReceptionStarted = registerSignal("LoRaMotoGWRadioReceptionStarted");
        LoRaMotoGWRadioReceptionFinishedCorrect = registerSignal("LoRaMotoGWRadioReceptionFinishedCorrect");
        LoRaMotoGWRadioReceptionStarted_counter = 0;
        LoRaMotoGWRadioReceptionFinishedCorrect_counter = 0;
        iAmTransmiting = false;
    }
}

void LoRaMotoGWRadio::finish()
{
    FlatRadioBase::finish();
    EV_INFO << "Correct finished radio reception count = " << LoRaMotoGWRadioReceptionFinishedCorrect_counter << endl;
    recordScalar("DER - Data Extraction Rate", double(LoRaMotoGWRadioReceptionFinishedCorrect_counter)/LoRaMotoGWRadioReceptionStarted_counter);
}

void LoRaMotoGWRadio::handleSelfMessage(cMessage *message)
{
    if (message == switchTimer)
        handleSwitchTimer(message);
    else if (isTransmissionTimer(message))
        handleTransmissionTimer(message);
    else if (isReceptionTimer(message))
        handleReceptionTimer(message);
    else
        throw cRuntimeError("Unknown self message");
}


bool LoRaMotoGWRadio::isTransmissionTimer(const cMessage *message) const
{
    return !strcmp(message->getName(), "transmissionTimer");
}

void LoRaMotoGWRadio::handleTransmissionTimer(cMessage *message)
{
    if (message->getKind() == IRadioSignal::SIGNAL_PART_WHOLE)
        endTransmission(message);
    else if (message->getKind() == IRadioSignal::SIGNAL_PART_PREAMBLE)
        continueTransmission(message);
    else if (message->getKind() == IRadioSignal::SIGNAL_PART_HEADER)
        continueTransmission(message);
    else if (message->getKind() == IRadioSignal::SIGNAL_PART_DATA)
        endTransmission(message);
    else
        throw cRuntimeError("Unknown self message");
}

void LoRaMotoGWRadio::handleUpperPacket(cPacket *packet)
{
    emit(LayeredProtocolBase::packetReceivedFromUpperSignal, packet);
    if (separateTransmissionParts)
        startTransmission(packet, IRadioSignal::SIGNAL_PART_PREAMBLE);
    else
        startTransmission(packet, IRadioSignal::SIGNAL_PART_WHOLE);
}

void LoRaMotoGWRadio::startTransmission(cPacket *macFrame, IRadioSignal::SignalPart part)
{
    if(iAmTransmiting == false)
    {
        iAmTransmiting = true;
        auto radioFrame = createRadioFrame(macFrame);
        auto transmission = radioFrame->getTransmission();

        cMessage *txTimer = new cMessage("transmissionTimer");
        txTimer->setKind(part);
        txTimer->setContextPointer(const_cast<RadioFrame *>(radioFrame));
        scheduleAt(transmission->getEndTime(part), txTimer);
        EV_INFO << "Transmission started: " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(part) << " as " << transmission << endl;
        check_and_cast<LoRaMedium *>(medium)->emit(IRadioMedium::transmissionStartedSignal, check_and_cast<const cObject *>(transmission));
    }
    else delete macFrame;
}

void LoRaMotoGWRadio::continueTransmission(cMessage *timer)
{
    auto previousPart = (IRadioSignal::SignalPart)timer->getKind();
    auto nextPart = (IRadioSignal::SignalPart)(previousPart + 1);
    auto radioFrame = static_cast<RadioFrame *>(timer->getContextPointer());
    auto transmission = radioFrame->getTransmission();
    EV_INFO << "Transmission ended: " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(previousPart) << " as " << radioFrame->getTransmission() << endl;
    timer->setKind(nextPart);
    scheduleAt(transmission->getEndTime(nextPart), timer);
    EV_INFO << "Transmission started: " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(nextPart) << " as " << transmission << endl;
}

void LoRaMotoGWRadio::endTransmission(cMessage *timer)
{
    iAmTransmiting = false;
    auto part = (IRadioSignal::SignalPart)timer->getKind();
    auto radioFrame = static_cast<RadioFrame *>(timer->getContextPointer());
    auto transmission = radioFrame->getTransmission();
    timer->setContextPointer(nullptr);
    concurrentTransmissions.remove(timer);
    EV_INFO << "Transmission ended: " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(part) << " as " << transmission << endl;
    check_and_cast<LoRaMedium *>(medium)->emit(IRadioMedium::transmissionEndedSignal, check_and_cast<const cObject *>(transmission));
    delete(timer);
}

void LoRaMotoGWRadio::handleLowerPacket(RadioFrame *radioFrame)
{
    auto receptionTimer = createReceptionTimer(radioFrame);
    if (separateReceptionParts)
        startReception(receptionTimer, IRadioSignal::SIGNAL_PART_PREAMBLE);
    else
        startReception(receptionTimer, IRadioSignal::SIGNAL_PART_WHOLE);
}

bool LoRaMotoGWRadio::isReceptionTimer(const cMessage *message) const
{
    return !strcmp(message->getName(), "receptionTimer");
}

void LoRaMotoGWRadio::startReception(cMessage *timer, IRadioSignal::SignalPart part)
{
    auto radioFrame = static_cast<RadioFrame *>(timer->getControlInfo());
    auto arrival = radioFrame->getArrival();
    auto reception = radioFrame->getReception();
    emit(LoRaMotoGWRadioReceptionStarted, true);
    if (simTime() >= getSimulation()->getWarmupPeriod())
        LoRaMotoGWRadioReceptionStarted_counter++;
    if (isReceiverMode(radioMode) && arrival->getStartTime(part) == simTime() && iAmTransmiting == false) {
        auto transmission = radioFrame->getTransmission();
        auto isReceptionAttempted = medium->isReceptionAttempted(this, transmission, part);
        EV_INFO << "LoRaMotoGWRadio Reception started: " << (isReceptionAttempted ? "attempting" : "not attempting") << " " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(part) << " as " << reception << endl;
        if (isReceptionAttempted) {
            if(iAmGateway) {
                concurrentReceptions.push_back(timer);
            }
            receptionTimer = timer;
        }
    }
    else
        EV_INFO << "LoRaMotoGWRadio Reception started: ignoring " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(part) << " as " << reception << endl;
    timer->setKind(part);
    scheduleAt(arrival->getEndTime(part), timer);
    //updateTransceiverState();
    //updateTransceiverPart();
    radioMode = RADIO_MODE_TRANSCEIVER;
    check_and_cast<LoRaMedium *>(medium)->emit(IRadioMedium::receptionStartedSignal, check_and_cast<const cObject *>(reception));
    if(iAmGateway) EV << "[MSDebug] start reception, size : " << concurrentReceptions.size() << endl;
}

void LoRaMotoGWRadio::continueReception(cMessage *timer)
{
    auto previousPart = (IRadioSignal::SignalPart)timer->getKind();
    auto nextPart = (IRadioSignal::SignalPart)(previousPart + 1);
    auto radioFrame = static_cast<RadioFrame *>(timer->getControlInfo());
    auto arrival = radioFrame->getArrival();
    auto reception = radioFrame->getReception();
    if(iAmGateway) {
        std::list<cMessage *>::iterator it;
        for (it=concurrentReceptions.begin(); it!=concurrentReceptions.end(); it++) {
            if(*it == timer) receptionTimer = timer;
        }
    }
    if (timer == receptionTimer && isReceiverMode(radioMode) && arrival->getEndTime(previousPart) == simTime() && iAmTransmiting == false) {

        auto transmission = radioFrame->getTransmission();
        bool isReceptionSuccessful = medium->isReceptionSuccessful(this, transmission, previousPart);
        EV_INFO << "LoRaMotoGWRadio 00 Reception ended: " << (isReceptionSuccessful ? "successfully" : "unsuccessfully") << " for " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(previousPart) << " as " << reception << endl;
        if (!isReceptionSuccessful) {
            receptionTimer = nullptr;
            if(iAmGateway) concurrentReceptions.remove(timer);
        }
        auto isReceptionAttempted = medium->isReceptionAttempted(this, transmission, nextPart);
        EV_INFO << "LoRaMotoGWRadio Reception started: " << (isReceptionAttempted ? "attempting" : "not attempting") << " " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(nextPart) << " as " << reception << endl;
        if (!isReceptionAttempted) {
            receptionTimer = nullptr;
            if(iAmGateway) concurrentReceptions.remove(timer);
        }
    }
    else {
        EV_INFO << "LoRaMotoGWRadio Reception ended: ignoring " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(previousPart) << " as " << reception << endl;
        EV_INFO << "LoRaMotoGWRadio Reception started: ignoring " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(nextPart) << " as " << reception << endl;
    }
    timer->setKind(nextPart);
    scheduleAt(arrival->getEndTime(nextPart), timer);
    //updateTransceiverState();
    //updateTransceiverPart();
    radioMode = RADIO_MODE_TRANSCEIVER;
}

void LoRaMotoGWRadio::endReception(cMessage *timer)
{
    auto part = (IRadioSignal::SignalPart)timer->getKind();
    auto radioFrame = static_cast<RadioFrame *>(timer->getControlInfo());
    auto arrival = radioFrame->getArrival();
    auto reception = radioFrame->getReception();
    std::list<cMessage *>::iterator it;
    if(iAmGateway) {
        for (it=concurrentReceptions.begin(); it!=concurrentReceptions.end(); it++) {
            if(*it == timer) receptionTimer = timer;
        }
    }
    if (timer == receptionTimer && isReceiverMode(radioMode) && arrival->getEndTime() == simTime() && iAmTransmiting == false) {
        auto transmission = radioFrame->getTransmission();
// TODO: this would draw twice from the random number generator in isReceptionSuccessful: auto isReceptionSuccessful = medium->isReceptionSuccessful(this, transmission, part);
        auto isReceptionSuccessful = medium->getReceptionDecision(this, radioFrame->getListening(), transmission, part)->isReceptionSuccessful();
        isReceptionSuccessful = true;
        EV_INFO << "LoRaMotoGWRadio 01 Reception ended: " << (isReceptionSuccessful ? "successfully" : "unsuccessfully") << " for " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(part) << " as " << reception << endl;
        if(isReceptionSuccessful) {
            auto macFrame = medium->receivePacket(this, radioFrame);
            emit(LayeredProtocolBase::packetSentToUpperSignal, macFrame);
            emit(LoRaMotoGWRadioReceptionFinishedCorrect, true);
            if (simTime() >= getSimulation()->getWarmupPeriod())
                LoRaMotoGWRadioReceptionFinishedCorrect_counter++;
            sendUp(macFrame);
        }
        receptionTimer = nullptr;
        if(iAmGateway) concurrentReceptions.remove(timer);
    }
    else
        EV_INFO << "LoRaMotoGWRadio Reception ended: ignoring " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(part) << " as " << reception << endl;
    //updateTransceiverState();
    //updateTransceiverPart();
    radioMode = RADIO_MODE_TRANSCEIVER;
    check_and_cast<LoRaMedium *>(medium)->emit(IRadioMedium::receptionEndedSignal, check_and_cast<const cObject *>(reception));
    delete timer;
}

void LoRaMotoGWRadio::abortReception(cMessage *timer)
{
    auto radioFrame = static_cast<RadioFrame *>(timer->getControlInfo());
    auto part = (IRadioSignal::SignalPart)timer->getKind();
    auto reception = radioFrame->getReception();
    EV_INFO << "LoRaMotoGWRadio Reception aborted: for " << (IRadioFrame *)radioFrame << " " << IRadioSignal::getSignalPartName(part) << " as " << reception << endl;
    if (timer == receptionTimer) {
        if(iAmGateway) concurrentReceptions.remove(timer);
        receptionTimer = nullptr;
    }
    updateTransceiverState();
    updateTransceiverPart();
}
}

}
