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

#include "LoRaMotoGWApp.h"
#include "../LoRa/LoRaMac.h"
#include "../LoRa/LoRaMacFrame_m.h"

#include "inet/mobility/static/StationaryMobility.h"
namespace inet {

Define_Module(LoRaMotoGWApp);

void LoRaMotoGWApp::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        std::pair<double,double> coordsValues = std::make_pair(-1, -1);

        cModule *host = getContainingNode(this);
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;

        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;

        if (!isOperational)
            throw cRuntimeError("This module doesn't support starting in node DOWN state");

        do {
            timeToFirstPacket = par("timeToFirstPacket");
            EV << "Wylosowalem czas :" << timeToFirstPacket << endl;
        } while(timeToFirstPacket <= 5);

        //timeToFirstPacket = par("timeToFirstPacket");
        sendMeasurements = new cMessage("sendMeasurements");
        scheduleAt(simTime()+timeToFirstPacket, sendMeasurements);

        sentPackets = 0;
        receivedPackets = 0;

        receivedADRCommands = 0;
        numberOfPacketsToSend = par("numberOfPacketsToSend");
        numberOfPacketsToForward = par("numberOfPacketsToForward");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        //LoRa physical layer parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaCR = par("initialLoRaCR");
        loRaUseHeader = par("initialUseHeader");
        evaluateADRinNode = par("evaluateADRinNode");
        sfVector.setName("SF Vector");
        tpVector.setName("TP Vector");
    }
}

void LoRaMotoGWApp::finish()
{
    cModule *host = getContainingNode(this);
    recordScalar("sentPackets", sentPackets);
    recordScalar("receivedPackets", receivedPackets);
}

void LoRaMotoGWApp::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {

    }
    else {
        handleMessageFromLowerLayer(msg);
    }
}

void LoRaMotoGWApp::handleMessageFromLowerLayer(cMessage *msg)
{
    LoRaMacFrame *frame = check_and_cast<LoRaMacFrame *>(msg);

    DevAddr transmitter = frame->getTransmitterAddress();
    EV << transmitter << endl;
    EV << transmitter.str() << endl;
    EV << transmitter.getInt() << endl;

    if (simTime() >= getSimulation()->getWarmupPeriod()) {
        //Process the message

        receivedPackets++;
        receivedPacketsStats.record(receivedPackets);
    }

    delete msg;
}

bool LoRaMotoGWApp::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
//    Enter_Method_Silent();
//
//    throw cRuntimeError("Unsupported lifecycle operation '%s'", operation->getClassName());
//    return true;
    return true;
}



} //end namespace inet
