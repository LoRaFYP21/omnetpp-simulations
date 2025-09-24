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
#include <iostream>

#include "LoRaNodeApp.h"
#include "inet/common/FSMA.h"
#include "../LoRa/LoRaMac.h"
#include <sstream>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif


#include "inet/mobility/static/StationaryMobility.h"
namespace inet {

#define BROADCAST_ADDRESS   16777215


#define NO_FORWARDING                 0
#define FLOODING_BROADCAST_SINGLE_SF  1
#define SMART_BROADCAST_SINGLE_SF     2
#define HOP_COUNT_SINGLE_SF           3
#define RSSI_SUM_SINGLE_SF            4
#define RSSI_PROD_SINGLE_SF           5
#define ETX_SINGLE_SF                 6
#define TIME_ON_AIR_HC_CAD_SF        11
#define TIME_ON_AIR_SF_CAD_SF        12

Define_Module (LoRaNodeApp);

void LoRaNodeApp::initialize(int stage) {

    cSimpleModule::initialize(stage);
    routingMetric = par("routingMetric");

    //Current network settings
    // numberOfNodes = par("numberOfNodes");
    // std::cout << "numberOfNodes: " << numberOfNodes << std::endl;
    // numberOfEndNodes = par("numberOfEndNodes");
    // std::cout << "numberOfEndNodes: " << numberOfEndNodes << std::endl;

    if(routingMetric == 0){ // for the END nodes, where no forwarding
        numberOfNodes = par("numberOfEndNodes");
    }
    else{
        numberOfNodes = par("numberOfNodes"); // relay nodes

    }

    if (stage == INITSTAGE_LOCAL) {
        // Get this node's ID
        nodeId = getContainingNode(this)->getIndex();
        std::pair<double, double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);

        // Generate random location for nodes if circle deployment type
        if (strcmp(host->par("deploymentType").stringValue(), "circle") == 0) {
            coordsValues = generateUniformCircleCoordinates(
                    host->par("rad").doubleValue(),
                    host->par("centX").doubleValue(),
                    host->par("centY").doubleValue());
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            mobility->par("initialX").setDoubleValue(coordsValues.first);
            mobility->par("initialY").setDoubleValue(coordsValues.second);

        } else if (strcmp(host->par("deploymentType").stringValue(), "edges")== 0) {
            double minX = host->par("minX");
            double maxX = host->par("maxX");
            double minY = host->par("minY");
            double maxY = host->par("maxY");
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
//            if (strcmp(host->par("deploymentType").stringValue(), "circle")==0) {
//                       coordsValues = generateUniformCircleCoordinates(host->par("maxGatewayDistance").doubleValue(), host->par("gatewayX").doubleValue(), host->par("gatewayY").doubleValue());
//                       StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
//                       mobility->par("initialX").setDoubleValue(coordsValues.first);
//                       mobility->par("initialY").setDoubleValue(coordsValues.second);
//                    }

            mobility->par("initialX").setDoubleValue(
                    minX + maxX * (((nodeId + 1) % 4 / 2) % 2));
            mobility->par("initialY").setDoubleValue(
                    minY + maxY * (((nodeId) % 4 / 2) % 2));
        } else if (strcmp(host->par("deploymentType").stringValue(), "grid") == 0) {
            double minX = host->par("minX");
            double sepX = host->par("sepX");
            double minY = host->par("minY");
            double sepY = host->par("sepY");
            int cols = int(sqrt(numberOfNodes));
            int idx = getContainingNode(this)->getIndex();
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            if (idx == 0 && routingMetric == 0){ // end node 0 at middle
                mobility->par("initialX").setDoubleValue(minX + sepX * (cols/2));
                mobility->par("initialY").setDoubleValue(minY + sepY * (cols/2)+ uniform(0,100));
            }
            else{
                mobility->par("initialX").setDoubleValue(
                        minX + sepX * (idx % cols) + uniform(0,100));
                mobility->par("initialY").setDoubleValue(
                        minY + sepY * ((int) idx / cols) + uniform(0,100));
            }
        } else {
            double minX = host->par("minX");
            double maxX = host->par("maxX");
            double minY = host->par("minY");
            double maxY = host->par("maxY");
            //creating the possibility to give a position with the X and Y coordinates.
            double inix = host->par("initialX");
            double iniy = host->par("initialY");
                // checking whether the node is a end node or not.
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            mobility->par("initialX").setDoubleValue(inix);
            mobility->par("initialY").setDoubleValue(iniy);
        }
    } else if (stage == INITSTAGE_APPLICATION_LAYER) {
        bool isOperational;

        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;

        if (!isOperational)
        {
            throw cRuntimeError("This module doesn't support starting in node DOWN state");
        }

        // Initialize counters
        sentPackets = 0;
        sentDataPackets = 0;
        sentRoutingPackets = 0;
        sentAckPackets = 0;
        receivedPackets = 0;
        receivedPacketsForMe = 0;
        receivedPacketsFromMe = 0;
        receivedPacketsToForward = 0;
        receivedRoutingPackets = 0;
        receivedDataPackets = 0;
        receivedDataPacketsForMe = 0;
        receivedDataPacketsForMeUnique = 0;
        receivedDataPacketsFromMe = 0;
        receivedDataPacketsToForward = 0;
        receivedDataPacketsToForwardCorrect = 0;
        receivedDataPacketsToForwardExpired = 0;
        receivedDataPacketsToForwardUnique = 0;
        receivedAckPackets = 0;
        receivedAckPacketsForMe = 0;
        receivedAckPacketsFromMe = 0;
        receivedAckPacketsToForward = 0;
        receivedAckPacketsToForwardCorrect = 0;
        receivedAckPacketsToForwardExpired = 0;
        receivedAckPacketsToForwardUnique = 0;
        receivedADRCommands = 0;
        forwardedPackets = 0;
        forwardedDataPackets = 0;
        forwardedAckPackets = 0;
        forwardPacketsDuplicateAvoid = 0;
        packetsToForwardMaxVectorSize = 0;
        broadcastDataPackets = 0;
        broadcastForwardedPackets = 0;
        deletedRoutes = 0;
        forwardBufferFull = 0;

        firstDataPacketTransmissionTime = 0;
        lastDataPacketTransmissionTime = 0;
        firstDataPacketReceptionTime = 0;
        lastDataPacketReceptionTime = 0;

        dataPacketsDue = false;
        forwardPacketsDue = false;
        routingPacketsDue = false;

        sendPacketsContinuously = par("sendPacketsContinuously");
        onlyNode0SendsPackets = par("onlyNode0SendsPackets");
        enforceDutyCycle = par("enforceDutyCycle");
        dutyCycle = par("dutyCycle");
        numberOfDestinationsPerNode = par("numberOfDestinationsPerNode");
        numberOfPacketsPerDestination = par("numberOfPacketsPerDestination");

        numberOfPacketsToForward = par("numberOfPacketsToForward");

        packetsToForwardMaxVectorSize = par("packetsToForwardMaxVectorSize");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");

        currDataInt = 0;

        //LoRa physical layer parameters
        loRaTP = par("initialLoRaTP").doubleValue();
        loRaCF = units::values::Hz(par("initialLoRaCF").doubleValue());
        loRaSF = par("initialLoRaSF");
        minLoRaSF = par("minLoRaSF");
        maxLoRaSF = par("maxLoRaSF");
        if (loRaSF < minLoRaSF) {
            loRaSF = minLoRaSF;
        }
        else if (loRaSF > maxLoRaSF) {
            loRaSF = maxLoRaSF;
        }
        loRaBW = inet::units::values::Hz(par("initialLoRaBW").doubleValue());
        loRaCR = par("initialLoRaCR");
        loRaUseHeader = par("initialUseHeader");
        loRaCAD = par("initialLoRaCAD");
        loRaCADatt = par("initialLoRaCADatt").doubleValue();
        evaluateADRinNode = par("evaluateADRinNode");
        txSfVector.setName("Tx1 SF Vector");
        txTpVector.setName("Tx1 TP Vector");
        rxRssiVector.setName("Rx1 RSSI Vector");
        rxSfVector.setName("Rx1 SF Vector");

        // DistanceX.setName("Distance X Vector");

        // DistanceY.setName("Distance Y Vector");

        //Routing variables
        routingMetric = par("routingMetric");
        routeDiscovery = par("routeDiscovery");
        // Route discovery must be enabled for broadcast-based smart forwarding
        switch (routingMetric) {
            case SMART_BROADCAST_SINGLE_SF:
                routeDiscovery = true;
        }
        routingPacketPriority = par("routingPacketPriority");
        ownDataPriority = par("ownDataPriority");
        routeTimeout = par("routeTimeout");
        storeBestRoutesOnly = par("storeBestRouteOnly");
        getRoutesFromDataPackets = par("getRoutesFromDataPackets");
        packetTTL = par("packetTTL");
        stopRoutingAfterDataDone = par("stopRoutingAfterDataDone");

    // AODV-lite params
    if (hasPar("useAODV")) useAODV = par("useAODV");
    if (hasPar("selectedTxNodeId")) selectedTxNodeId = par("selectedTxNodeId");
    if (hasPar("selectedRxNodeId")) selectedRxNodeId = par("selectedRxNodeId");

        windowSize = std::min(32, std::max<int>(1, par("windowSize").intValue())); //Must be an int between 1 and 32
        // cModule *host = getContainingNode(this);

//         // bool iAmEnd = host->par("iAmEnd");
//         cModule* parentModule = getParentModule();
//         bool iAmEnd = parentModule->par("iAmEnd").boolValue();

//         EV << "iAmEnd: " << iAmEnd << endl;
// //        std::cout << "Hello, world!" << std::endl;
//         std::cout << "The node I am end as follows!" << std::endl;
//         std::cout << "iAmEnd value: " << iAmEnd << std::endl;

//         if (iAmEnd) {
//             // Code to execute if iAmEnd is true
//             std::cout << "This is an end node." << std::endl;
//             // Perform any actions specific to end nodes here
//         } else {
//             // Code to execute if iAmEnd is false
//             std::cout << "This is not an end node." << std::endl;
//             // Perform actions for non-end nodes here
//         }


        // if (iAmEnd){
        //     packetTTL = 0;

        // }
        if ( packetTTL == 0) {
            if (strcmp(getContainingNode(this)->par("deploymentType").stringValue(), "grid") == 0) {
//                packetTTL = 2*(sqrt(numberOfNodes)-1);
                packetTTL = (numberOfNodes)-1;
                // packetTTL = 0;
                if (routingMetric != 0) {
//                    packetTTL = 0;
                    packetTTL = (numberOfNodes)-1;
                    //packetTTL = math::max(2,2*(sqrt(numberOfNodes)-1));
                }
            }
            else {
//                packetTTL = 0;
                packetTTL = 2*(sqrt(numberOfNodes));
                if (routingMetric != 0) {
//                    packetTTL = 0;
                    //packetTTL = math::max(2,2*(sqrt(numberOfNodes)-1));
                    packetTTL = (numberOfNodes)-1;
                    //print the packetTTL value
                    EV << "packetTTL value is " << packetTTL << endl;
                }
            }
        }
//        else{
//            packetTTL = 0;
//        }

        //Packet sizes
        dataPacketSize = par("dataPacketDefaultSize");
        routingPacketMaxSize = par("routingPacketMaxSize");

        // Data packets timing
        timeToNextDataPacketMin = par("timeToNextDataPacketMin");
        timeToNextDataPacketMax = par("timeToNextDataPacketMax");
        timeToNextDataPacketAvg = par("timeToNextDataPacketAvg");

        // Routing packets timing
        timeToNextRoutingPacketMin = par("timeToNextRoutingPacketMin");
        timeToNextRoutingPacketMax = par("timeToNextRoutingPacketMax");
        timeToNextRoutingPacketAvg = par("timeToNextRoutingPacketAvg");

        simTimeResolution = pow(10, simTimeResolution.getScaleExp());

        neighbourNodes = {};
        knownNodes = {};
        LoRaPacketsToSend = {};
        LoRaPacketsToForward = {};
        LoRaPacketsForwarded = {};
        DataPacketsForMe = {};
        ACKedNodes = {};

    //Routing table
        singleMetricRoutingTable = {};
        dualMetricRoutingTable = {};

    // Prepare routing CSV path (per-node file)
    openRoutingCsv();

    //Node identifier (optionally offset to avoid collisions across arrays)
    nodeId = getContainingNode(this)->getIndex();
    if (hasPar("idBase")) idBase = par("idBase");
    nodeId += idBase;

        //Application acknowledgment
        requestACKfromApp = par("requestACKfromApp");
        stopOnACK = par("stopOnACK");
        AppACKReceived = false;
        firstACK = 0;

        //Spreading factor
        increaseSF = par("increaseSF");
        firstACKSF = 0;
        packetsPerSF = par("packetsPerSF");
        packetsInSF = 0;

        //Forwarded packets vector size
        forwardedPacketVectorSize = par("forwardedPacketVectorSize");

        //WATCHES only for GUI
        if (getEnvir()->isGUI()) {
            WATCH(sentPackets);
            WATCH(sentDataPackets);
            WATCH(sentRoutingPackets);
            WATCH(sentAckPackets);
            WATCH(receivedPackets);
            WATCH(receivedPacketsForMe);
            WATCH(receivedPacketsFromMe);
            WATCH(receivedPacketsToForward);
            WATCH(receivedRoutingPackets);
            WATCH(receivedDataPackets);
            WATCH(receivedDataPacketsForMe);
            WATCH(receivedDataPacketsForMeUnique);
            WATCH(receivedDataPacketsFromMe);
            WATCH(receivedDataPacketsToForward);
            WATCH(receivedDataPacketsToForwardCorrect);
            WATCH(receivedDataPacketsToForwardExpired);
            WATCH(receivedDataPacketsToForwardUnique);
            WATCH(receivedAckPackets);
            WATCH(receivedAckPacketsForMe);
            WATCH(receivedAckPacketsFromMe);
            WATCH(receivedAckPacketsToForward);
            WATCH(receivedAckPacketsToForwardCorrect);
            WATCH(receivedAckPacketsToForwardExpired);
            WATCH(receivedAckPacketsToForwardUnique);
            WATCH(receivedADRCommands);
            WATCH(forwardedPackets);
            WATCH(forwardedDataPackets);
            WATCH(forwardedAckPackets);
            WATCH(forwardPacketsDuplicateAvoid);
            WATCH(packetsToForwardMaxVectorSize);
            WATCH(broadcastDataPackets);
            WATCH(broadcastForwardedPackets);
            WATCH(deletedRoutes);
            WATCH(forwardBufferFull);

            WATCH(AppACKReceived);
            WATCH(firstACK);
            WATCH(packetTTL);
            WATCH(loRaSF);
            WATCH(packetsInSF);

            WATCH_VECTOR(neighbourNodes);
            WATCH_VECTOR(knownNodes);
            WATCH_VECTOR(ACKedNodes);

            WATCH(firstDataPacketTransmissionTime);
            WATCH(lastDataPacketTransmissionTime);
            WATCH(firstDataPacketReceptionTime);
            WATCH(lastDataPacketReceptionTime);

            //WATCH_VECTOR(singleMetricRoutingTable);
            //WATCH_VECTOR(dualMetricRoutingTable);

            WATCH_VECTOR(LoRaPacketsToSend);
            WATCH_VECTOR(LoRaPacketsToForward);
            WATCH_VECTOR(LoRaPacketsForwarded);
            WATCH_VECTOR(DataPacketsForMe);
        }

        if (numberOfDestinationsPerNode == 0 ) {
            numberOfDestinationsPerNode = numberOfNodes-1;
            EV<< "printing node ID" << endl;
        }
        generateDataPackets();

        // Routing packets timer (disabled when AODV mode is enabled)
        if (!useAODV) {
            timeToFirstRoutingPacket = math::max(5, par("timeToFirstRoutingPacket"))+getTimeToNextRoutingPacket();
            switch (routingMetric) {
                // No routing packets are to be sent
                case NO_FORWARDING:
                case FLOODING_BROADCAST_SINGLE_SF:
                case SMART_BROADCAST_SINGLE_SF:
                    break;
                // Schedule selfRoutingPackets
                default:
                    routingPacketsDue = true;
                    nextRoutingPacketTransmissionTime = timeToFirstRoutingPacket;
                    EV << "Time to first routing packet: " << timeToFirstRoutingPacket << endl;
                    break;
            }
        } else {
            routingPacketsDue = false;
        }

        // Data packets timer
        timeToFirstDataPacket = math::max(5, par("timeToFirstDataPacket"))+getTimeToNextDataPacket();
        if (LoRaPacketsToSend.size() > 0) {
                    dataPacketsDue = true;
                    nextDataPacketTransmissionTime = timeToFirstDataPacket;
                    EV << "Time to first data packet: " << timeToFirstDataPacket << endl;
        }

        // Forward packets timer
        timeToFirstForwardPacket = math::max(5, par("timeToFirstForwardPacket"))+getTimeToNextForwardPacket();
        // THis should not happen, though
        if (LoRaPacketsToForward.size() > 0) {
                    forwardPacketsDue = true;
                    nextForwardPacketTransmissionTime = timeToFirstForwardPacket;
                    EV << "Time to first forward packet: " << timeToFirstForwardPacket << endl;
        }


        selfPacket = new cMessage("selfPacket");
        EV_INFO << "selfPacket vinuja" <<endl;
        if (dataPacketsDue || forwardPacketsDue || routingPacketsDue) {

            // Only data packet due
            if (dataPacketsDue && !forwardPacketsDue && !routingPacketsDue) {
                scheduleAt(simTime() + timeToFirstDataPacket, selfPacket);
                EV << "Self packet triggered by due data packet" << endl;
            }
            // Only forward packet due
            else if (routingPacketsDue && !dataPacketsDue && !forwardPacketsDue) {
                scheduleAt(simTime() + timeToFirstRoutingPacket, selfPacket);
                EV << "Self packet triggered by due routing packet" << endl;
            }
            // Only routing packet due
            else if (forwardPacketsDue && !dataPacketsDue && !routingPacketsDue) {
                scheduleAt(simTime() + timeToFirstForwardPacket, selfPacket);
                EV << "Self packet triggered by due forward packet" << endl;
            }
            // Data packet due earlier
            else if (timeToFirstDataPacket < timeToFirstForwardPacket && timeToFirstDataPacket < timeToFirstRoutingPacket ) {
                scheduleAt(simTime() + timeToFirstDataPacket, selfPacket);
                EV << "Self packet triggered by due data packet before other due packets" << endl;
            }
            // Forward packet due earlier
                else if (timeToFirstForwardPacket < timeToFirstDataPacket && timeToFirstForwardPacket < timeToFirstRoutingPacket ) {
                scheduleAt(simTime() + timeToFirstForwardPacket, selfPacket);
                EV << "Self packet triggered by due forward packet before other due packets" << endl;
            }
            else {
                scheduleAt(simTime() + timeToFirstRoutingPacket, selfPacket);
                EV << "Self packet triggered by due routing packet before other due packets" << endl;
            }
        }

        dutyCycleEnd = simTime();
    }
}

std::pair<double, double> LoRaNodeApp::generateUniformCircleCoordinates(
    double radius, double centX, double centY) {
    // Do NOT overwrite member nodeId here; it may carry idBase offset for AODV.
    int idx = getContainingNode(this)->getIndex();
    routingMetric = par("routingMetric");

    if (idx == 0 && routingMetric == 0) // only for the end nodes and the packet originator
    {
        // return std::make_pair(centX, centY);
        std::pair<double, double> coordValues = std::make_pair(centX, centY);
        return coordValues;
    }
    
    double randomValueRadius = uniform(0, (radius * radius));
    double randomTheta = uniform(0, 2 * M_PI);

    // generate coordinates for circle with origin at 0,0
    double x = sqrt(randomValueRadius) * cos(randomTheta);
    double y = sqrt(randomValueRadius) * sin(randomTheta);
    // Change coordinates based on coordinate system used in OMNeT, with origin at top left
    x = x + centX;
    y = centY - y;

//    EV_INFO << "MY__X__________" << x <<endl;
//    EV_INFO << "MY__Y__________ " << y <<endl;

//    double z = ()

    std::cout << " MY__X__________"  << x << std::endl;
    std::cout << " MY__Y__________ "  << y << std::endl;

//    DistanceX.record(x);
//    DistanceY.record(y);

    EV_INFO << " MY__X__________" << x << endl;
    EV_INFO << " MY__Y__________" << y << endl;


    EV_INFO << "selfPacket " <<endl;
    std::pair<double, double> coordValues = std::make_pair(x, y);
    return coordValues;
}

void LoRaNodeApp::finish() {
    cModule *host = getContainingNode(this);
    StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
            host->getSubmodule("mobility"));
    Coord coord = mobility->getCurrentPosition();
//    recordScalar("positionX", coord.x);
//    recordScalar("positionY", coord.y);
    // DistanceX.record(coord.x);
    recordScalar("CordiX",coord.x);
    // DistanceY.record(coord.y);
    recordScalar("CordiY",coord.y);


    recordScalar("finalTP", loRaTP);
    recordScalar("finalSF", loRaSF);

    recordScalar("sentPackets", sentPackets);
    recordScalar("sentDataPackets", sentDataPackets);
    recordScalar("sentRoutingPackets", sentRoutingPackets);
    recordScalar("sentAckPackets", sentAckPackets);
    recordScalar("receivedPackets", receivedPackets);
    recordScalar("receivedPacketsForMe", receivedPacketsForMe);
    recordScalar("receivedPacketsFromMe", receivedPacketsFromMe);
    recordScalar("receivedPacketsToForward", receivedPacketsToForward);
    recordScalar("receivedDataPackets", receivedDataPackets);
    recordScalar("receivedDataPacketsForMe", receivedDataPacketsForMe);
    recordScalar("receivedDataPacketsForMeUnique", receivedDataPacketsForMeUnique);
    recordScalar("receivedDataPacketsFromMe", receivedDataPacketsFromMe);
    recordScalar("receivedDataPacketsToForward", receivedDataPacketsToForward);
    recordScalar("receivedDataPacketsToForwardCorrect",
            receivedDataPacketsToForwardCorrect);
    recordScalar("receivedDataPacketsToForwardExpired",
            receivedDataPacketsToForwardExpired);
    recordScalar("receivedDataPacketsToForwardUnique",
            receivedDataPacketsToForwardUnique);
    recordScalar("receivedAckPacketsToForward", receivedAckPacketsToForward);
    recordScalar("receivedAckPacketsToForwardCorrect",
            receivedAckPacketsToForwardCorrect);
    recordScalar("receivedAckPacketsToForwardExpired",
            receivedAckPacketsToForwardExpired);
    recordScalar("receivedAckPacketsToForwardUnique",
            receivedAckPacketsToForwardUnique);
    recordScalar("receivedAckPackets", receivedAckPackets);
    recordScalar("receivedAckPacketsForMe", receivedAckPacketsForMe);
    recordScalar("receivedAckPacketsFromMe", receivedAckPacketsFromMe);
    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("forwardedPackets", forwardedPackets);
    recordScalar("forwardedDataPackets", forwardedDataPackets);
    recordScalar("forwardedAckPackets", forwardedAckPackets);
    recordScalar("forwardPacketsDuplicateAvoid", forwardPacketsDuplicateAvoid);
    recordScalar("packetsToForwardMaxVectorSize", packetsToForwardMaxVectorSize);
    recordScalar("broadcastDataPackets", broadcastDataPackets);
    recordScalar("broadcastForwardedPackets", broadcastForwardedPackets);

    recordScalar("firstDataPacketTransmissionTime", firstDataPacketTransmissionTime);
    recordScalar("lastDataPacketTransmissionTime", lastDataPacketTransmissionTime);
    recordScalar("firstDataPacketReceptionTime", firstDataPacketReceptionTime);
    recordScalar("lastDataPacketReceptionTime", lastDataPacketReceptionTime);

    recordScalar("receivedADRCommands", receivedADRCommands);
    recordScalar("AppACKReceived", AppACKReceived);
    recordScalar("firstACK", firstACK);
    recordScalar("firstACKSF", firstACKSF);

    recordScalar("dataPacketsNotSent", LoRaPacketsToSend.size());
    recordScalar("forwardPacketsNotSent", LoRaPacketsToSend.size());

    recordScalar("forwardBufferFull", forwardBufferFull);

    for (std::vector<LoRaAppPacket>::iterator lbptr = LoRaPacketsToSend.begin();
            lbptr < LoRaPacketsToSend.end(); lbptr++) {
        LoRaPacketsToSend.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsToForward.begin(); lbptr < LoRaPacketsToForward.end();
            lbptr++) {
        LoRaPacketsToForward.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsForwarded.begin(); lbptr < LoRaPacketsForwarded.end();
            lbptr++) {
        LoRaPacketsForwarded.erase(lbptr);
    }

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            DataPacketsForMe.begin(); lbptr < DataPacketsForMe.end();
            lbptr++) {
        DataPacketsForMe.erase(lbptr);
    }

    recordScalar("dataPacketsForMeLatencyMax", dataPacketsForMeLatency.getMax());
    recordScalar("dataPacketsForMeLatencyMean", dataPacketsForMeLatency.getMean());
    recordScalar("dataPacketsForMeLatencyMin", dataPacketsForMeLatency.getMin());
    recordScalar("dataPacketsForMeLatencyStdv", dataPacketsForMeLatency.getStddev());

    recordScalar("dataPacketsForMeUniqueLatencyMax", dataPacketsForMeUniqueLatency.getMax());
    recordScalar("dataPacketsForMeUniqueLatencyMean", dataPacketsForMeUniqueLatency.getMean());
    recordScalar("dataPacketsForMeUniqueLatencyMin", dataPacketsForMeUniqueLatency.getMin());
    recordScalar("dataPacketsForMeUniqueLatencyStdv", dataPacketsForMeUniqueLatency.getStddev());

    recordScalar("routingTableSizeMax", routingTableSize.getMax());
    recordScalar("routingTableSizeMean", routingTableSize.getMean());
    recordScalar("routingTableSizeMin", routingTableSize.getMin());
    recordScalar("routingTableSizeStdv", routingTableSize.getStddev());

    recordScalar("allTxPacketsSFStatsMax", allTxPacketsSFStats.getMax());
    recordScalar("allTxPacketsSFStatsMean", allTxPacketsSFStats.getMean());
    recordScalar("allTxPacketsSFStatsMin", allTxPacketsSFStats.getMin());
    recordScalar("allTxPacketsSFStatsStdv", allTxPacketsSFStats.getStddev());
    recordScalar("routingTxPacketsSFStatsMax", routingTxPacketsSFStats.getMax());
    recordScalar("routingTxPacketsSFStatsMean", routingTxPacketsSFStats.getMean());
    recordScalar("routingTxPacketsSFStatsMin", routingTxPacketsSFStats.getMin());
    recordScalar("routingTxPacketsSFStatsStdv", routingTxPacketsSFStats.getStddev());
    recordScalar("owndataTxPacketsSFStatsMax", routingTxPacketsSFStats.getMax());
    recordScalar("owndataTxPacketsSFStatsMean", routingTxPacketsSFStats.getMean());
    recordScalar("owndataTxPacketsSFStatsMin", routingTxPacketsSFStats.getMin());
    recordScalar("owndataTxPacketsSFStatsStdv", routingTxPacketsSFStats.getStddev());
    recordScalar("fwdTxPacketsSFStatsMax", routingTxPacketsSFStats.getMax());
    recordScalar("fwdTxPacketsSFStatsMean", routingTxPacketsSFStats.getMean());
    recordScalar("fwdTxPacketsSFStatsMin", routingTxPacketsSFStats.getMin());
    recordScalar("fwdTxPacketsSFStatsStdv", routingTxPacketsSFStats.getStddev());

    dataPacketsForMeLatency.recordAs("dataPacketsForMeLatency");
    dataPacketsForMeUniqueLatency.recordAs("dataPacketsForMeUniqueLatency");

    // No persistent CSV stream; snapshots overwrite per write
}

void LoRaNodeApp::handleMessage(cMessage *msg) {

    if (msg->isSelfMessage()) {
        handleSelfMessage(msg);
    } else {
        handleMessageFromLowerLayer(msg);
    }
}


void LoRaNodeApp::handleSelfMessage(cMessage *msg) {
    // Guard to prevent double scheduling
    inSelfHandler = true;

    // Received a selfMessage for transmitting a scheduled packet.  Only proceed to send a packet
    // if the 'mac' module in 'LoRaNic' is IDLE and the warmup period is due (TODO: implement check for the latter).
    LoRaMac *lrmc = (LoRaMac *)getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac");
    if (lrmc->fsm.getState() == IDLE ) {

        simtime_t txDuration = 0;
        simtime_t nextScheduleTime = 0;

        bool sendData = false;
        bool sendForward = false;
    bool sendRouting = false;
    bool sendAodv = false;

        // Check if there are data packets to send, and if it is time to send them
        // TODO: Not using dataPacketsDue ???
        if ( LoRaPacketsToSend.size() > 0 && simTime() >= nextDataPacketTransmissionTime ) {
            sendData = true;
        }

        // Check if there are data packets to forward, and if it is time to send them
        // TODO: Not using forwardPacketsDue ???
        if ( LoRaPacketsToForward.size() > 0 && simTime() >= nextForwardPacketTransmissionTime ) {
            sendForward = true;
        }

        // Check if there are routing packets to send (disabled in AODV mode)
        if ( !useAODV && routingPacketsDue && simTime() >= nextRoutingPacketTransmissionTime ) {
            sendRouting = true;
        }

        // AODV control packets due?
        if (useAODV && aodvPacketsToSend.size() > 0) {
            sendAodv = true;
        }

        // Now there could be between none and three types of packets due to be sent. Decide between routing and the
        // other two types randomly with the probability from the routingPacketPriotity parameter
        if (sendRouting && (sendData || sendForward) ) {
            // Either send a routing packet...
            if (bernoulli(routingPacketPriority)) {
                sendData = false;
                sendForward = false;
            }
            else {
                sendRouting = false;
            }
        }

    // Send routing packet
        if (sendRouting) {
            txDuration = sendRoutingPacket();
            if (enforceDutyCycle) {
                // Update duty cycle end
                dutyCycleEnd = simTime() + txDuration/dutyCycle;
                // Update next routing packet transmission time, taking the duty cycle into account
                nextRoutingPacketTransmissionTime = simTime() + math::max(getTimeToNextRoutingPacket().dbl(), txDuration.dbl()/dutyCycle);
            }
            else {
                // Update next routing packet transmission time
                nextRoutingPacketTransmissionTime = simTime() + math::max(getTimeToNextRoutingPacket().dbl(), txDuration.dbl());
            }
        }

        // Send AODV control first (small, unblock discovery), else data/forward
        else if (sendAodv) {
            LoRaAppPacket *ctrl = new LoRaAppPacket(aodvPacketsToSend.front());
            aodvPacketsToSend.erase(aodvPacketsToSend.begin());
            LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
            cInfo->setLoRaTP(loRaTP);
            cInfo->setLoRaCF(loRaCF);
            cInfo->setLoRaSF(loRaSF);
            cInfo->setLoRaBW(loRaBW);
            cInfo->setLoRaCR(loRaCR);
            ctrl->setControlInfo(cInfo);
            EV_INFO << "AODV ctrl tx from queue at node=" << nodeId << " type=" << ctrl->getMsgType() << endl;
            std::cout << "AODV ctrl tx node=" << nodeId << " type=" << ctrl->getMsgType() << std::endl;
            send(ctrl, "appOut");
            // small tx duration estimate to pace scheduling
            txDuration = 0.2; // conservative small value
            aodvPacketsDue = (aodvPacketsToSend.size() > 0);
        }

        // Send data or forward packet
        else if (sendData || sendForward) {

            // If both data and forward packets are due, decide randomly between the two with the probability from the
            // ownDataPriority parameter
            if (sendData && sendForward) {
                if (bernoulli(ownDataPriority))
                    // Send own data packet
                    sendForward = false;
                else
                    // Send forward packet
                    sendData = false;
            }

            // Send data packet
            if (sendData) {

                txDuration = sendDataPacket();
                if (enforceDutyCycle) {
                    // Update duty cycle end
                    dutyCycleEnd = simTime() + txDuration/dutyCycle;
                    // Update next data packet transmission time, taking the duty cycle into account
                    nextDataPacketTransmissionTime = simTime() + math::max(getTimeToNextDataPacket().dbl(), txDuration.dbl()/dutyCycle);
                }
                else {
                    // Update next data packet transmission time
                    nextDataPacketTransmissionTime = simTime() + math::max(getTimeToNextDataPacket().dbl(), txDuration.dbl());
                }
            }
            // or send forward packet
            else {
                txDuration = sendForwardPacket();
                if (enforceDutyCycle) {
                    // Update duty cycle end
                    dutyCycleEnd = simTime() + txDuration/dutyCycle;
                    // Update next forward packet transmission time, taking the duty cycle into account
                    nextForwardPacketTransmissionTime = simTime() + math::max(getTimeToNextForwardPacket().dbl(), txDuration.dbl()/dutyCycle);
                }
                else {
                // Update next forward packet transmission time
                    nextForwardPacketTransmissionTime = simTime() + math::max(getTimeToNextForwardPacket().dbl(), txDuration.dbl());
                }
            }
        }

        // We've sent a packet (routing, data or forward). Now reschedule a selfMessage if needed.
        if ( LoRaPacketsToSend.size() > 0 )
            dataPacketsDue = true;
        if ( LoRaPacketsToForward.size() > 0 )
            forwardPacketsDue = false;
        // routingPackets due is handled below.

        // Calculate next schedule time, first based on routing packets next transmission time,
        if (routingPacketsDue) {
            nextScheduleTime = nextRoutingPacketTransmissionTime.dbl();
        }
        if (aodvPacketsDue) {
            // schedule soon for next AODV control
            nextScheduleTime = nextScheduleTime == 0 ? (simTime() + 0.1) : std::min(nextScheduleTime.dbl(), (simTime() + 0.1).dbl());
        }
        // then based on data packets, if they are to be scheduled earlier,
        if (dataPacketsDue) {
            nextScheduleTime = std::min(nextScheduleTime.dbl(), nextDataPacketTransmissionTime.dbl());
        }
        // then based on forward packets, if they are to be scheduled earlier,
        if (forwardPacketsDue) {
            nextScheduleTime = std::min(nextScheduleTime.dbl(), nextForwardPacketTransmissionTime.dbl());
        }
        // but, in any case, not earlier than simTime()+txDuration.
        nextScheduleTime = math::max(nextScheduleTime.dbl(), simTime().dbl()+txDuration.dbl());

        // Take the duty cycle into account
        if (enforceDutyCycle) {
            nextScheduleTime = math::max(nextScheduleTime.dbl(), dutyCycleEnd.dbl());
        }

        // Last, although this should never happen, check the schedule time is in the future, otherwise just add a 1s delay
        if (! (nextScheduleTime > simTime()) ) {
            nextScheduleTime = simTime() + 1;
        }

        // Schedule a self message to send routing, data or forward packets. Since some calculations lose precision, add an extra delay
        // (10x simtime-resolution unit) to avoid timing conflicts in the LoRaMac layer when simulations last very long.
        if (routingPacketsDue || dataPacketsDue || forwardPacketsDue || aodvPacketsDue) {
            if (selfPacket->isScheduled()) cancelEvent(selfPacket);
            scheduleAt(nextScheduleTime + 10*simTimeResolution, selfPacket);
        }

        if (!sendPacketsContinuously && routingPacketsDue) {

            bool allNodesDone = true;

            for (int i=0; i<numberOfNodes; i++) {
                LoRaNodeApp *lrndpp = (LoRaNodeApp *) getParentModule()->getParentModule()->getSubmodule("loRaNodes", i)->getSubmodule("LoRaNodeApp");
                if ( !(lrndpp->lastDataPacketTransmissionTime > 0 && \
                     // ToDo: maybe too restrictive? If no packets were received at all
                     // simulation laster until the very end
                     //lrndpp->lastDataPacketReceptionTime > 0 &&
                     lrndpp->lastDataPacketTransmissionTime + stopRoutingAfterDataDone < simTime() && \
                     lrndpp->lastDataPacketReceptionTime + stopRoutingAfterDataDone < simTime() )) {
                    allNodesDone = false;
                    break;
                }

                if (allNodesDone) {
                    routingPacketsDue = false;
                }
            }
        }
    }


    // The LoRa radio was busy with a reception, so re-schedule the selfMessage a bit later
    else {
        // Instead of doing scheduling almost immediately: scheduleAt(simTime() + 10*simTimeResolution, selfPacket);
        // wait 20 microseconds, which is approx. the transmission time for 1 bit (SF7, 125 kHz, 4:5)
        if (selfPacket->isScheduled()) cancelEvent(selfPacket);
        scheduleAt(simTime() + 0.00002, selfPacket);
    }

    inSelfHandler = false;
}

void LoRaNodeApp::handleMessageFromLowerLayer(cMessage *msg) {
    receivedPackets++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // AODV-lite control handling
    if (packet->getMsgType() == RREQ) {
        handleRreq(packet);
        delete msg;
        return;
    }
    if (packet->getMsgType() == RREP) {
        if (packet->getDestination() == nodeId || packet->getVia() == nodeId) {
            handleRrep(packet);
        } else {
            EV_INFO << "AODV RREP rx but not for this node (via=" << packet->getVia()
                    << ", dest=" << packet->getDestination() << ") at node=" << nodeId << ". Ignoring." << endl;
            std::cout << "AODV RREP ignore node=" << nodeId << " via=" << packet->getVia()
                      << " dest=" << packet->getDestination() << std::endl;
        }
        delete msg;
        return;
    }

    // Check if the packet is from this node (i.e., a packet that some
    // other node is broadcasting which we have happened to receive). We
    // count it and discard it immediately.
    if (packet->getSource() == nodeId) {
        receivedDataPacketsFromMe++;
        bubble("I received a LoRa packet originally sent by me!");
        if (firstDataPacketReceptionTime == 0) {
            firstDataPacketReceptionTime = simTime();
        }
        lastDataPacketReceptionTime = simTime();
    }
    // Else, check if the packet is for this node (i.e., a packet directly
    // received from the origin or relayed by a neighbour)
    else if (packet->getDestination() == nodeId) {
        bubble("I received a data packet for me!");

        std::cout << "msg type at dest: " << packet->getMsgType() << std::endl;

        manageReceivedPacketForMe(packet);
        if (firstDataPacketReceptionTime == 0) {
            firstDataPacketReceptionTime = simTime();
        }
        lastDataPacketReceptionTime = simTime();
    }
    // Else it can be a routing protocol broadcast message
    else if (packet->getDestination() == BROADCAST_ADDRESS) {
        manageReceivedRoutingPacket(packet);
    }
    // Else it can be a data packet from and to other nodes...
    else {
        // which we may forward, if it is being broadcast
        if (packet->getVia() == BROADCAST_ADDRESS && routeDiscovery == true) {

            std::cout << "msg type broadcast and route : " << packet->getMsgType() << std::endl;

            bubble("I received a multicast data packet to forward!");
            manageReceivedDataPacketToForward(packet);
            if (firstDataPacketReceptionTime == 0) {
                firstDataPacketReceptionTime = simTime();
            }
            lastDataPacketReceptionTime = simTime();
        }
        // or unicast via this node
        else if (packet->getVia() == nodeId) {
            bubble("I received a unicast data packet to forward!");
            manageReceivedDataPacketToForward(packet);
            if (firstDataPacketReceptionTime == 0) {
                firstDataPacketReceptionTime = simTime();
            }
            lastDataPacketReceptionTime = simTime();
        }
        // or not, if it's a unicast packet we just happened to receive.
        else {
            bubble("Unicast message not for me! but still forwarding");
            manageReceivedDataPacketToForward(packet);
            receivedDataPackets++;
            lastDataPacketReceptionTime = simTime();
        }
    }

    delete msg;
}

void LoRaNodeApp::manageReceivedRoutingPacket(cMessage *msg) {
    //does not execting this part in our application

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // Check it actually is a routing message
    if (packet->getMsgType() == RREQ) { handleRreq(packet); return; }
    if (packet->getMsgType() == RREP) { handleRrep(packet); return; }
    if (packet->getMsgType() == ROUTING) {

        receivedRoutingPackets++;

    sanitizeRoutingTable();

        switch (routingMetric) {

            // The node performs no forwarding
            case NO_FORWARDING:
                bubble("Discarding routing packet as forwarding is disabled");
                break;

            // Forwarding is broadcast-based
            case FLOODING_BROADCAST_SINGLE_SF:
            case SMART_BROADCAST_SINGLE_SF:
                bubble("Discarding routing packet as forwarding is broadcast-based");
                break;

            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
                bubble("Processing routing packet");

                // Add route to new neighbour node...
                if (!isRouteInSingleMetricRoutingTable(packet->getSource(), packet->getSource()) ) {
                    EV << "Adding neighbour " << packet->getSource() << endl;
                    singleMetricRoute newNeighbour;
                        newNeighbour.id = packet->getSource();
                        newNeighbour.via = packet->getSource();
                        newNeighbour.valid = simTime() + routeTimeout;
                        switch (routingMetric) {
                            case HOP_COUNT_SINGLE_SF:
                                newNeighbour.metric = 1;
                                break;
                            case RSSI_SUM_SINGLE_SF:
                            case RSSI_PROD_SINGLE_SF:
                                newNeighbour.metric = std::abs(packet->getOptions().getRSSI());
                                break;
                            case ETX_SINGLE_SF:
                                newNeighbour.metric = 1;
                                newNeighbour.window[0] = packet->getDataInt();
                                // Fill window[1] and beyond, until windowSizeth element, with 0's
                                for (int i = 1; i<windowSize; i++) {
                                    newNeighbour.window[i] = 0;
                                }
                                break;
                        }

                    if (storeBestRoutesOnly) {
                        addOrReplaceBestSingleRoute(newNeighbour);
                    } else {
                        singleMetricRoutingTable.push_back(newNeighbour);
                    }
                }

                // or refresh route to known neighbour.
                else {
                    int routeIndex = getRouteIndexInSingleMetricRoutingTable(packet->getSource(), packet->getSource());
                    if (routeIndex >= 0) {
                        singleMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                        // Besides the route validity time, each metric may need different things to be updated
                        switch (routingMetric) {
                            // RSSI may change over time (e.g., different Tx power, mobility...)
                            case RSSI_SUM_SINGLE_SF:
                            case RSSI_PROD_SINGLE_SF:
                                singleMetricRoutingTable[routeIndex].metric = std::abs(packet->getOptions().getRSSI());
                                break;
                            // Metric must be recalculated and window must be updated
                            case ETX_SINGLE_SF:
                                int metric = 1;
                                // Calculate the metric based on the window of previously received routing packets and update it
                                for (int i=0; i<windowSize; i++) {
                                    metric = metric + (packet->getDataInt() - (singleMetricRoutingTable[routeIndex].window[i] + i + 1));
                                }
                                singleMetricRoutingTable[routeIndex].metric = std::max(1, metric);
                                // Update the window
                                for (int i=windowSize; i>0; i--) {
                                    singleMetricRoutingTable[routeIndex].window[i] = singleMetricRoutingTable[routeIndex].window[i-1];
                                }
                                singleMetricRoutingTable[routeIndex].window[0] = packet->getDataInt();
                                break;
                        }
                     }
                }

                // Iterate the routes in the incoming packet and add them to the routing table, or update them
                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId) {
                        // Add new route
                        if (!isRouteInSingleMetricRoutingTable(thisRoute.getId(), packet->getSource())) {
                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << endl;

                            singleMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.valid = simTime() + routeTimeout;
                            switch(routingMetric) {
                            case HOP_COUNT_SINGLE_SF:
                                newRoute.metric = thisRoute.getPriMetric()+1;
                                break;
                            case RSSI_SUM_SINGLE_SF:
                                newRoute.metric = thisRoute.getPriMetric()+std::abs(packet->getOptions().getRSSI());
                                break;
                            case RSSI_PROD_SINGLE_SF:
                                newRoute.metric = thisRoute.getPriMetric()*std::abs(packet->getOptions().getRSSI());
                                break;
                            case ETX_SINGLE_SF:
                                newRoute.metric = \
                                    singleMetricRoutingTable[getRouteIndexInSingleMetricRoutingTable(packet->getSource(), packet->getSource())].metric \
                                    + thisRoute.getPriMetric();
                                break;
                            }

                            if (storeBestRoutesOnly) {
                                addOrReplaceBestSingleRoute(newRoute);
                            } else {
                                singleMetricRoutingTable.push_back(newRoute);
                            }
                        }
                        // Or update known one
                        else {
                            int routeIndex = getRouteIndexInSingleMetricRoutingTable(thisRoute.getId(), packet->getSource());
                            if (routeIndex >= 0) {
                                switch (routingMetric) {
                                    case HOP_COUNT_SINGLE_SF:
                                        singleMetricRoutingTable[routeIndex].metric = thisRoute.getPriMetric()+1;
                                        break;
                                    case RSSI_SUM_SINGLE_SF:
                                        singleMetricRoutingTable[routeIndex].metric = thisRoute.getPriMetric()+std::abs(packet->getOptions().getRSSI());
                                        break;
                                    case RSSI_PROD_SINGLE_SF:
                                        singleMetricRoutingTable[routeIndex].metric = thisRoute.getPriMetric()*std::abs(packet->getOptions().getRSSI());
                                        break;
                                    case ETX_SINGLE_SF:
                                        singleMetricRoutingTable[routeIndex].metric = \
                                            singleMetricRoutingTable[getRouteIndexInSingleMetricRoutingTable(packet->getSource(), packet->getSource())].metric \
                                            + thisRoute.getPriMetric();
                                        break;
                                }
                                singleMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                                // If keeping only best route, ensure table consistency against other candidates
                                if (storeBestRoutesOnly) {
                                    addOrReplaceBestSingleRoute(singleMetricRoutingTable[routeIndex]);
                                }
                            }
                        }
                    }
                }
                break;

            case TIME_ON_AIR_HC_CAD_SF:
                bubble("Processing routing packet");

                if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getSource(), packet->getOptions().getLoRaSF())) {
//                    EV << "Adding neighbour " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;

                    dualMetricRoute newNeighbour;
                    newNeighbour.id = packet->getSource();
                    newNeighbour.via = packet->getSource();
                    newNeighbour.sf = packet->getOptions().getLoRaSF();
                    newNeighbour.priMetric = pow(2, packet->getOptions().getLoRaSF() - 7);
                    newNeighbour.secMetric = 1;
                    newNeighbour.valid = simTime() + routeTimeout;
                    dualMetricRoutingTable.push_back(newNeighbour);
                }

                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId ) {
                        // Add new route
                        if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getVia(), packet->getOptions().getLoRaSF())) {
//                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;
                            dualMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.sf = packet->getOptions().getLoRaSF();
                            newRoute.priMetric = thisRoute.getPriMetric() + pow(2, packet->getOptions().getLoRaSF());
                            newRoute.secMetric = thisRoute.getSecMetric() + 1;
                            newRoute.valid = simTime() + routeTimeout;
                        }
                    }
                    // Or update known one
                    else {
                        int routeIndex = getRouteIndexInDualMetricRoutingTable(thisRoute.getId(), packet->getSource(), packet->getOptions().getLoRaSF());
                        if (routeIndex >= 0) {
                            dualMetricRoutingTable[routeIndex].priMetric = thisRoute.getPriMetric() + pow(2, packet->getOptions().getLoRaSF());
                            dualMetricRoutingTable[routeIndex].secMetric = thisRoute.getSecMetric() + 1;
                            dualMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                        }
                    }
                }

//                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;
                break;

            case TIME_ON_AIR_SF_CAD_SF:
                bubble("Processing routing packet");

//                EV << "Processing routing packet in node " << nodeId << endl;
//                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;

                if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getSource(), packet->getOptions().getLoRaSF())) {
//                    EV << "Adding neighbour " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;

                    dualMetricRoute newNeighbour;
                    newNeighbour.id = packet->getSource();
                    newNeighbour.via = packet->getSource();
                    newNeighbour.sf = packet->getOptions().getLoRaSF();
                    newNeighbour.priMetric = pow(2, packet->getOptions().getLoRaSF() - 7);
                    newNeighbour.secMetric = packet->getOptions().getLoRaSF() - 7;
                    newNeighbour.valid = simTime() + routeTimeout;
                    dualMetricRoutingTable.push_back(newNeighbour);
                }

                for (int i = 0; i < packet->getRoutingTableArraySize(); i++) {
                    LoRaRoute thisRoute = packet->getRoutingTable(i);

                    if (thisRoute.getId() != nodeId ) {
                        // Add new route
                        if ( !isRouteInDualMetricRoutingTable(packet->getSource(), packet->getVia(), packet->getOptions().getLoRaSF())) {
//                            EV << "Adding route to node " << thisRoute.getId() << " via " << packet->getSource() << " with SF " << packet->getOptions().getLoRaSF() << endl;
                            dualMetricRoute newRoute;
                            newRoute.id = thisRoute.getId();
                            newRoute.via = packet->getSource();
                            newRoute.sf = packet->getOptions().getLoRaSF();
                            newRoute.priMetric = thisRoute.getPriMetric() + pow(2, packet->getOptions().getLoRaSF());
                            newRoute.secMetric = thisRoute.getSecMetric() + packet->getOptions().getLoRaSF() - 7;
                            newRoute.valid = simTime() + routeTimeout;
                        }
                        // Or update known one
                        else {
                            int routeIndex = getRouteIndexInDualMetricRoutingTable(thisRoute.getId(), packet->getSource(), packet->getOptions().getLoRaSF());
                            if (routeIndex >= 0) {
                                dualMetricRoutingTable[routeIndex].priMetric = thisRoute.getPriMetric() + pow(2, packet->getOptions().getLoRaSF());
                                dualMetricRoutingTable[routeIndex].secMetric = thisRoute.getSecMetric() + packet->getOptions().getLoRaSF() - 7;
                                dualMetricRoutingTable[routeIndex].valid = simTime() + routeTimeout;
                            }
                        }
                    }
                }

//                EV << "Routing table size: " << end(dualMetricRoutingTable) - begin(dualMetricRoutingTable) << endl;
                break;

            default:
                break;
        }
        routingTableSize.collect(singleMetricRoutingTable.size());

        // Log snapshot after processing routing packet
        logRoutingSnapshot("routing_packet_processed");
    }

    EV << "## Routing table at node " << nodeId << "##" << endl;
    for (int i=0; i<singleMetricRoutingTable.size(); i++) {
        EV << "Node " << singleMetricRoutingTable[i].id << " via " << singleMetricRoutingTable[i].via << " with cost " << singleMetricRoutingTable[i].metric << endl;
    }
}

// Helper: keep only the best route per destination (single-metric tables)
// Policy: lower metric is better; if equal, keep the one with latest validity time; if still equal, prefer existing.
void LoRaNodeApp::addOrReplaceBestSingleRoute(const LoRaNodeApp::singleMetricRoute &candidate) {
    LoRaNodeApp::singleMetricRoute cand = candidate;
    int bestIdx = -1;
    for (int i = 0; i < (int)singleMetricRoutingTable.size(); ++i) {
        if (singleMetricRoutingTable[i].id == candidate.id) {
            if (bestIdx == -1) bestIdx = i; else {
                const auto &cur = singleMetricRoutingTable[i];
                const auto &best = singleMetricRoutingTable[bestIdx];
                if (cur.metric < best.metric || (cur.metric == best.metric && cur.valid > best.valid))
                    bestIdx = i;
            }
        }
    }
    bool candidateIsBest = true;
    if (bestIdx != -1) {
        const auto &best = singleMetricRoutingTable[bestIdx];
        if (best.metric < cand.metric || (best.metric == cand.metric && best.valid >= cand.valid))
            candidateIsBest = false;
    }
    if (candidateIsBest) {
        for (auto it = singleMetricRoutingTable.begin(); it != singleMetricRoutingTable.end(); ) {
            if (it->id == cand.id) it = singleMetricRoutingTable.erase(it); else ++it;
        }
        singleMetricRoutingTable.push_back(cand);
    } else {
        if (bestIdx == -1) singleMetricRoutingTable.push_back(cand);
    }
}
void LoRaNodeApp::openRoutingCsv() {
    // Build folder and file name: simulations folder is the working dir; create "routing_tables" subfolder
#ifdef _WIN32
    const char sep = '\\';
#else
    const char sep = '/';
#endif
    std::string folder = std::string("routing_tables");
    // Try to create folder using C runtime; if it exists, ignore errors
#ifdef _WIN32
    _mkdir(folder.c_str());
#else
    mkdir(folder.c_str(), 0775);
#endif

    // File name includes nodeId
    std::stringstream ss;
    ss << folder << sep << "node_" << nodeId << "_routing.csv";
    routingCsvPath = ss.str();
    routingCsvReady = true;
}

void LoRaNodeApp::logRoutingSnapshot(const char *eventName) {
    if (!routingCsvReady) return;
    routingCsv.open(routingCsvPath, std::ios::out | std::ios::trunc);
    if (!routingCsv.is_open()) return;
    routingCsv << "simTime,event,nodeId,metricType,tableSize,id,via,metric,validUntil,sf,priMetric,secMetric" << std::endl;

    const char *metricName = nullptr;
    switch (routingMetric) {
        case NO_FORWARDING: metricName = "NO_FORWARDING"; break;
        case FLOODING_BROADCAST_SINGLE_SF: metricName = "FLOODING"; break;
        case SMART_BROADCAST_SINGLE_SF: metricName = "SMART_BROADCAST"; break;
        case HOP_COUNT_SINGLE_SF: metricName = "HOP_COUNT"; break;
        case RSSI_SUM_SINGLE_SF: metricName = "RSSI_SUM"; break;
        case RSSI_PROD_SINGLE_SF: metricName = "RSSI_PROD"; break;
        case ETX_SINGLE_SF: metricName = "ETX"; break;
        case TIME_ON_AIR_HC_CAD_SF: metricName = "TOA_HC"; break;
        case TIME_ON_AIR_SF_CAD_SF: metricName = "TOA"; break;
        default: metricName = "UNKNOWN"; break;
    }

    for (const auto &r : singleMetricRoutingTable) {
        routingCsv << simTime() << ',' << eventName << ',' << nodeId << ',' << metricName
                   << ',' << singleMetricRoutingTable.size()
                   << ',' << r.id << ',' << r.via << ',' << r.metric << ',' << r.valid
                   << ",,," << std::endl;
    }
    for (const auto &r : dualMetricRoutingTable) {
        routingCsv << simTime() << ',' << eventName << ',' << nodeId << ',' << metricName
                   << ',' << dualMetricRoutingTable.size()
                   << ',' << r.id << ',' << r.via << ",," << r.valid
                   << ',' << r.sf << ',' << r.priMetric << ',' << r.secMetric
                   << std::endl;
    }
    routingCsv.flush();
    routingCsv.close();
}

void LoRaNodeApp::manageReceivedPacketToForward(cMessage *msg) {
    receivedPacketsToForward++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);
    std::cout << "normal forwarding " << packet->getMsgType() << std::endl;
    switch (packet->getMsgType()) {
    // DATA packet
    case DATA:
        manageReceivedDataPacketToForward(packet);
        break;
    // ACK packet
    case ACK:
        manageReceivedAckPacketToForward(packet);
        break;
    default:
        break;
    }
}

void LoRaNodeApp::manageReceivedAckPacketToForward(cMessage *msg) {
    receivedAckPackets++;
    receivedAckPacketsToForward++;
}

void LoRaNodeApp::manageReceivedDataPacketToForward(cMessage *msg) {
    receivedDataPackets++;
    receivedDataPacketsToForward++;
    bool newPacketToForward = false;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);
    LoRaAppPacket *dataPacket = packet->dup();

    // Check for too old packets with TTL <= 1
    if (packet->getTtl() <= 1) {
        bubble("This packet has reached TTL expiration!");
        receivedDataPacketsToForwardExpired++;
    }

    // Packet has not reached its maximum TTL
    else {
        receivedDataPacketsToForwardCorrect++;

        switch (routingMetric) {
            case NO_FORWARDING:
                bubble("Discarding packet as forwarding is disabled");
                break;

            case FLOODING_BROADCAST_SINGLE_SF:
            case SMART_BROADCAST_SINGLE_SF:
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
            case TIME_ON_AIR_HC_CAD_SF:
            case TIME_ON_AIR_SF_CAD_SF:
            default:
                // Check if the packet has already been forwarded
                if (isPacketForwarded(packet)) {
                    bubble("This packet has already been forwarded!");
                    forwardPacketsDuplicateAvoid++;
                }
                // Check if the packet is buffered to be forwarded
                else if (isPacketToBeForwarded(packet)) {
                    bubble("This packet is already scheduled to be forwarded!");
                    forwardPacketsDuplicateAvoid++;
                // A previously-unknown packet has arrived
                } else {
                    bubble("Saving packet to forward it later!");
                    receivedDataPacketsToForwardUnique++;

                    dataPacket->setTtl(packet->getTtl() - 1);
                    if (packetsToForwardMaxVectorSize == 0 || LoRaPacketsToForward.size()<packetsToForwardMaxVectorSize) {
                        LoRaPacketsToForward.push_back(*dataPacket);
                        newPacketToForward = true;
                    }
                    else {
                        forwardBufferFull++;
                    }
                }
        }

    }

    delete dataPacket;

    if (newPacketToForward) {
        forwardPacketsDue = true;

        if (!selfPacket->isScheduled()) {
            simtime_t nextScheduleTime = simTime() + 10*simTimeResolution;

            if (enforceDutyCycle) {
                nextScheduleTime = math::max(nextScheduleTime.dbl(), dutyCycleEnd.dbl());
            }

            if (! (nextScheduleTime > simTime()) ) {
                nextScheduleTime = simTime() + 1;
            }

            scheduleAt(nextScheduleTime, selfPacket);
            forwardPacketsDue = true;
        }
    }
}

void LoRaNodeApp::manageReceivedPacketForMe(cMessage *msg) {
    receivedPacketsForMe++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    switch (packet->getMsgType()) {
    // DATA packet
    case DATA:
        //manageReceivedDataPacketForMe(packet);
        std::cout << " forwarding even I am the destination " << packet->getMsgType() << std::endl;
        manageReceivedDataPacketToForward(packet);
        break;
        // ACK packet
    case ACK:
        manageReceivedAckPacketForMe(packet);
        break;
        // Other type
    default:
        break;
    }
}

void LoRaNodeApp::manageReceivedDataPacketForMe(cMessage *msg) {
    receivedDataPackets++;
    receivedDataPacketsForMe++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);
    dataPacketsForMeLatency.collect(simTime()-packet->getDepartureTime());

    if (isDataPacketForMeUnique(packet)) {
        DataPacketsForMe.push_back(*packet);
        receivedDataPacketsForMeUnique++;
        dataPacketsForMeUniqueLatency.collect(simTime()-packet->getDepartureTime());
    }
}

void LoRaNodeApp::manageReceivedAckPacketForMe(cMessage *msg) {
    receivedAckPackets++;
    receivedAckPacketsForMe++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // Optional: do something with the packet
}

bool LoRaNodeApp::handleOperationStage(LifecycleOperation *operation, int stage,
        IDoneCallback *doneCallback) {
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'",
            operation->getClassName());
    return true;
}


simtime_t LoRaNodeApp::sendDataPacket() {
    LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");
    std::cout << " i am sending the packet: "  << std::endl;
    bool localData = true;
    bool transmit = false;
    simtime_t txDuration = 0;

    // Send local data packets with a configurable ownDataPriority priority over packets to forward, if there is any
    if (
            (LoRaPacketsToSend.size() > 0 && bernoulli(ownDataPriority))
            || (LoRaPacketsToSend.size() > 0 && LoRaPacketsToForward.size() == 0)) {

        if (useAODV && routeDiscovery) {
            sanitizeRoutingTable();
            int destCheck = LoRaPacketsToSend.front().getDestination();
            int routeIndexCheck = getBestRouteIndexTo(destCheck);
            if (routeIndexCheck < 0) {
                maybeStartDiscoveryFor(destCheck);
                delete dataPacket;
                return 0;
            }
        }

        bubble("Sending a local data packet!");

        // Name packets to ease tracking
        std::string fullName = dataPacket->getName();;
        const char* addName = "Orig";
        fullName += addName;
        fullName += std::to_string(nodeId);


        // Get the data from the first packet in the data buffer to send it
        dataPacket->setMsgType(LoRaPacketsToSend.front().getMsgType());
        dataPacket->setDataInt(LoRaPacketsToSend.front().getDataInt());
        dataPacket->setSource(LoRaPacketsToSend.front().getSource());
        dataPacket->setVia(LoRaPacketsToSend.front().getSource());
        dataPacket->setDestination(LoRaPacketsToSend.front().getDestination());
        dataPacket->setTtl(LoRaPacketsToSend.front().getTtl());
        dataPacket->getOptions().setAppACKReq(LoRaPacketsToSend.front().getOptions().getAppACKReq());
        dataPacket->setByteLength(LoRaPacketsToSend.front().getByteLength());
        dataPacket->setDepartureTime(simTime());

        addName = "Dest";
        fullName += addName;
        fullName += std::to_string(dataPacket->getDestination());
        dataPacket->setName(fullName.c_str());

        LoRaPacketsToSend.erase(LoRaPacketsToSend.begin());

        transmit = true;

        sentDataPackets++;
        if (firstDataPacketTransmissionTime == 0)
            firstDataPacketTransmissionTime = simTime();
        lastDataPacketTransmissionTime = simTime();
    }

    // Forward other nodes' packets, if any
    else if (LoRaPacketsToForward.size() > 0) {

        bubble("Forwarding a packet!");
        localData = false;

        std::string fullName = dataPacket->getName();;
        const char* addName = "Fwd";
        fullName += addName;
        dataPacket->setName(fullName.c_str());
        fullName += std::to_string(nodeId);

        switch (routingMetric) {
            case NO_FORWARDING:
                // This should never happen
                bubble("Forwarding disabled!");
                break;

            case FLOODING_BROADCAST_SINGLE_SF:
            case SMART_BROADCAST_SINGLE_SF:
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
            case TIME_ON_AIR_HC_CAD_SF:
            case TIME_ON_AIR_SF_CAD_SF:
            default:
                while (LoRaPacketsToForward.size() > 0) {
                    addName = "FWD-";
                    fullName += addName;
                    fullName += std::to_string(routingMetric);
                    addName = "-";
                    fullName += addName;
                    dataPacket->setName(fullName.c_str());

                    // Get the data from the first packet in the forwarding buffer to send it
                    dataPacket->setMsgType(LoRaPacketsToForward.front().getMsgType());
                    dataPacket->setDataInt(LoRaPacketsToForward.front().getDataInt());
                    dataPacket->setSource(LoRaPacketsToForward.front().getSource());
                    dataPacket->setVia(LoRaPacketsToForward.front().getSource());
                    dataPacket->setDestination(LoRaPacketsToForward.front().getDestination());
                    dataPacket->setTtl(LoRaPacketsToForward.front().getTtl());
                    dataPacket->getOptions().setAppACKReq(LoRaPacketsToForward.front().getOptions().getAppACKReq());
                    dataPacket->setByteLength(LoRaPacketsToForward.front().getByteLength());
                    dataPacket->setDepartureTime(LoRaPacketsToForward.front().getDepartureTime());

                    // Erase the first packet in the forwarding buffer
                    LoRaPacketsToForward.erase(LoRaPacketsToForward.begin());

                    // Redundantly check that the packet has not been forwarded in the mean time, which should never occur
                    if (!isPacketForwarded(dataPacket)) {
                        bubble("Forwarding packet!");
                        forwardedPackets++;
                        forwardedDataPackets++;
                        transmit = true;

                        // Keep a copy of the forwarded packet to avoid sending it again if received later on
                        LoRaPacketsForwarded.push_back(*dataPacket);
                        if (LoRaPacketsForwarded.size() > forwardedPacketVectorSize){
                            LoRaPacketsForwarded.erase(LoRaPacketsForwarded.begin());
                        }
                        break;
                    }
                }
                break;
        }
    }

    if (transmit) {
        sentPackets++;

        std::string fullName = dataPacket->getName();;
        const char* ownName = "Tx";
        fullName += ownName;
        dataPacket->setName(fullName.c_str());

        //add LoRa control info
        LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
        cInfo->setLoRaTP(loRaTP);
        cInfo->setLoRaCF(loRaCF);
        cInfo->setLoRaSF(loRaSF);
        cInfo->setLoRaBW(loRaBW);
        cInfo->setLoRaCR(loRaCR);

        sanitizeRoutingTable();

        int routeIndex = getBestRouteIndexTo(dataPacket->getDestination());

        switch (routingMetric) {
            case FLOODING_BROADCAST_SINGLE_SF:
                dataPacket->setVia(BROADCAST_ADDRESS);
                if (localData)
                    broadcastDataPackets++;
                else
                    broadcastForwardedPackets++;
                break;
            case SMART_BROADCAST_SINGLE_SF:
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
                if ( routeIndex >= 0 ) {
                    dataPacket->setVia(singleMetricRoutingTable[routeIndex].via);
                }
                else{
                    dataPacket->setVia(BROADCAST_ADDRESS);
                    if (localData)
                        broadcastDataPackets++;
                    else
                        broadcastForwardedPackets++;
                }
                break;
            case TIME_ON_AIR_HC_CAD_SF:
            case TIME_ON_AIR_SF_CAD_SF:
                if ( routeIndex >= 0 ) {
                    dataPacket->setVia(dualMetricRoutingTable[routeIndex].via);
                    cInfo->setLoRaSF(dualMetricRoutingTable[routeIndex].sf);
                }
                else{
                    dataPacket->setVia(BROADCAST_ADDRESS);
                    if (localData)
                        broadcastDataPackets++;
                    else
                        broadcastForwardedPackets++;
                }
                break;
        }

        dataPacket->setControlInfo(cInfo);

        txDuration = calculateTransmissionDuration(dataPacket);

        allTxPacketsSFStats.collect(loRaSF);
        if (localData) {
            owndataTxPacketsSFStats.collect(loRaSF);
        }
        else {
            fwdTxPacketsSFStats.collect(loRaSF);
        }


        send(dataPacket, "appOut");
        txSfVector.record(loRaSF);
        txTpVector.record(loRaTP);
        //rxRssiVector.record(loRaTP);

        emit(LoRa_AppPacketSent, loRaSF);
    }
    else {
        delete dataPacket;
    }

    // Generate more packets if needed
    if (sendPacketsContinuously && LoRaPacketsToSend.size() == 0) {
        generateDataPackets();
    }

    return txDuration;
}

simtime_t LoRaNodeApp::sendForwardPacket() {
    LoRaAppPacket *forwardPacket = new LoRaAppPacket("DataFrame");
    std::cout << " im here forwarding the new packet: "  << std::endl;
    bool transmit = false;
    simtime_t txDuration = 0;

    if (LoRaPacketsToForward.size() > 0) {

        bubble("Forwarding a packet!");

        std::string fullName = forwardPacket->getName();;
        const char* addName = "Fwd";
        fullName += addName;
        forwardPacket->setName(fullName.c_str());
        fullName += std::to_string(nodeId);

        switch (routingMetric) {
            case NO_FORWARDING:
                // This should never happen
                bubble("Forwarding disabled!");
                break;

            case FLOODING_BROADCAST_SINGLE_SF:
            case SMART_BROADCAST_SINGLE_SF:
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
            case TIME_ON_AIR_HC_CAD_SF:
            case TIME_ON_AIR_SF_CAD_SF:
            default:
                while (LoRaPacketsToForward.size() > 0) {
                    addName = "FWD-";
                    fullName += addName;
                    fullName += std::to_string(routingMetric);
                    addName = "-";
                    fullName += addName;
                    forwardPacket->setName(fullName.c_str());

                    // Get the data from the first packet in the forwarding buffer to send it
                    forwardPacket->setMsgType(LoRaPacketsToForward.front().getMsgType());
                    forwardPacket->setDataInt(LoRaPacketsToForward.front().getDataInt());
                    forwardPacket->setSource(LoRaPacketsToForward.front().getSource());
                    forwardPacket->setVia(LoRaPacketsToForward.front().getSource());
                    forwardPacket->setDestination(LoRaPacketsToForward.front().getDestination());
                    forwardPacket->setTtl(LoRaPacketsToForward.front().getTtl());
                    forwardPacket->getOptions().setAppACKReq(LoRaPacketsToForward.front().getOptions().getAppACKReq());
                    forwardPacket->setByteLength(LoRaPacketsToForward.front().getByteLength());
                    forwardPacket->setDepartureTime(LoRaPacketsToForward.front().getDepartureTime());

                    // Erase the first packet in the forwarding buffer
                    LoRaPacketsToForward.erase(LoRaPacketsToForward.begin());

                    // Redundantly check that the packet has not been forwarded in the mean time, which should never occur
                    if (!isPacketForwarded(forwardPacket)) {
                        bubble("Forwarding packet!");
                        forwardedPackets++;
                        forwardedDataPackets++;
                        transmit = true;

                        // Keep a copy of the forwarded packet to avoid sending it again if received later on
                        LoRaPacketsForwarded.push_back(*forwardPacket);
                        if (LoRaPacketsForwarded.size() > forwardedPacketVectorSize){
                            LoRaPacketsForwarded.erase(LoRaPacketsForwarded.begin());
                        }
                        break;
                    }
                }
                break;
        }
    }

    if (transmit) {
        sentPackets++;

        std::string fullName = forwardPacket->getName();;
        const char* ownName = "Tx";
        fullName += ownName;
        forwardPacket->setName(fullName.c_str());

        //add LoRa control info
        LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
        cInfo->setLoRaTP(loRaTP);
        cInfo->setLoRaCF(loRaCF);
        cInfo->setLoRaSF(loRaSF);
        cInfo->setLoRaBW(loRaBW);
        cInfo->setLoRaCR(loRaCR);

        sanitizeRoutingTable();

        int routeIndex = getBestRouteIndexTo(forwardPacket->getDestination());

        switch (routingMetric) {
            case FLOODING_BROADCAST_SINGLE_SF:
                forwardPacket->setVia(BROADCAST_ADDRESS);
                broadcastForwardedPackets++;
                break;
            case SMART_BROADCAST_SINGLE_SF:
            case HOP_COUNT_SINGLE_SF:
            case RSSI_SUM_SINGLE_SF:
            case RSSI_PROD_SINGLE_SF:
            case ETX_SINGLE_SF:
                if ( routeIndex >= 0 ) {
                    forwardPacket->setVia(singleMetricRoutingTable[routeIndex].via);
                }
                else{
                    forwardPacket->setVia(BROADCAST_ADDRESS);
                    broadcastForwardedPackets++;
                }
                break;
            case TIME_ON_AIR_HC_CAD_SF:
            case TIME_ON_AIR_SF_CAD_SF:
                if ( routeIndex >= 0 ) {
                    forwardPacket->setVia(dualMetricRoutingTable[routeIndex].via);
                    cInfo->setLoRaSF(dualMetricRoutingTable[routeIndex].sf);
                }
                else{
                    forwardPacket->setVia(BROADCAST_ADDRESS);
                    broadcastForwardedPackets++;
                }
                break;
        }

        forwardPacket->setControlInfo(cInfo);

        txDuration = calculateTransmissionDuration(forwardPacket);

        allTxPacketsSFStats.collect(loRaSF);
        fwdTxPacketsSFStats.collect(loRaSF);

        send(forwardPacket, "appOut");
        txSfVector.record(loRaSF);
        txTpVector.record(loRaTP);
        //rxRssiVector.record(loRaTP);
        emit(LoRa_AppPacketSent, loRaSF);
    }
    else {
        delete forwardPacket;
    }

    return txDuration;
}



simtime_t LoRaNodeApp::sendRoutingPacket() {
    // In AODV mode, legacy periodic ROUTING packets are suppressed
    if (useAODV) {
        return 0;
    }

    bool transmit = false;
    simtime_t txDuration = 0;
    int numberOfRoutes = 0;

    LoRaAppPacket *routingPacket = new LoRaAppPacket("RoutingPacket");

    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;

    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);

    sanitizeRoutingTable();

    std::vector<LoRaRoute> theseLoRaRoutes;
    int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);
    int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

    switch (routingMetric) {

        case NO_FORWARDING:
            break;

        case FLOODING_BROADCAST_SINGLE_SF:
        case SMART_BROADCAST_SINGLE_SF:
            break;

        case HOP_COUNT_SINGLE_SF:
        case RSSI_SUM_SINGLE_SF:
        case RSSI_PROD_SINGLE_SF:
        case ETX_SINGLE_SF:

            transmit = true;

            // Count the number of best routes
            for (int i=0; i<numberOfNodes; i++) {
                if (i != nodeId) {
                    if (getBestRouteIndexTo(i) >= 0) {
                        numberOfRoutes++;
                    }
                }
            }

            // Make room for numberOfRoutes routes
            routingPacket->setRoutingTableArraySize(numberOfRoutes);

            // Add the best route to each node
            for (int i=0; i<numberOfNodes; i++) {
                if (i != nodeId) {
                    if (getBestRouteIndexTo(i) >= 0) {

                        LoRaRoute thisLoRaRoute;
                        thisLoRaRoute.setId(singleMetricRoutingTable[getBestRouteIndexTo(i)].id);
                        thisLoRaRoute.setPriMetric(singleMetricRoutingTable[getBestRouteIndexTo(i)].metric);
                        routingPacket->setRoutingTable(numberOfRoutes-1, thisLoRaRoute);
                        numberOfRoutes--;
                    }
                }
            }

            break;

        case TIME_ON_AIR_HC_CAD_SF:
        case TIME_ON_AIR_SF_CAD_SF:
            transmit = true;

            loRaSF = pickCADSF();
            cInfo->setLoRaSF(loRaSF);

            std::vector<LoRaRoute> allLoRaRoutes;

            routingPacket->setRoutingTableArraySize(dualMetricRoutesCount);

           for (int i = 0; i < dualMetricRoutesCount; i++) {
               LoRaRoute thisLoRaRoute;
               thisLoRaRoute.setId(dualMetricRoutingTable[i].id);
               thisLoRaRoute.setPriMetric(dualMetricRoutingTable[i].priMetric);
               thisLoRaRoute.setSecMetric(dualMetricRoutingTable[i].secMetric);
               allLoRaRoutes.push_back(thisLoRaRoute);
               routingPacket->setRoutingTable(i, thisLoRaRoute);
           }

            break;
    }


    if (transmit) {
        sentPackets++;
        sentRoutingPackets++;
        //add LoRa control info


        routingPacket->setControlInfo(cInfo);

        routingPacket->setMsgType(ROUTING);
        routingPacket->setDataInt(sentRoutingPackets);
        routingPacket->setSource(nodeId);
        routingPacket->setVia(nodeId);
        routingPacket->setDestination(BROADCAST_ADDRESS);
        routingPacket->getOptions().setAppACKReq(false);
        routingPacket->setByteLength(routingPacketMaxSize);
        routingPacket->setDepartureTime(simTime());

        txSfVector.record(loRaSF);
        txTpVector.record(loRaTP);
        //rxRssiVector.record(loRaTP);

        txDuration = calculateTransmissionDuration(routingPacket);

        allTxPacketsSFStats.collect(loRaSF);
        routingTxPacketsSFStats.collect(loRaSF);

        send(routingPacket, "appOut");
        bubble("Sending routing packet");
        emit(LoRa_AppPacketSent, loRaSF);
    }
    else {
        delete routingPacket;
    }
    return txDuration;
}

void LoRaNodeApp::generateDataPackets() {

    if ((selectedTxNodeId >= 0 && nodeId == selectedTxNodeId) ||
        (selectedTxNodeId < 0 && (!onlyNode0SendsPackets || nodeId == 0))) {
        std::vector<int> destinations = { };

        if (selectedRxNodeId >= 0) {
            destinations.push_back(selectedRxNodeId);
        } else {
            if (numberOfDestinationsPerNode == 0 )
                numberOfDestinationsPerNode = numberOfNodes-1;

            while (destinations.size() < (size_t)numberOfDestinationsPerNode
                    && numberOfNodes - 1 - (int)destinations.size() > 0) {

                int destination = intuniform(0, numberOfNodes - 1);

                if (destination != nodeId) {
                    bool newDestination = true;

                    for (int i = 0; i < (int)destinations.size(); i++) {
                        if (destination == destinations[i]) {
                            newDestination = false;
                            break;
                        }
                    }

                    if (newDestination) {
                        destinations.push_back(destination);
                    }
                }
            }
        }

        for (int k = 0; k < numberOfPacketsPerDestination; k++) {
            for (int j = 0; j < destinations.size(); j++) {
                LoRaAppPacket *dataPacket = new LoRaAppPacket("DataPacket");

                dataPacket->setMsgType(DATA);
                dataPacket->setDataInt(currDataInt+k);
                dataPacket->setSource(nodeId);
                dataPacket->setVia(nodeId);
                dataPacket->setDestination(destinations[j]);
                dataPacket->getOptions().setAppACKReq(requestACKfromApp);
                dataPacket->setByteLength(dataPacketSize);
                dataPacket->setDepartureTime(simTime());

                switch (routingMetric) {
    //            case 0:
    //                dataPacket->setTtl(1);
    //                break;
                default:
                    dataPacket->setTtl(packetTTL);
                    break;
                }

                LoRaPacketsToSend.push_back(*dataPacket);
                delete dataPacket;
            }
            currDataInt++;
        }
    }
}

void LoRaNodeApp::increaseSFIfPossible() {
    if (loRaSF < 12) {
        // char text[32];
        // sprintf(text, "Increasing SF from %d to %d", loRaSF, loRaSF+1);
        // bubble(text);
        loRaSF++;
    }
}

bool LoRaNodeApp::isNeighbour(int neighbourId) {
    for (std::vector<int>::iterator nbptr = neighbourNodes.begin();
            nbptr < neighbourNodes.end(); nbptr++) {
        if (neighbourId == *nbptr) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isRouteInSingleMetricRoutingTable(int id, int via) {
    if (getRouteIndexInSingleMetricRoutingTable(id, via) >= 0) {
        return true;
    }
    return false;
}

int LoRaNodeApp::getRouteIndexInSingleMetricRoutingTable(int id, int via) {
    int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);

    for (int i = 0; i < singleMetricRoutesCount; i++) {
        if (singleMetricRoutingTable[i].id == id && singleMetricRoutingTable[i].via == via) {
            return i;
        }
    }

    return -1;
}

bool LoRaNodeApp::isRouteInDualMetricRoutingTable(int id, int via, int sf) {
    if (getRouteIndexInDualMetricRoutingTable(id, via, sf) >= 0) {
        return true;
    }
    return false;
}

int LoRaNodeApp::getRouteIndexInDualMetricRoutingTable(int id, int via, int sf) {
    int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

    for (int i = 0; i < dualMetricRoutesCount; i++) {
        if (dualMetricRoutingTable[i].id == id && dualMetricRoutingTable[i].via == via && dualMetricRoutingTable[i].sf == sf) {
            return i;
        }
    }

    return -1;
}


bool LoRaNodeApp::isKnownNode(int knownNodeId) {
    for (std::vector<int>::iterator nbptr = knownNodes.begin();
            nbptr < knownNodes.end(); nbptr++) {
        if (knownNodeId == *nbptr) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isACKed(int nodeId) {
    for (std::vector<int>::iterator nbptr = ACKedNodes.begin();
            nbptr < ACKedNodes.end(); nbptr++) {
        if (nodeId == *nbptr) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isPacketForwarded(cMessage *msg) {
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsForwarded.begin(); lbptr < LoRaPacketsForwarded.end();
            lbptr++) {
        if (packet->getMsgType() == lbptr->getMsgType()
                && packet->getDataInt() == lbptr->getDataInt()
                && packet->getSource() == lbptr->getSource()
                && packet->getDestination() == lbptr->getDestination()) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isPacketToBeForwarded(cMessage *msg) {
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            LoRaPacketsToForward.begin(); lbptr < LoRaPacketsToForward.end();
            lbptr++) {
        if (packet->getMsgType() == lbptr->getMsgType()
                && packet->getDataInt() == lbptr->getDataInt()
                && packet->getSource() == lbptr->getSource()
                && packet->getDestination() == lbptr->getDestination()) {
            return true;
        }
    }
    return false;
}

bool LoRaNodeApp::isDataPacketForMeUnique(cMessage *msg) {
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    for (std::vector<LoRaAppPacket>::iterator lbptr =
            DataPacketsForMe.begin(); lbptr < DataPacketsForMe.end();
            lbptr++) {
        if (packet->getMsgType() == lbptr->getMsgType()
                && packet->getDataInt() == lbptr->getDataInt()
                && packet->getSource() == lbptr->getSource()
                && packet->getDestination() == lbptr->getDestination()) {
            return false;
        }
    }
    return true;
}

int LoRaNodeApp::pickCADSF() {
    do {
        int thisSF = intuniform(minLoRaSF,maxLoRaSF);
        if (bernoulli(pow(0.5, thisSF-minLoRaSF+1)))
            return thisSF;
    } while (true);
}

int LoRaNodeApp::getBestRouteIndexTo(int destination) {
    if (singleMetricRoutingTable.size() > 0) {

        int singleMetricRoutesCount = end(singleMetricRoutingTable) - begin(singleMetricRoutingTable);

        std::vector<singleMetricRoute> availableRoutes;

        for (int i = 0; i < singleMetricRoutesCount; i++) {
            if (singleMetricRoutingTable[i].id == destination) {
                availableRoutes.push_back(singleMetricRoutingTable[i]);

            }
        }

        if (availableRoutes.size() > 0) {
            int bestRoute = 0;
            int bestMetric = availableRoutes[0].metric;

            int availableRoutesCount = end(availableRoutes) - begin(availableRoutes);
            for (int j = 0; j < availableRoutesCount; j++) {
                if (availableRoutes[j].metric < bestMetric) {
                    bestMetric = availableRoutes[j].metric;
                }
            }

            simtime_t lastMetric = 0;

            for (int k = 0; k < availableRoutesCount; k++) {
                if (availableRoutes[k].metric == bestMetric) {
                    if (availableRoutes[k].valid >= lastMetric) {
                        bestRoute = k;
                        lastMetric = availableRoutes[k].valid;
                    }
                }
            }
            return getRouteIndexInSingleMetricRoutingTable(availableRoutes[bestRoute].id, availableRoutes[bestRoute].via);
        }
    }
    else if (dualMetricRoutingTable.size() > 0) {
        int dualMetricRoutesCount = dualMetricRoutingTable.size();

        std::vector<dualMetricRoute> availableRoutes;

        for (int i = 0; i < dualMetricRoutesCount; i++) {
            if (dualMetricRoutingTable[i].id == destination) {
                availableRoutes.push_back(dualMetricRoutingTable[i]);
            }
        }

        if (availableRoutes.size() > 0) {
            int bestRoute = 0;

            int availableRoutesCount = end(availableRoutes) - begin(availableRoutes);

            for (int j = 0; j < availableRoutesCount; j++) {
                if (availableRoutes[j].priMetric < availableRoutes[bestRoute].priMetric ||
                        ( availableRoutes[j].priMetric == availableRoutes[bestRoute].priMetric &&
                                availableRoutes[j].secMetric < availableRoutes[bestRoute].secMetric) ||
                                ( availableRoutes[j].priMetric == availableRoutes[bestRoute].priMetric &&
                                        availableRoutes[j].secMetric == availableRoutes[bestRoute].secMetric &&
                                            availableRoutes[j].valid > availableRoutes[bestRoute].valid)) {
                    bestRoute = j;
                }
            }
            return getRouteIndexInDualMetricRoutingTable(availableRoutes[bestRoute].id, availableRoutes[bestRoute].via, availableRoutes[bestRoute].sf);
        }
    }

    return -1;
}

void LoRaNodeApp::sanitizeRoutingTable() {
    bool routeDeleted = false;

    if (singleMetricRoutingTable.size() > 0) {

        do {
            routeDeleted = false;

            for (std::vector<singleMetricRoute>::iterator smr =
                    singleMetricRoutingTable.begin(); smr < singleMetricRoutingTable.end();
                    smr++) {
                if (smr->valid < simTime()) {
                    singleMetricRoutingTable.erase(smr);
                    routeDeleted = true;
                    deletedRoutes++;
                    break;
                }
            }
        } while (routeDeleted);
    }
    else if (dualMetricRoutingTable.size() > 0) {

        do {
            routeDeleted = false;

            for (std::vector<dualMetricRoute>::iterator dmr =
                    dualMetricRoutingTable.begin(); dmr < dualMetricRoutingTable.end();
                    dmr++) {
                if (dmr->valid < simTime()) {
                    dualMetricRoutingTable.erase(dmr);
                    routeDeleted = true;
                    deletedRoutes++;
                    break;
                }
            }
        } while (routeDeleted);
    }
}

int LoRaNodeApp::getSFTo(int destination) {
    if (dualMetricRoutingTable.size() > 0) {
        int dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

        std::vector<dualMetricRoute> availableRoutes;

        for (int i = 0; i < dualMetricRoutesCount; i++) {
            if (dualMetricRoutingTable[i].id == destination) {
                availableRoutes.push_back(dualMetricRoutingTable[i]);
            }
        }

        dualMetricRoutesCount = end(dualMetricRoutingTable) - begin(dualMetricRoutingTable);

        if (availableRoutes.size() > 0) {
            int bestRoute = 0;

            int availableRoutesCount = end(availableRoutes) - begin(availableRoutes);

            for (int j = 0; j < availableRoutesCount; j++) {
                if (availableRoutes[j].sf < availableRoutes[bestRoute].sf) {
                    bestRoute = j;
                }
            }

            if ( availableRoutes[bestRoute].sf >= minLoRaSF && availableRoutes[bestRoute].sf <= maxLoRaSF) {
                return availableRoutes[bestRoute].sf;
            }
        }
    }

    return minLoRaSF;
}


simtime_t LoRaNodeApp::calculateTransmissionDuration(cMessage *msg) {

    TransmissionRequest *controlInfo = dynamic_cast<TransmissionRequest *>(msg->getControlInfo());

    const LoRaAppPacket *frame = check_and_cast<const LoRaAppPacket *>(msg);
    const LoRaMacControlInfo *cInfo = check_and_cast<const LoRaMacControlInfo *>(frame->getControlInfo());

    int nPreamble = 8;
    simtime_t Tsym = (pow(2, cInfo->getLoRaSF()))/(cInfo->getLoRaBW().get()/1000);
    simtime_t Tpreamble = (nPreamble + 4.25) * Tsym / 1000;

    int payloadBytes = frame->getByteLength()+8; //+8 bytes for headers

    int payloadSymbNb = 8 + math::max(ceil((8*payloadBytes - 4*cInfo->getLoRaSF() + 28 + 16 - 20*0)/(4*(cInfo->getLoRaSF()-2*0)))*(cInfo->getLoRaCR() + 4), 0);

    simtime_t Theader = 0.5 * (8+payloadSymbNb) * Tsym / 1000;
    simtime_t Tpayload = 0.5 * (8+payloadSymbNb) * Tsym / 1000;

    const simtime_t duration = Tpreamble + Theader + Tpayload;
    return duration;
}

simtime_t LoRaNodeApp::getTimeToNextRoutingPacket() {
    if ( strcmp(par("timeToNextRoutingPacketDist").stringValue(), "uniform") == 0) {
        simtime_t routingTime = uniform(timeToNextRoutingPacketMin, timeToNextRoutingPacketMax);
        return routingTime;
    }
    else if ( strcmp(par("timeToNextRoutingPacketDist").stringValue(), "exponential") == 0) {
        simtime_t routingTime = exponential(timeToNextRoutingPacketAvg);
        return routingTime;
    }
    return simTime();
}

simtime_t LoRaNodeApp::getTimeToNextDataPacket() {
    if ( strcmp(par("timeToNextDataPacketDist").stringValue(), "uniform") == 0) {
        simtime_t DataTime = uniform(timeToNextDataPacketMin, timeToNextDataPacketMax);
        return DataTime;
    }
    else if ( strcmp(par("timeToNextDataPacketDist").stringValue(), "exponential") == 0) {
        simtime_t DataTime = exponential(timeToNextDataPacketAvg);
        return DataTime;
    }
    return simTime();
}

simtime_t LoRaNodeApp::getTimeToNextForwardPacket() {
    if ( strcmp(par("timeToNextForwardPacketDist").stringValue(), "uniform") == 0) {
        simtime_t ForwardTime = uniform(timeToNextForwardPacketMin, timeToNextForwardPacketMax);
        return ForwardTime;
    }
    else if ( strcmp(par("timeToNextForwardPacketDist").stringValue(), "exponential") == 0) {
        simtime_t DataTime = exponential(timeToNextDataPacketAvg);
        return DataTime;
    }
    return simTime();
}



// ---- AODV-lite helpers ----
void LoRaNodeApp::maybeStartDiscoveryFor(int destId) {
    sendRreq(destId);
    if (!selfPacket->isScheduled()) scheduleAt(simTime() + 1, selfPacket);
}

void LoRaNodeApp::sendRreq(int destId) {
    LoRaAppPacket *rreq = new LoRaAppPacket("RREQ");
    int rreqId = ++rreqSeq;
    long long key = ((long long)nodeId << 32) | (unsigned int)rreqId;
    seenRreqs.insert(key);
    rreq->setMsgType(RREQ);
    rreq->setDataInt(rreqId);
    rreq->setSource(nodeId);
    rreq->setDestination(destId);
    rreq->setVia(BROADCAST_ADDRESS);
    rreq->setLastHop(nodeId);
    int aodvTtl = hasPar("aodvTtl") ? (int)par("aodvTtl") : 0;
    rreq->setTtl(aodvTtl > 0 ? aodvTtl : std::max(2, packetTTL));
    rreq->getOptions().setAppACKReq(false);
    rreq->setByteLength(8);
    rreq->setDepartureTime(simTime());
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);
    rreq->setControlInfo(cInfo);
    EV_INFO << "AODV RREQ originate: src=" << nodeId
            << " dest=" << destId
            << " id=" << rreqId
            << " ttl=" << rreq->getTtl() << endl;
    std::cout << "AODV RREQ originate src=" << nodeId
              << " dest=" << destId
              << " id=" << rreqId
              << " ttl=" << rreq->getTtl() << std::endl;
    enqueueAodvOrSend(rreq);
}

void LoRaNodeApp::handleRreq(cMessage *msg) {
    LoRaAppPacket *pkt = check_and_cast<LoRaAppPacket *>(msg);
    EV_INFO << "AODV RREQ rx at node=" << nodeId
            << " src=" << pkt->getSource()
            << " dest=" << pkt->getDestination()
            << " id=" << pkt->getDataInt()
            << " ttl=" << pkt->getTtl()
            << " lasthop=" << pkt->getLastHop() << endl;
    std::cout << "AODV RREQ rx node=" << nodeId
              << " src=" << pkt->getSource()
              << " dest=" << pkt->getDestination()
              << " id=" << pkt->getDataInt()
              << " ttl=" << pkt->getTtl()
              << " lh=" << pkt->getLastHop() << std::endl;
    long long key = ((long long)pkt->getSource() << 32) | (unsigned int)pkt->getDataInt();
    if (seenRreqs.find(key) != seenRreqs.end()) return;
    seenRreqs.insert(key);

    // Reverse route to origin via the neighbor we heard this packet from
    singleMetricRoute rev{};
    rev.id = pkt->getSource();
    rev.via = pkt->getLastHop();
    rev.metric = 1;
    rev.valid = simTime() + routeTimeout;
    if (storeBestRoutesOnly) addOrReplaceBestSingleRoute(rev); else singleMetricRoutingTable.push_back(rev);
    EV_INFO << "AODV RREQ install reverse route: dest(origin)=" << rev.id << " via=" << rev.via << endl;

    if (pkt->getDestination() == nodeId) {
        int nextHopIndex = getBestRouteIndexTo(pkt->getSource());
        int viaHop = (nextHopIndex >= 0) ? singleMetricRoutingTable[nextHopIndex].via : pkt->getLastHop();
        EV_INFO << "AODV RREQ reached destination node=" << nodeId << ", sending RREP via=" << viaHop << endl;
        std::cout << "AODV RREQ reached dest node=" << nodeId << ", RREP via=" << viaHop << std::endl;
        sendRrep(pkt->getSource(), viaHop);
        return;
    }

    if (pkt->getTtl() > 1) {
        LoRaAppPacket *fwd = new LoRaAppPacket("RREQ");
        fwd->setMsgType(RREQ);
        fwd->setDataInt(pkt->getDataInt());
        fwd->setSource(pkt->getSource());
        fwd->setDestination(pkt->getDestination());
        fwd->setVia(BROADCAST_ADDRESS);
        fwd->setLastHop(nodeId);
        fwd->setTtl(pkt->getTtl() - 1);
        fwd->getOptions().setAppACKReq(false);
        fwd->setByteLength(8);
        fwd->setDepartureTime(simTime());
        LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
        cInfo->setLoRaTP(loRaTP);
        cInfo->setLoRaCF(loRaCF);
        cInfo->setLoRaSF(loRaSF);
        cInfo->setLoRaBW(loRaBW);
        cInfo->setLoRaCR(loRaCR);
    fwd->setControlInfo(cInfo);
        EV_INFO << "AODV RREQ forward at node=" << nodeId
                << " to broadcast, ttl now=" << fwd->getTtl() << endl;
        std::cout << "AODV RREQ fwd node=" << nodeId
                  << " ttl=" << fwd->getTtl() << std::endl;
    enqueueAodvOrSend(fwd);
    }
}

void LoRaNodeApp::sendRrep(int originId, int viaNextHop) {
    LoRaAppPacket *rrep = new LoRaAppPacket("RREP");
    rrep->setMsgType(RREP);
    rrep->setSource(nodeId);
    rrep->setDestination(originId);
    rrep->setVia(viaNextHop);
    rrep->setLastHop(nodeId);
    int aodvTtl = hasPar("aodvTtl") ? (int)par("aodvTtl") : 0;
    rrep->setTtl(aodvTtl > 0 ? aodvTtl : std::max(2, packetTTL));
    rrep->getOptions().setAppACKReq(false);
    rrep->setByteLength(8);
    rrep->setDepartureTime(simTime());
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);
    rrep->setControlInfo(cInfo);
    EV_INFO << "AODV RREP send from node=" << nodeId
            << " to origin=" << originId
            << " via=" << viaNextHop
            << " ttl=" << rrep->getTtl() << endl;
    std::cout << "AODV RREP send node=" << nodeId
              << " to=" << originId
              << " via=" << viaNextHop
              << " ttl=" << rrep->getTtl() << std::endl;
    enqueueAodvOrSend(rrep);
}

void LoRaNodeApp::handleRrep(cMessage *msg) {
    LoRaAppPacket *pkt = check_and_cast<LoRaAppPacket *>(msg);
    EV_INFO << "AODV RREP rx at node=" << nodeId
            << " src(dest)=" << pkt->getSource()
            << " dest(origin)=" << pkt->getDestination()
            << " ttl=" << pkt->getTtl()
            << " lasthop=" << pkt->getLastHop() << endl;
    std::cout << "AODV RREP rx node=" << nodeId
              << " src=" << pkt->getSource()
              << " dest=" << pkt->getDestination()
              << " ttl=" << pkt->getTtl()
              << " lh=" << pkt->getLastHop() << std::endl;
    // Forward route to final destination (pkt->getSource()) via lastHop
    singleMetricRoute fwd{};
    fwd.id = pkt->getSource();
    fwd.via = pkt->getLastHop();
    fwd.metric = 1;
    fwd.valid = simTime() + routeTimeout;
    if (storeBestRoutesOnly) addOrReplaceBestSingleRoute(fwd); else singleMetricRoutingTable.push_back(fwd);

    if (pkt->getDestination() == nodeId) {
        EV_INFO << "AODV RREP reached origin at node=" << nodeId << ". Scheduling data send." << endl;
        if (!selfPacket->isScheduled()) scheduleAt(simTime() + 0.5, selfPacket);
        return;
    }

    if (pkt->getTtl() > 1) {
        int nextHopIndex = getBestRouteIndexTo(pkt->getDestination());
        int viaHop = (nextHopIndex >= 0) ? singleMetricRoutingTable[nextHopIndex].via : pkt->getLastHop();
        if (viaHop == nodeId) {
            EV_INFO << "AODV RREP drop at node=" << nodeId << ": next hop resolves to self" << endl;
            std::cout << "AODV RREP drop node=" << nodeId << " via=self" << std::endl;
            return;
        }

        LoRaAppPacket *fwdR = new LoRaAppPacket("RREP");
        fwdR->setMsgType(RREP);
        fwdR->setSource(pkt->getSource());
        fwdR->setDestination(pkt->getDestination());
        fwdR->setVia(viaHop);
        fwdR->setLastHop(nodeId);
        fwdR->setTtl(pkt->getTtl() - 1);
        fwdR->getOptions().setAppACKReq(false);
        fwdR->setByteLength(8);
        fwdR->setDepartureTime(simTime());
        LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
        cInfo->setLoRaTP(loRaTP);
        cInfo->setLoRaCF(loRaCF);
        cInfo->setLoRaSF(loRaSF);
        cInfo->setLoRaBW(loRaBW);
        cInfo->setLoRaCR(loRaCR);
        fwdR->setControlInfo(cInfo);
        EV_INFO << "AODV RREP forward at node=" << nodeId << " via=" << viaHop << " ttl=" << fwdR->getTtl() << endl;
        std::cout << "AODV RREP fwd node=" << nodeId << " via=" << viaHop << " ttl=" << fwdR->getTtl() << std::endl;
        enqueueAodvOrSend(fwdR);
    }
}

void LoRaNodeApp::enqueueAodvOrSend(LoRaAppPacket *pkt) {
    LoRaMac *lrmc = (LoRaMac *)getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac");
    if (lrmc->fsm.getState() == IDLE) {
        send(pkt, "appOut");
    } else {
        aodvPacketsToSend.push_back(*pkt);
        delete pkt;
        aodvPacketsDue = true;
        if (!selfPacket->isScheduled()) scheduleAt(simTime() + 0.05, selfPacket);
    }
}

} //end namespace inet
