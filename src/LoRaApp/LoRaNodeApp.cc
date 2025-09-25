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
#include <algorithm> // for std::max

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
            StationaryMobility *mobility = check_and_cast<StationaryMobility *>(
                    host->getSubmodule("mobility"));
            if (nodeId == 0 && routingMetric == 0){ // end node 0 at middle
                mobility->par("initialX").setDoubleValue(minX + sepX * (cols/2));
                mobility->par("initialY").setDoubleValue(minY + sepY * (cols/2)+ uniform(0,100));
            }
            else{
                mobility->par("initialX").setDoubleValue(
                        minX + sepX * (nodeId % cols) + uniform(0,100));
                mobility->par("initialY").setDoubleValue(
                        minY + sepY * ((int) nodeId / cols) + uniform(0,100));
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
    bool forceSingleDestination = par("forceSingleDestination");
    int forcedDestinationId = par("forcedDestinationId");

        numberOfPacketsToForward = par("numberOfPacketsToForward");

        packetsToForwardMaxVectorSize = par("packetsToForwardMaxVectorSize");

        LoRa_AppPacketSent = registerSignal("LoRa_AppPacketSent");
    LoRa_AppPacketDelivered = registerSignal("LoRa_AppPacketDelivered");

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
    // Prepare delivered CSV path (per-node file)
    openDeliveredCsv();

        //Node identifier
        nodeId = getContainingNode(this)->getIndex();

        //Application acknowledgment
        requestACKfromApp = par("requestACKfromApp");
        stopOnACK = par("stopOnACK");
        AppACKReceived = false;
        firstACK = 0;
        // Proactively create ACK latency CSV if ACKs are requested so user sees the file even before first ACK arrives
        if (requestACKfromApp && !ackLatencyCsvReady) {
            openAckLatencyCsv();
        }

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

        // Routing packets timer (enforce a minimum start delay of 5s)
        {
            simtime_t base = par("timeToFirstRoutingPacket");
            if (base < 5) base = 5;
            timeToFirstRoutingPacket = base + getTimeToNextRoutingPacket();
        }
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

        // Data packets timer (enforce minimum start delay of 5s)
        {
            simtime_t base = par("timeToFirstDataPacket");
            if (base < 5) base = 5;
            timeToFirstDataPacket = base + getTimeToNextDataPacket();
        }
        if (LoRaPacketsToSend.size() > 0) {
                    dataPacketsDue = true;
                    nextDataPacketTransmissionTime = timeToFirstDataPacket;
                    EV << "Time to first data packet: " << timeToFirstDataPacket << endl;
        }

        // Forward packets timer (enforce minimum start delay of 5s)
        {
            simtime_t base = par("timeToFirstForwardPacket");
            if (base < 5) base = 5;
            timeToFirstForwardPacket = base + getTimeToNextForwardPacket();
        }
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
    nodeId = getContainingNode(this)->getIndex();
    routingMetric = par("routingMetric");

    if (nodeId == 0 && routingMetric == 0) // only for the end nodes and the packet originator
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

    // Received a selfMessage for transmitting a scheduled packet.  Only proceed to send a packet
    // if the 'mac' module in 'LoRaNic' is IDLE and the warmup period is due (TODO: implement check for the latter).
    LoRaMac *lrmc = (LoRaMac *)getParentModule()->getSubmodule("LoRaNic")->getSubmodule("mac");
    if (lrmc->fsm.getState() == IDLE ) {

        simtime_t txDuration = 0;
        simtime_t nextScheduleTime = 0;

        bool sendData = false;
        bool sendForward = false;
        bool sendRouting = false;

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

        // Check if there are routing packets to send
        if ( routingPacketsDue && simTime() >= nextRoutingPacketTransmissionTime ) {
            sendRouting = true;
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
                nextRoutingPacketTransmissionTime = simTime() + std::max(getTimeToNextRoutingPacket().dbl(), txDuration.dbl()/dutyCycle);
            }
            else {
                // Update next routing packet transmission time
                nextRoutingPacketTransmissionTime = simTime() + std::max(getTimeToNextRoutingPacket().dbl(), txDuration.dbl());
            }
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
                    nextDataPacketTransmissionTime = simTime() + std::max(getTimeToNextDataPacket().dbl(), txDuration.dbl()/dutyCycle);
                }
                else {
                    // Update next data packet transmission time
                    nextDataPacketTransmissionTime = simTime() + std::max(getTimeToNextDataPacket().dbl(), txDuration.dbl());
                }
            }
            // or send forward packet
            else {
                txDuration = sendForwardPacket();
                if (enforceDutyCycle) {
                    // Update duty cycle end
                    dutyCycleEnd = simTime() + txDuration/dutyCycle;
                    // Update next forward packet transmission time, taking the duty cycle into account
                    nextForwardPacketTransmissionTime = simTime() + std::max(getTimeToNextForwardPacket().dbl(), txDuration.dbl()/dutyCycle);
                }
                else {
                // Update next forward packet transmission time
                    nextForwardPacketTransmissionTime = simTime() + std::max(getTimeToNextForwardPacket().dbl(), txDuration.dbl());
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
        // then based on data packets, if they are to be scheduled earlier,
        if (dataPacketsDue) {
            nextScheduleTime = std::min(nextScheduleTime.dbl(), nextDataPacketTransmissionTime.dbl());
        }
        // then based on forward packets, if they are to be scheduled earlier,
        if (forwardPacketsDue) {
            nextScheduleTime = std::min(nextScheduleTime.dbl(), nextForwardPacketTransmissionTime.dbl());
        }
        // but, in any case, not earlier than simTime()+txDuration.
    nextScheduleTime = std::max(nextScheduleTime.dbl(), simTime().dbl()+txDuration.dbl());

        // Take the duty cycle into account
        if (enforceDutyCycle) {
            nextScheduleTime = std::max(nextScheduleTime.dbl(), dutyCycleEnd.dbl());
        }

        // Last, although this should never happen, check the schedule time is in the future, otherwise just add a 1s delay
        if (! (nextScheduleTime > simTime()) ) {
            nextScheduleTime = simTime() + 1;
        }

        // Schedule a self message to send routing, data or forward packets. Since some calculations lose precision, add an extra delay
        // (10x simtime-resolution unit) to avoid timing conflicts in the LoRaMac layer when simulations last very long.
        if (routingPacketsDue || dataPacketsDue || forwardPacketsDue) {
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
        scheduleAt(simTime() + 0.00002, selfPacket);
    }
}

void LoRaNodeApp::handleMessageFromLowerLayer(cMessage *msg) {
    receivedPackets++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // nextHop filtering (new deterministic unicast layer on top of broadcast medium)
    // Accept if nextHop == -1 (broadcast/unknown) OR I am the intended nextHop OR I am the final destination (always accept own dest)
    if (packet->hasPar("nextHop") || true) { // field exists in message definition now
        int nh = packet->getNextHop();
        if (packet->getDestination() != nodeId) { // if not final destination
            if (nh >= 0 && nh != nodeId) {
                // Not for me to process; early drop
                EV << "[Node " << nodeId << "] Dropping packet seq=" << packet->getDataInt() << " not my nextHop (" << nh << ")" << endl;
                delete msg;
                return;
            }
        }
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
    // Make a safe copy because we'll potentially erase from the vector
    LoRaNodeApp::singleMetricRoute cand = candidate;
    // Find all entries for this destination id
    int bestIdx = -1;
    for (int i = 0; i < (int)singleMetricRoutingTable.size(); ++i) {
        if (singleMetricRoutingTable[i].id == candidate.id) {
            if (bestIdx == -1) {
                bestIdx = i;
            } else {
                // Determine if current i is better than current bestIdx
                const auto &cur = singleMetricRoutingTable[i];
                const auto &best = singleMetricRoutingTable[bestIdx];
                if (cur.metric < best.metric ||
                    (cur.metric == best.metric && cur.valid > best.valid)) {
                    bestIdx = i;
                }
            }
        }
    }

    // Compare candidate against current best (if any)
    bool candidateIsBest = true;
    if (bestIdx != -1) {
        const auto &best = singleMetricRoutingTable[bestIdx];
        if (best.metric < cand.metric ||
            (best.metric == cand.metric && best.valid >= cand.valid)) {
            candidateIsBest = false;
        }
    }

    if (candidateIsBest) {
        // Remove all routes to this destination, then insert candidate
        for (auto it = singleMetricRoutingTable.begin(); it != singleMetricRoutingTable.end(); ) {
            if (it->id == cand.id) it = singleMetricRoutingTable.erase(it); else ++it;
        }
        singleMetricRoutingTable.push_back(cand);
    } else {
        // Candidate is worse; if there is no entry for this destination yet, insert it, else ignore
        if (bestIdx == -1) {
            singleMetricRoutingTable.push_back(cand);
            // Now reduce to one (candidate is the only one)
        }
        // else do nothing (keep the existing best)
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

void LoRaNodeApp::openDeliveredCsv() {
    // Build folder and file name under simulations/delivered_packets
#ifdef _WIN32
    const char sep = '\\';
#else
    const char sep = '/';
#endif
    std::string folder = std::string("delivered_packets");
#ifdef _WIN32
    _mkdir(folder.c_str());
#else
    mkdir(folder.c_str(), 0775);
#endif
    std::stringstream ss;
    ss << folder << sep << "node_" << nodeId << "_delivered.csv";
    deliveredCsvPath = ss.str();
    // Create the file if absent and write a simple header once
    std::ofstream f(deliveredCsvPath, std::ios::out | std::ios::app);
    if (f.is_open()) {
        if (f.tellp() == 0) {
            // Added nextHop column after viaBefore to record final nextHop seen at delivery time
            f << "simTime,src,dst,seq,ttl,viaBefore,nextHop,arrivalNode" << std::endl;
        }
        f.close();
        deliveredCsvReady = true;
    } else {
        deliveredCsvReady = false;
    }
}

void LoRaNodeApp::openAckLatencyCsv() {
#ifdef _WIN32
    const char sep = '\\';
#else
    const char sep = '/';
#endif
    std::string folder = std::string("ack_latency");
#ifdef _WIN32
    _mkdir(folder.c_str());
#else
    mkdir(folder.c_str(), 0775);
#endif
    std::stringstream ss;
    ss << folder << sep << "node_" << nodeId << "_ack_latency.csv";
    ackLatencyCsvPath = ss.str();
    EV << "[Node " << nodeId << "] Opening ACK latency CSV at: " << ackLatencyCsvPath << endl;
    std::ofstream f(ackLatencyCsvPath, std::ios::out | std::ios::app);
    if (f.is_open()) {
        if (f.tellp() == 0) {
            f << "simTime,src,dst,seq,dataDeparture,ackArrival,rtLatency" << std::endl;
        }
        f.close();
        ackLatencyCsvReady = true;
        EV << "[Node " << nodeId << "] ACK latency CSV ready." << endl;
    } else {
        ackLatencyCsvReady = false;
        EV << "[Node " << nodeId << "] Failed to open ACK latency CSV." << endl;
    }
}

void LoRaNodeApp::logAckLatency(int dataSrc, int dataDst, int dataSeq, simtime_t dataDeparture, simtime_t ackArrival) {
    if (!ackLatencyCsvReady) return;
    ackLatencyCsv.open(ackLatencyCsvPath, std::ios::out | std::ios::app);
    if (!ackLatencyCsv.is_open()) return;
    ackLatencyCsv << simTime() << "," << dataSrc << "," << dataDst << "," << dataSeq << "," << dataDeparture << "," << ackArrival << "," << (ackArrival - dataDeparture) << std::endl;
    ackLatencyCsv.flush();
    ackLatencyCsv.close();
}

void LoRaNodeApp::noteDataForAckTracking(const LoRaAppPacket *packet) {
    if (!packet->getOptions().getAppACKReq()) return; // only track those needing ACK
    if (!ackLatencyCsvReady) openAckLatencyCsv();
    std::tuple<int,int,int> key(packet->getSource(), packet->getDestination(), packet->getDataInt());
    pendingAckMap[key] = packet->getDepartureTime();
    EV << "[Node " << nodeId << "] Tracking DATA for ACK seq=" << packet->getDataInt() << " src=" << packet->getSource() << " dst=" << packet->getDestination() << " depTime=" << packet->getDepartureTime() << endl;
}

void LoRaNodeApp::logDeliveredPacket(const LoRaAppPacket *packet) {
    if (!deliveredCsvReady) return;
    deliveredCsv.open(deliveredCsvPath, std::ios::out | std::ios::app);
    if (!deliveredCsv.is_open()) return;
    deliveredCsv << simTime() << ","
                 << packet->getSource() << ","
                 << packet->getDestination() << ","
                 << packet->getDataInt() << ","
                 << packet->getTtl() << ","
                 << packet->getVia() << ","
                 << packet->getNextHop() << ","
                 << nodeId
                 << std::endl;
    deliveredCsv.flush();
    deliveredCsv.close();
}

void LoRaNodeApp::logRoutingSnapshot(const char *eventName) {
    if (!routingCsvReady) return;
    // Open file in truncate mode to reflect current snapshot
    routingCsv.open(routingCsvPath, std::ios::out | std::ios::trunc);
    if (!routingCsv.is_open()) return;
    // Write rows in key=value format for readability
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
    // Single-metric table
    for (const auto &r : singleMetricRoutingTable) {
        routingCsv << "simTime=" << simTime()
                   << ",event=" << eventName
                   << ",nodeId=" << nodeId
                   << ",metricType=" << metricName
                   << ",tableSize=" << singleMetricRoutingTable.size()
                   << ",id=" << r.id
                   << ",via=" << r.via
                   << ",metric=" << r.metric
                   << ",validUntil=" << r.valid
                   << ",sf="
                   << ",priMetric="
                   << ",secMetric="
                   << std::endl;
    }
    // Dual-metric table
    for (const auto &r : dualMetricRoutingTable) {
        routingCsv << "simTime=" << simTime()
                   << ",event=" << eventName
                   << ",nodeId=" << nodeId
                   << ",metricType=" << metricName
                   << ",tableSize=" << dualMetricRoutingTable.size()
                   << ",id=" << r.id
                   << ",via=" << r.via
                   << ",metric="
                   << ",validUntil=" << r.valid
                   << ",sf=" << r.sf
                   << ",priMetric=" << r.priMetric
                   << ",secMetric=" << r.secMetric
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
                nextScheduleTime = std::max(nextScheduleTime.dbl(), dutyCycleEnd.dbl());
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
        // Log definitive delivery and emit a signal for statistics
        logDeliveredPacket(packet);
        emit(LoRa_AppPacketDelivered, (long)packet->getSource());
        // If application requested an ACK and this is true destination, enqueue ACK back to source
        if (packet->getOptions().getAppACKReq()) {
            enqueueAckFor(packet);
        }
        // OLD behavior was to forward the DATA again even at destination, which delayed ACK transmission.
        // Since we only need an ACK back to the source, we stop here and dispose of the packet.
        delete packet;
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

// Create a minimal ACK packet referencing original DATA and push to send queue
void LoRaNodeApp::enqueueAckFor(const LoRaAppPacket *dataPkt) {
    LoRaAppPacket ackTemplate;
    ackTemplate.setMsgType(ACK);
    ackTemplate.setDataInt(dataPkt->getDataInt()); // could store seq if available
    ackTemplate.setSource(nodeId); // this node (destination of DATA) becomes ACK source
    ackTemplate.setDestination(dataPkt->getSource());
    ackTemplate.setVia(nodeId);
    ackTemplate.setTtl(packetTTL); // reuse default TTL
    ackTemplate.getOptions().setAppACKReq(false); // ACK itself does not request further ACK
    // minimal size: reuse data packet size logic? keep small (e.g., 5 bytes header) but here reuse existing size field setter
    ackTemplate.setByteLength(std::min( (int) dataPkt->getByteLength(), 10));
    ackTemplate.setDepartureTime(simTime());
    // Initialize nextHop for ACK (reverse path): we try to find best route to original source
    sanitizeRoutingTable();
    int routeIndex = getBestRouteIndexTo(ackTemplate.getDestination());
    if (routeIndex >= 0) {
        if (singleMetricRoutingTable.size() > 0) {
            ackTemplate.setNextHop(singleMetricRoutingTable[routeIndex].via);
        } else if (dualMetricRoutingTable.size() > 0) {
            ackTemplate.setNextHop(dualMetricRoutingTable[routeIndex].via);
        }
    } else {
        ackTemplate.setNextHop(-1); // fallback broadcast
    }
    // Push into send buffer
    LoRaPacketsToSend.push_back(ackTemplate);
    // Counters will be incremented when actually transmitted in sendDataPacket()
    dataPacketsDue = true; // reuse same scheduling path
    if (!selfPacket->isScheduled()) {
        // schedule soon respecting duty cycle
        simtime_t nextScheduleTime = simTime() + 10*simTimeResolution;
        if (enforceDutyCycle) {
            nextScheduleTime = std::max(nextScheduleTime.dbl(), dutyCycleEnd.dbl());
        }
        if (!(nextScheduleTime > simTime())) {
            nextScheduleTime = simTime() + 1;
        }
        scheduleAt(nextScheduleTime, selfPacket);
    }
    EV << "[Node " << nodeId << "] Enqueued ACK for original DATA seq=" << dataPkt->getDataInt() << " back to src=" << dataPkt->getSource() << endl;
        // Guarantee ACK latency CSV existence when ACK is generated at destination side
        if (!ackLatencyCsvReady) {
            openAckLatencyCsv();
            if (ackLatencyCsvReady) {
                EV << "[Node " << nodeId << "] ack_latency CSV created on ACK enqueue." << endl;
            } else {
                EV << "[Node " << nodeId << "] FAILED to create ack_latency CSV on ACK enqueue." << endl;
            }
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
    // For an ACK: original DATA src = packet->getDestination(), original DATA dst = packet->getSource()
    std::tuple<int,int,int> key(packet->getDestination(), packet->getSource(), packet->getDataInt());
    auto it = pendingAckMap.find(key);
    if (it != pendingAckMap.end()) {
        simtime_t dep = it->second;
        if (!ackLatencyCsvReady) openAckLatencyCsv();
        logAckLatency(std::get<0>(key), std::get<1>(key), std::get<2>(key), dep, simTime());
        pendingAckMap.erase(it);
        AppACKReceived = true;
        EV << "[Node " << nodeId << "] ACK received for seq=" << packet->getDataInt() << " originalSrc=" << std::get<0>(key) << " originalDst=" << std::get<1>(key) << " RTT=" << (simTime()-dep) << endl;
    }
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

    // PRIORITIZE ACKs: If any ACK is pending in LoRaPacketsToSend, send it before forwarding queued packets.
    int ackIndex = -1;
    for (size_t i = 0; i < LoRaPacketsToSend.size(); ++i) {
        if (LoRaPacketsToSend[i].getMsgType() == ACK) { ackIndex = (int)i; break; }
    }

    bool canSendOwn = LoRaPacketsToSend.size() > 0 && (ackIndex >= 0 || LoRaPacketsToForward.size() == 0 || bernoulli(ownDataPriority));

    if (canSendOwn) {

        bubble("Sending a local data packet!");

        // Name packets to ease tracking
        std::string fullName = dataPacket->getName();;
        const char* addName = "Orig";
        fullName += addName;
        fullName += std::to_string(nodeId);


    // Select packet: ACK first if present, else the first queued
    size_t pick = (ackIndex >= 0) ? (size_t)ackIndex : 0;
    dataPacket->setMsgType(LoRaPacketsToSend[pick].getMsgType());
    dataPacket->setDataInt(LoRaPacketsToSend[pick].getDataInt());
    dataPacket->setSource(LoRaPacketsToSend[pick].getSource());
    dataPacket->setVia(LoRaPacketsToSend[pick].getSource());
    dataPacket->setDestination(LoRaPacketsToSend[pick].getDestination());
    dataPacket->setTtl(LoRaPacketsToSend[pick].getTtl());
    dataPacket->getOptions().setAppACKReq(LoRaPacketsToSend[pick].getOptions().getAppACKReq());
    dataPacket->setByteLength(LoRaPacketsToSend[pick].getByteLength());
    dataPacket->setDepartureTime(simTime());

        addName = "Dest";
        fullName += addName;
        fullName += std::to_string(dataPacket->getDestination());
        dataPacket->setName(fullName.c_str());

    LoRaPacketsToSend.erase(LoRaPacketsToSend.begin() + pick);

        transmit = true;

        if (dataPacket->getMsgType() == DATA) {
            sentDataPackets++;
            noteDataForAckTracking(dataPacket);
            EV << "[Node " << nodeId << "] Originated DATA seq=" << dataPacket->getDataInt() << " dst=" << dataPacket->getDestination() << " ACKReq=" << dataPacket->getOptions().getAppACKReq() << endl;
        } else if (dataPacket->getMsgType() == ACK) {
            sentAckPackets++;
        }
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
                        if (dataPacket->getMsgType() == DATA) {
                            forwardedDataPackets++;
                        } else if (dataPacket->getMsgType() == ACK) {
                            forwardedAckPackets++;
                        }
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

        // Determine nextHop just-in-time before assigning via (legacy field)
        assignNextHop(dataPacket);
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

        // Update nextHop for forwarded packet
        assignNextHop(forwardPacket);
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

// Assign nextHop based on current best route to destination (single or dual metric tables)
void LoRaNodeApp::assignNextHop(LoRaAppPacket *pkt) {
    if (!pkt) return;
    int dst = pkt->getDestination();
    // Destination reached locally handled before calling this; if already destination, keep -1
    sanitizeRoutingTable();
    int routeIndex = getBestRouteIndexTo(dst);
    if (routeIndex >= 0) {
        if (singleMetricRoutingTable.size() > 0) {
            // routeIndex refers to appropriate table (single vs dual) depending on which had entries
            // We infer by checking bounds: if routeIndex within single table size
            if (routeIndex < (int)singleMetricRoutingTable.size()) {
                pkt->setNextHop(singleMetricRoutingTable[routeIndex].via);
            } else if (dualMetricRoutingTable.size() > 0) {
                int dualIdx = routeIndex - (int)singleMetricRoutingTable.size();
                if (dualIdx >=0 && dualIdx < (int)dualMetricRoutingTable.size()) {
                    pkt->setNextHop(dualMetricRoutingTable[dualIdx].via);
                }
            }
        } else if (dualMetricRoutingTable.size() > 0) {
            if (routeIndex < (int)dualMetricRoutingTable.size()) {
                pkt->setNextHop(dualMetricRoutingTable[routeIndex].via);
            }
        }
    } else {
        pkt->setNextHop(-1); // broadcast fallback
    }
    EV << "[Node " << nodeId << "] assignNextHop dst=" << dst << " nextHop=" << pkt->getNextHop() << endl;
}



simtime_t LoRaNodeApp::sendRoutingPacket() {

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

    if (!onlyNode0SendsPackets || nodeId == 0) {
        std::vector<int> destinations = { };
        // If configured, force node 0 to send only to a specific destination
        bool forceSingleDestination = par("forceSingleDestination");
        int forcedDestinationId = par("forcedDestinationId");
        if (forceSingleDestination && nodeId == 0 && forcedDestinationId >= 0 && forcedDestinationId < numberOfNodes && forcedDestinationId != nodeId) {
            destinations.push_back(forcedDestinationId);
        } else {
            if (numberOfDestinationsPerNode == 0 )
                numberOfDestinationsPerNode = numberOfNodes-1;

            while (destinations.size() < numberOfDestinationsPerNode
                    && numberOfNodes - 1 - destinations.size() > 0) {

                int destination = intuniform(0, numberOfNodes - 1);

                if (destination != nodeId) {
                    bool newDestination = true;

                    for (int i = 0; i < destinations.size(); i++) {
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
                // Assign a unique, monotonically increasing sequence per packet
                dataPacket->setDataInt(currDataInt++);
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
                // Ensure ACK latency CSV is created early if ACKs are requested, even before actual transmission
                if (requestACKfromApp && !ackLatencyCsvReady) {
                    openAckLatencyCsv();
                }
                delete dataPacket;
            }
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

    int payloadSymbNb = 8 + std::max( (int) std::ceil((8*payloadBytes - 4*cInfo->getLoRaSF() + 28 + 16 - 20*0)
                                 /(4*(cInfo->getLoRaSF()-2*0))) * (cInfo->getLoRaCR() + 4), 0);

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



} //end namespace inet
