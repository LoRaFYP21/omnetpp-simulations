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
#include <set>
#include <cstdio> // snprintf for dynamic event names
#include <climits> // INT_MAX for end-node filtering

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

// ---------------------------------------------------------------------------
// Routing Metric Enum Mapping (documentation)
// NO_FORWARDING (0)                : Node generates/receives only; no forwarding logic.
// FLOODING_BROADCAST_SINGLE_SF (1) : Blind broadcast forwarding using a single SF.
// SMART_BROADCAST_SINGLE_SF (2)    : Broadcast with additional heuristics (e.g., duplicate avoidance).
// HOP_COUNT_SINGLE_SF (3)          : Single-metric table; metric = hop count (lower is better).
// RSSI_SUM_SINGLE_SF (4)           : Single-metric; metric = sum of RSSI along path (higher raw RSSI -> lower derived cost assumed).
// RSSI_PROD_SINGLE_SF (5)          : Single-metric; metric = product/aggregation of RSSI factors.
// ETX_SINGLE_SF (6)                : Single-metric; metric = Expected Transmission Count (lower = more reliable path).
// TIME_ON_AIR_HC_CAD_SF (11)       : Dual-metric; primary combines time-on-air + hop count + CAD attempts.
// TIME_ON_AIR_SF_CAD_SF (12)       : Dual-metric; primary combines time-on-air + spreading factor + CAD attempts.
// ---------------------------------------------------------------------------
// NOTE: Comments in some .ini files previously labelled metric 3 as ETX; this
// patch clarifies correct mapping where 3 = HOP_COUNT and 6 = ETX.
// ---------------------------------------------------------------------------

// Helper: identify if this app instance should behave like an end node.
// Rescue end nodes rely on iAmRescue; regular end nodes rely on iAmEnd.
static inline bool isEndNodeHost(cSimpleModule* self) {
    cModule *hostMod = getContainingNode(self);
    if (!hostMod) return false;
    try {
        // Rescue nodes act as end nodes without needing iAmEnd
        if (hostMod->hasPar("iAmRescue") && (bool)hostMod->par("iAmRescue"))
            return true;
        // Regular end nodes
        if (hostMod->hasPar("iAmEnd") && (bool)hostMod->par("iAmEnd"))
            return true;
    } catch (...) {
        return false;
    }
    return false;
}

// Helper: identify if this app instance belongs to a rescue node (host parameter iAmRescue=true)
static inline bool isRescueNodeHost(cSimpleModule* self) {
    cModule *hostMod = getContainingNode(self);
    if (!hostMod) return false;
    if (!hostMod->hasPar("iAmRescue")) return false;
    try {
        return (bool)hostMod->par("iAmRescue");
    } catch (...) {
        return false;
    }
}

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
        originalNodeIndex = nodeId;  // Store original index before offset
        // Offset node ID based on host role:
        // - Rescue nodes: +2000 (preferred when iAmRescue is true)
        // - End nodes (non-rescue) participating in routing: +1000
        {
            cModule *hostMod = getContainingNode(this);
            bool isRescue = false;
            bool isEnd = false;
            if (hostMod && hostMod->hasPar("iAmRescue")) {
                isRescue = (bool)hostMod->par("iAmRescue");
            }
            if (hostMod && hostMod->hasPar("iAmEnd")) {
                isEnd = (bool)hostMod->par("iAmEnd");
            } else {
                // Fallback: try detecting by vector base name
                cModule *parent = getParentModule();
                if (parent) {
                    const char *baseName = parent->getName(); // "loRaNodes" or "loRaEndNodes"
                    if (strcmp(baseName, "loRaEndNodes") == 0) isEnd = true;
                }
            }
            if (isRescue) {
                nodeId += 2000;
            } else if (isEnd && routingMetric != 0) {
                nodeId += 1000;
            }
        }

        // Initialize expected convergence count using only relay nodes
        // Do this once globally; end nodes do not contribute to the expected count
        if (stopRoutingWhenAllConverged && globalNodesExpectingConvergence == 0) {
            int relayCount = 0;
            try {
                // Parameter holds relay array size in our networks
                relayCount = par("numberOfNodes");
            } catch (...) {
                relayCount = numberOfNodes; // fallback to member
            }
            if (relayCount > 0) {
                globalNodesExpectingConvergence = relayCount;
            }
        }

        // Fresh path log per simulation run: have node 0 truncate and recreate header immediately
        if (nodeId == 0) {
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
            std::stringstream pss; pss << folder << sep << "paths.csv";
            std::ofstream pf(pss.str(), std::ios::out | std::ios::trunc);
            if (pf.is_open()) {
                pf << "simTime,event,packetSeq,src,dst,currentNode,ttlAfterDecr,chosenVia,nextHopType" << std::endl;
                pf.close();
            }
            // Reset the flag used in ensurePathLogInitialized so it knows file already has header
            pathLogReady = false; // will be set true on first ensurePathLogInitialized() call
        }
        std::pair<double, double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);

        // Generate random location for nodes if circle deployment type
    if (strcmp(host->par("deploymentType").stringValue(), "circle") == 0) {
        coordsValues = generateUniformCircleCoordinates(
            host->par("rad").doubleValue(),
            host->par("centX").doubleValue(),
            host->par("centY").doubleValue());
        cModule *mobMod = host->getSubmodule("mobility");
        if (mobMod) {
        if (mobMod->hasPar("initialX")) mobMod->par("initialX").setDoubleValue(coordsValues.first);
        if (mobMod->hasPar("initialY")) mobMod->par("initialY").setDoubleValue(coordsValues.second);
        }

        } else if (strcmp(host->par("deploymentType").stringValue(), "edges")== 0) {
            double minX = host->par("minX");
            double maxX = host->par("maxX");
            double minY = host->par("minY");
            double maxY = host->par("maxY");
        cModule *mobMod = host->getSubmodule("mobility");
//            if (strcmp(host->par("deploymentType").stringValue(), "circle")==0) {
//                       coordsValues = generateUniformCircleCoordinates(host->par("maxGatewayDistance").doubleValue(), host->par("gatewayX").doubleValue(), host->par("gatewayY").doubleValue());
//                       StationaryMobility *mobility = check_and_cast<StationaryMobility *>(host->getSubmodule("mobility"));
//                       mobility->par("initialX").setDoubleValue(coordsValues.first);
//                       mobility->par("initialY").setDoubleValue(coordsValues.second);
//                    }

        if (mobMod) {
        double newX = minX + maxX * (((nodeId + 1) % 4 / 2) % 2);
        double newY = minY + maxY * (((nodeId) % 4 / 2) % 2);
        if (mobMod->hasPar("initialX")) mobMod->par("initialX").setDoubleValue(newX);
        if (mobMod->hasPar("initialY")) mobMod->par("initialY").setDoubleValue(newY);
        }
        } else if (strcmp(host->par("deploymentType").stringValue(), "grid") == 0) {
            double minX = host->par("minX");
            double sepX = host->par("sepX");
            double minY = host->par("minY");
            double sepY = host->par("sepY");
            int cols = int(sqrt(numberOfNodes));
            cModule *mobMod2 = host->getSubmodule("mobility");
            if (mobMod2) {
                double newX;
                double newY;
                if (nodeId == 0 && routingMetric == 0){ // end node 0 at middle
                    newX = minX + sepX * (cols/2);
                    newY = minY + sepY * (cols/2)+ uniform(0,100);
                } else {
                    newX = minX + sepX * (nodeId % cols) + uniform(0,100);
                    newY = minY + sepY * ((int) nodeId / cols) + uniform(0,100);
                }
                if (mobMod2->hasPar("initialX")) mobMod2->par("initialX").setDoubleValue(newX);
                if (mobMod2->hasPar("initialY")) mobMod2->par("initialY").setDoubleValue(newY);
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
            cModule *mobMod3 = host->getSubmodule("mobility");
            if (mobMod3) {
                if (mobMod3->hasPar("initialX")) mobMod3->par("initialX").setDoubleValue(inix);
                if (mobMod3->hasPar("initialY")) mobMod3->par("initialY").setDoubleValue(iniy);
            }
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
    unicastNoRouteDrops = 0;
    unicastWrongNextHopDrops = 0;
    unicastFallbackBroadcasts = 0;

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

        // Routing freeze parameters
        if (hasPar("freezeRoutingAtThreshold"))
            freezeRoutingAtThreshold = par("freezeRoutingAtThreshold");
        if (hasPar("routingFreezeUniqueCount"))
            routingFreezeUniqueCount = par("routingFreezeUniqueCount");
        if (hasPar("stopRoutingWhenAllConverged"))
            stopRoutingWhenAllConverged = par("stopRoutingWhenAllConverged");
        if (hasPar("freezeValidityHorizon")) {
            // Clamp to a reasonable positive value; OMNeT++ simtime range with scale exponent -10 is ~ +/-9.22e8 seconds
            double horizon = par("freezeValidityHorizon").doubleValue();
            if (horizon <= 0) {
                // fallback: 10 * routeTimeout or 1e5 whichever greater
                double fallback = std::max(1e5, 10.0 * routeTimeout.dbl());
                EV_WARN << "freezeValidityHorizon <= 0 supplied; using fallback " << fallback << "s" << endl;
                horizon = fallback;
            } else if (horizon > 9.0e8) {
                EV_WARN << "freezeValidityHorizon=" << horizon << " too large; clamping to 9.0e8s to avoid simtime overflow" << endl;
                horizon = 9.0e8; // keep a safety margin
            }
            freezeValidityHorizon = horizon; // stored as simtime_t later when used
        } else {
            // Parameter absent (older configs) -> derive heuristic
            double fallback = std::max(1e5, 10.0 * routeTimeout.dbl());
            freezeValidityHorizon = fallback;
        }
        routingFrozen = false;
        routingFrozenTime = -1;
        locallyConverged = false;

        // Initialize global convergence accounting once per process
        // Determine participating nodes: use numberOfNodes (relay nodes) unless routingMetric == 0 (end nodes)
        if (globalNodesExpectingConvergence == 0) {
            // We consider all nodes in the loRaNodes vector as participants for simplicity
            int totalCandidates = par("numberOfNodes");
            if (totalCandidates <= 0) totalCandidates = 0;
            globalNodesExpectingConvergence = totalCandidates;
        }
        // Prepare global CSV once
        if (!globalConvergenceCsvReady) {
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
            std::stringstream gss; gss << folder << sep << "global_routing_convergence.csv";
            globalConvergenceCsvPath = gss.str();
            std::ofstream gf(globalConvergenceCsvPath, std::ios::out | std::ios::app);
            if (gf.is_open()) {
                if (gf.tellp() == 0) {
                    gf << "simTime,event,nodeId,uniqueCount,totalNodes,threshold" << std::endl;
                }
                gf.close();
                globalConvergenceCsvReady = true;
            }
        }

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

    // Note: prepare CSV paths after nodeId is known (set further down)


        //Node identifier (re-assign and apply rescue/end offset consistently)
            nodeId = getContainingNode(this)->getIndex();
        {
            cModule *hostMod = getContainingNode(this);
            bool isRescue = false;
            bool isEnd = false;
            if (hostMod && hostMod->hasPar("iAmRescue")) {
                isRescue = hostMod->par("iAmRescue").boolValue();
            }
            if (hostMod && hostMod->hasPar("iAmEnd")) {
                isEnd = hostMod->par("iAmEnd").boolValue();
            } else {
                cModule *parent = getParentModule();
                if (parent) {
                    const char *baseName = parent->getName();
                    if (strcmp(baseName, "loRaEndNodes") == 0) isEnd = true;
                }
            }
            if (isRescue) {
                nodeId += 2000;
            } else if (isEnd && routingMetric != 0) {
                nodeId += 1000;
            }
        }

    // Prepare routing CSV path (per-node file) now that nodeId is set
    openRoutingCsv();

    // Prepare delivered CSV path (per-node file)
    openDeliveredCsv();

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
        // If global convergence already announced, suppress routing beacons
        routingPacketsDue = ! (stopRoutingWhenAllConverged && globalConvergedFired);
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
        // Failure scheduling parameters (local + optional global subset override)
        timeToFailureParam = par("timeToFailure");
        failureJitterFracParam = par("failureJitterFrac");

        // Global subset logic: only executed once globally, then applied per node
        initGlobalFailureSelection();
        if (globalFailureSubsetCountParam > 0) {
            // Determine if this node is in the chosen failing subset
            bool inSubset = std::find(globalFailingNodes.begin(), globalFailingNodes.end(), nodeId) != globalFailingNodes.end();
            if (!inSubset) {
                timeToFailureParam = -1; // force disable
            } else {
                // Compose failure time: start offset + (optional exponential tail)
                simtime_t startOffset = globalFailureStartTimeParam >= 0 ? simtime_t(globalFailureStartTimeParam) : SIMTIME_ZERO;
                simtime_t tail = SIMTIME_ZERO;
                if (globalFailureExpMeanParam > 0) {
                    tail = exponential(globalFailureExpMeanParam);
                }
                timeToFailureParam = startOffset + tail; // deterministic or shifted exponential
                failureJitterFracParam = 0; // jitter not applied with global logic
            }
        }

        if (timeToFailureParam >= 0 && !failureEvent) {
            scheduleFailure();
        }
        if (timeToFailureParam >= 0 && !failureEvent) {
            EV_WARN << "[FailureDiag] WARNING: timeToFailureParam=" << timeToFailureParam << " but failureEvent not scheduled (unexpected)" << endl;
            recordScalar("failureSchedulingAnomaly", 1);
        }

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
    // Recompute effective node id with rescue/end offsets for any path that recalculates it
    int baseId = getContainingNode(this)->getIndex();
    routingMetric = par("routingMetric");
    nodeId = baseId + (isRescueNodeHost(this) ? 2000 : ((isEndNodeHost(this) && routingMetric != 0) ? 1000 : 0));

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
    Coord coord;
    if (auto mobMod = host->getSubmodule("mobility")) {
        // Try dynamic cast to IMobility if available
        if (auto mobIface = dynamic_cast<inet::IMobility *>(mobMod)) {
            coord = mobIface->getCurrentPosition();
        } else if (mobMod->hasPar("initialX") && mobMod->hasPar("initialY")) {
            coord = Coord(mobMod->par("initialX").doubleValue(), mobMod->par("initialY").doubleValue(), 0);
        }
    }
//    recordScalar("positionX", coord.x);
//    recordScalar("positionY", coord.y);
    // DistanceX.record(coord.x);
    recordScalar("CordiX",coord.x);
    // DistanceY.record(coord.y);
    recordScalar("CordiY",coord.y);


    recordScalar("finalTP", loRaTP);
    recordScalar("finalSF", loRaSF);

    // Failure related scalars
    recordScalar("failed", failed ? 1 : 0);
    if (failureTime >= SIMTIME_ZERO)
        recordScalar("failureTime", failureTime);
    // Freeze related scalars
    recordScalar("freezeValidityHorizon", freezeValidityHorizon.dbl());
    recordScalar("routingFrozen", routingFrozen ? 1 : 0);
    if (routingFrozenTime >= SIMTIME_ZERO)
        recordScalar("routingFrozenTime", routingFrozenTime);

    // Export detailed routing tables only if explicitly enabled.
    // By default (parameter absent or false) we skip generating node_<id>_single.csv,
    // node_<id>_dual.csv and node_<id>_routing_table.txt, keeping only live snapshot
    // files node_<id>_routing.csv produced by logRoutingSnapshot().
    if (hasPar("exportDetailedRoutingTables") && par("exportDetailedRoutingTables").boolValue()) {
        exportRoutingTables();
    }

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
    // FIX: forwardPacketsNotSent previously (incorrectly) used LoRaPacketsToSend.size()
    recordScalar("forwardPacketsNotSent", LoRaPacketsToForward.size());

    recordScalar("forwardBufferFull", forwardBufferFull);
    // Strict unicast scalars
    recordScalar("unicastNoRouteDrops", unicastNoRouteDrops);
    recordScalar("unicastWrongNextHopDrops", unicastWrongNextHopDrops);
    recordScalar("unicastFallbackBroadcasts", unicastFallbackBroadcasts);

    // Replace unsafe erase-in-loop (iterator invalidation) with clear() operations.
    LoRaPacketsToSend.clear();
    LoRaPacketsToForward.clear();
    LoRaPacketsForwarded.clear();
    DataPacketsForMe.clear();

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
    // If node already failed, drop everything except to process (already processed) failure event
    if (failed) {
        delete msg;
        return;
    }
    if (msg->isSelfMessage()) {
        handleSelfMessage(msg);
    } else {
        handleMessageFromLowerLayer(msg);
    }
}


void LoRaNodeApp::handleSelfMessage(cMessage *msg) {

    // If this is the failure event, perform failure and stop any further actions
    if (msg == failureEvent) {
        performFailure();
        return;
    }

    if (failed) {
        return; // Ignore timers after failure
    }

    // If global convergence reached, stop routing immediately in all nodes
    if (globalConvergedFired) {
        routingPacketsDue = false;
    }

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
            forwardPacketsDue = true;  // FIXED: Set to true when packets need forwarding
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
    if (failed) { delete msg; return; }
    receivedPackets++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

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

        // Path instrumentation: log destination reception prior to processing
        if (packet->getMsgType() == DATA) {
            logPathHop(packet, "RX_DST_PRE");
        }

        manageReceivedPacketForMe(packet);
        if (firstDataPacketReceptionTime == 0) {
            firstDataPacketReceptionTime = simTime();
        }
        lastDataPacketReceptionTime = simTime();
    }
    // Else possible routing protocol broadcast
    else if (packet->getDestination() == BROADCAST_ADDRESS) {
        manageReceivedRoutingPacket(packet); // still processed as before
    }
    // Else data packet between other nodes (forwarding decision)
    else {
        bool broadcastMode = (routingMetric == FLOODING_BROADCAST_SINGLE_SF || routingMetric == SMART_BROADCAST_SINGLE_SF);
        if (broadcastMode) {
            // Legacy behaviour for broadcast-based dissemination
            if (packet->getVia() == BROADCAST_ADDRESS) {
                manageReceivedDataPacketToForward(packet);
                if (firstDataPacketReceptionTime == 0) firstDataPacketReceptionTime = simTime();
                lastDataPacketReceptionTime = simTime();
            } else if (packet->getVia() == nodeId) { // targeted within broadcast metric (rare)
                manageReceivedDataPacketToForward(packet);
                if (firstDataPacketReceptionTime == 0) firstDataPacketReceptionTime = simTime();
                lastDataPacketReceptionTime = simTime();
            } else {
                // Ignore silently: broadcast metric but not intended next hop
                unicastWrongNextHopDrops++; // reuse counter for diagnostics
            }
        } else {
            // Unicast metrics with possible fallback broadcast when a sender had no route.
            // Accept if we are the explicit next hop OR if the packet was broadcast fallback (via == BROADCAST_ADDRESS).
            if (packet->getVia() == nodeId || packet->getVia() == BROADCAST_ADDRESS) {
                // Count fallback broadcasts only when they are actually processed (diagnostic).
                if (packet->getVia() == BROADCAST_ADDRESS) {
                    unicastFallbackBroadcasts++; // repurpose counter: processed fallback broadcasts
                }
                // Path instrumentation: log reception before forwarding logic
                if (packet->getMsgType() == DATA) {
                    logPathHop(packet, "RX_FWD_PRE");
                }
                manageReceivedDataPacketToForward(packet);
                if (firstDataPacketReceptionTime == 0) firstDataPacketReceptionTime = simTime();
                lastDataPacketReceptionTime = simTime();
            } else {
                // We overheard a unicast not for us: drop silently (diagnostic count)
                unicastWrongNextHopDrops++;
            }
        }
    }

    delete msg;
}

void LoRaNodeApp::manageReceivedRoutingPacket(cMessage *msg) {
    //does not execting this part in our application

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // Check it actually is a routing message
    if (packet->getMsgType() == ROUTING) {

        // If routing is frozen, we still count the packet but ignore any table modifications
        if (routingFrozen) {
            receivedRoutingPackets++;
            return; // tables remain stable
        }

        receivedRoutingPackets++;

    sanitizeRoutingTable();

    // End-node advertise-only behavior: ignore routing table modifications to remain stateless
    if (isEndNodeHost(this)) {
        // Keep statistics/logging stable; just snapshot the (empty or static) table and return
        routingTableSize.collect(singleMetricRoutingTable.size());

    // ------------------------------------------------------------------
    // FILTER: Retain only routes that lead to end nodes (IDs >= 1000).
    // Requirement: Relay nodes should keep ONLY routes to end nodes
    // (e.g., 1000, 1001) and discard intermediate relay destinations.
    // End node IDs are offset by +1000 earlier during initialization.
    // We derive end node range from numberOfEndNodes parameter if present;
    // fallback: assume any id >= 1000 is an end node.
    // ------------------------------------------------------------------
    filterRoutesToEndNodes();

    // After filtering, collect the (reduced) size again to reflect final table
    routingTableSize.collect(singleMetricRoutingTable.size());
        logRoutingSnapshot("routing_packet_ignored_endnode");
        return;
    }

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
                if (routingFrozen) break; // skip modifications when frozen

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
                if (routingFrozen) break; // skip modifications when frozen

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

    // Enforce relay-side filtering: keep only end-node destinations in tables
    // before collecting size and logging the snapshot. This ensures
    // node_X_routing.csv reflects only end-node routes (e.g., 1000, 1001).
    filterRoutesToEndNodes();
    routingTableSize.collect(singleMetricRoutingTable.size());

    // Log snapshot after processing routing packet (post-filtering)
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
    // If routing already frozen, ignore any new candidate to keep table stable
    if (routingFrozen) return;
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

    // After any insertion, check if we just reached threshold unique destinations (default 16)
    if (firstTimeReached16 < SIMTIME_ZERO) {
        std::set<int> uniqueIds;
        for (const auto &r : singleMetricRoutingTable) uniqueIds.insert(r.id);
        if ((int)uniqueIds.size() >= routingFreezeUniqueCount) {
            firstTimeReached16 = simTime();
            // Lazy-open convergence CSV (node 0 creates header, others append)
            if (!convergenceCsvReady) {
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
                std::stringstream css; css << folder << sep << "routing_convergence.csv";
                convergenceCsvPath = css.str();
                std::ofstream cf(convergenceCsvPath, std::ios::out | std::ios::app);
                if (cf.is_open()) {
                    if (cf.tellp() == 0) {
                        cf << "simTime,event,nodeId,threshold,uniqueCount" << std::endl;
                    }
                    cf.close();
                    convergenceCsvReady = true;
                }
            }
            if (convergenceCsvReady) {
                std::ofstream cf(convergenceCsvPath, std::ios::out | std::ios::app);
                if (cf.is_open()) {
                    // Dynamic event label (retain legacy REACHED16 name when threshold==16 for backward compatibility)
                    if (routingFreezeUniqueCount == 16) {
                        cf << simTime() << ",REACHED16," << nodeId << "," << routingFreezeUniqueCount << "," << uniqueIds.size() << std::endl;
                    } else {
                        char ev[32];
                        std::snprintf(ev, sizeof(ev), "REACHED%d", routingFreezeUniqueCount);
                        cf << simTime() << "," << ev << "," << nodeId << "," << routingFreezeUniqueCount << "," << uniqueIds.size() << std::endl;
                    }
                    cf.close();
                }
            }
            // If freeze is enabled and not yet frozen, apply freeze now
            if (freezeRoutingAtThreshold && !routingFrozen) {
                routingFrozen = true;
                routingFrozenTime = simTime();
                // Extend validity horizons using configured freezeValidityHorizon
                simtime_t extendBy = freezeValidityHorizon; // already clamped during initialize
                for (auto &r : singleMetricRoutingTable) {
                    r.valid = simTime() + extendBy;
                }
                for (auto &r : dualMetricRoutingTable) {
                    r.valid = simTime() + extendBy;
                }
                // Log FREEZE event
                if (convergenceCsvReady) {
                    std::ofstream cf2(convergenceCsvPath, std::ios::out | std::ios::app);
                    if (cf2.is_open()) {
                        cf2 << simTime() << ",FREEZE," << nodeId << "," << routingFreezeUniqueCount << "," << uniqueIds.size() << std::endl;
                        cf2.close();
                    }
                }
            }
            // Announce local convergence into the global aggregator
            if (stopRoutingWhenAllConverged) {
                announceLocalConvergenceIfNeeded((int)uniqueIds.size());
                tryStopRoutingGlobally();
            }
        }
    }

}

// When this node reaches threshold the first time, bump the global counter and log
void LoRaNodeApp::announceLocalConvergenceIfNeeded(int uniqueCount) {
    // End nodes (iAmEnd=true) do not contribute to the global convergence count
    if (isEndNodeHost(this)) return;
    if (locallyConverged) return;
    locallyConverged = true;
    // Increase shared count
    globalNodesConverged++;
    if (globalConvergenceCsvReady) {
        std::ofstream gf(globalConvergenceCsvPath, std::ios::out | std::ios::app);
        if (gf.is_open()) {
            gf << simTime() << ",NODE_CONVERGED," << nodeId << "," << uniqueCount << "," << globalNodesExpectingConvergence << "," << routingFreezeUniqueCount << std::endl;
            gf.close();
        }
    }
}

// If all nodes are converged, stop routing packets across the network
void LoRaNodeApp::tryStopRoutingGlobally() {
    if (!stopRoutingWhenAllConverged) return;
    if (globalConvergedFired) return;
    if (globalNodesExpectingConvergence <= 0) return; // nothing to do
    if (globalNodesConverged < globalNodesExpectingConvergence) return;
    // Fire once
    globalConvergedFired = true;
    // Broadcast-style stop: each node will see this condition independently; we set routingPacketsDue=false locally
    routingPacketsDue = false;
    // Also log a GLOBAL_CONVERGED event
    if (globalConvergenceCsvReady) {
        std::ofstream gf(globalConvergenceCsvPath, std::ios::out | std::ios::app);
        if (gf.is_open()) {
            gf << simTime() << ",GLOBAL_CONVERGED," << nodeId << ",,"
               << globalNodesExpectingConvergence << "," << routingFreezeUniqueCount << std::endl;
            gf.close();
        }
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
            f << "simTime,src,dst,seq,ttl,viaBefore,arrivalNode" << std::endl;
        }
        f.close();
        deliveredCsvReady = true;
    } else {
        deliveredCsvReady = false;
    }
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
                 << nodeId
                 << std::endl;
    deliveredCsv.flush();
    deliveredCsv.close();

    // Mark path completion for any delivered packet
    logPathHop(packet, "DELIVERED");
}

// Initialize global path log file once (all nodes append)
void LoRaNodeApp::ensurePathLogInitialized() {
    // We want a fresh paths.csv for each simulation run. Use a static process-wide flag.
    static bool pathLogClearedThisRun = false;
    if (pathLogReady) return; // This instance already initialized its handle

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
    ss << folder << sep << "paths.csv"; // single consolidated file
    pathLogFile = ss.str();

    // If not yet cleared this run, truncate and write header once.
    if (!pathLogClearedThisRun) {
        std::ofstream f(pathLogFile, std::ios::out | std::ios::trunc);
        if (f.is_open()) {
            f << "simTime,event,packetSeq,src,dst,currentNode,ttlAfterDecr,chosenVia,nextHopType" << std::endl;
            f.close();
            pathLogClearedThisRun = true;
            pathLogReady = true;
        }
    } else {
        // Already cleared by another module instance; just mark ready (file exists with header)
        pathLogReady = true;
    }
}

// Log each hop (transmission decision) for every data packet
void LoRaNodeApp::logPathHop(const LoRaAppPacket *packet, const char *eventTag) {
    EV << "DEBUG: logPathHop called for node " << nodeId << ", event: " << eventTag << ", packet src=" << packet->getSource() << ", dst=" << packet->getDestination() << endl;
    std::cout << "DEBUG: logPathHop called for node " << nodeId << ", event: " << eventTag << ", packet src=" << packet->getSource() << ", dst=" << packet->getDestination() << std::endl;
    
    ensurePathLogInitialized();
    if (!pathLogReady) {
        EV << "DEBUG: pathLogReady is false, returning" << endl;
        std::cout << "DEBUG: pathLogReady is false, returning" << std::endl;
        return;
    }
    std::ofstream f(pathLogFile, std::ios::out | std::ios::app);
    if (!f.is_open()) return;
    const char *nhType = (packet->getVia() == BROADCAST_ADDRESS) ? "BCAST" : "UNICAST";
    f << simTime() << ","
      << eventTag << ","
      << packet->getDataInt() << "," // using dataInt as sequence identifier
      << packet->getSource() << ","
      << packet->getDestination() << ","
      << nodeId << ","
      << packet->getTtl() << ","
      << packet->getVia() << ","
      << nhType
      << std::endl;
    f.close();
}


void LoRaNodeApp::logRoutingSnapshot(const char *eventName) {
    if (!routingCsvReady) return;
    // Open file in truncate mode to reflect current snapshot
    routingCsv.open(routingCsvPath, std::ios::out | std::ios::trunc);
    if (!routingCsv.is_open()) return;

    // Write header each time
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
    // End nodes operate as sources/sinks only; they do not forward others' packets
    if (isEndNodeHost(this)) { delete msg; return; }
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
    bool newAckToForward = false;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);
    LoRaAppPacket *ackPacket = packet->dup();

    // Check for too old ACK packets with TTL <= 1
    if (packet->getTtl() <= 1) {
        bubble("This ACK packet has reached TTL expiration!");
        receivedAckPacketsToForwardExpired++;
    }
    // ACK packet has not reached its maximum TTL
    else {
        receivedAckPacketsToForwardCorrect++;

        switch (routingMetric) {
            case NO_FORWARDING:
                bubble("Discarding ACK packet as forwarding is disabled");
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
                // Check if the ACK packet has already been forwarded
                if (isPacketForwarded(packet)) {
                    bubble("This ACK packet has already been forwarded!");
                    forwardPacketsDuplicateAvoid++;
                }
                // Check if the ACK packet is buffered to be forwarded
                else if (isPacketToBeForwarded(packet)) {
                    bubble("This ACK packet is already scheduled to be forwarded!");
                    forwardPacketsDuplicateAvoid++;
                }
                // A previously-unknown ACK packet has arrived
                else {
                    bubble("Saving ACK packet to forward it later!");
                    receivedAckPacketsToForwardUnique++;

                    ackPacket->setTtl(packet->getTtl() - 1);
                    if (packetsToForwardMaxVectorSize == 0 || LoRaPacketsToForward.size() < packetsToForwardMaxVectorSize) {
                        LoRaPacketsToForward.push_back(*ackPacket);
                        // Debug instrumentation: log enqueue of a forward ACK packet
                        logPathHop(ackPacket, "ENQUEUE_ACK_FWD");
                        newAckToForward = true;
                    }
                    else {
                        forwardBufferFull++;
                    }
                }
        }
    }

    delete ackPacket;

    if (newAckToForward) {
        forwardPacketsDue = true;

        if (!selfPacket->isScheduled()) {
            simtime_t nextScheduleTime = simTime() + 10*simTimeResolution;

            if (enforceDutyCycle) {
                nextScheduleTime = std::max(nextScheduleTime.dbl(), dutyCycleEnd.dbl());
            }

            if (!(nextScheduleTime > simTime())) {
                nextScheduleTime = simTime() + 1;
            }

            scheduleAt(nextScheduleTime, selfPacket);
            forwardPacketsDue = true;
        }
    }
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
                        // Debug instrumentation: log enqueue of a forward packet (all flows)
                        logPathHop(dataPacket, "ENQUEUE_FWD");
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
        
        // Generate ACK packet back to source using routing tables
        EV << "Destination received DATA packet from " << packet->getSource() << ", generating ACK" << endl;
        sendAckPacket(packet->getSource(), packet->getDataInt());
        
        // Strict unicast: stop here, do NOT forward further
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
    
    // Log ACK reception with round-trip information
    EV << "Node " << nodeId << " received ACK from " << packet->getSource() 
       << " for data packet seq " << packet->getDataInt() 
       << " at time " << simTime() << endl;
    
    bubble("Received ACK!");
    
    // Calculate round-trip time if departure time is available
    if (packet->getDepartureTime() > 0) {
        simtime_t roundTripTime = simTime() - packet->getDepartureTime();
        EV << "Round-trip time for seq " << packet->getDataInt() 
           << ": " << roundTripTime << "s" << endl;
    }
    
    // Log ACK delivery
    logPathHop(packet, "ACK_DELIVERED");
}

bool LoRaNodeApp::handleOperationStage(LifecycleOperation *operation, int stage,
        IDoneCallback *doneCallback) {
    Enter_Method_Silent();

    throw cRuntimeError("Unsupported lifecycle operation '%s'",
            operation->getClassName());
    return true;
}


simtime_t LoRaNodeApp::sendDataPacket() {
    if (failed) return 0; // Do not send after failure
    LoRaAppPacket *dataPacket = new LoRaAppPacket("DataFrame");
    std::cout << " i am sending the packet: "  << std::endl;
    bool localData = true;
    bool transmit = false;
    simtime_t txDuration = 0;

    // Send local data packets with a configurable ownDataPriority priority over packets to forward, if there is any
    if (
            (LoRaPacketsToSend.size() > 0 && bernoulli(ownDataPriority))
            || (LoRaPacketsToSend.size() > 0 && LoRaPacketsToForward.size() == 0)) {

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
                else {
                    // Broadcast fallback when no route known
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
                else {
                    dataPacket->setVia(BROADCAST_ADDRESS);
                    if (localData)
                        broadcastDataPackets++;
                    else
                        broadcastForwardedPackets++;
                }
                break;
        }

        // Log hop decision for all flows
        logPathHop(dataPacket, localData ? "TX_SRC" : "TX_FWD");

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
    if (failed) return 0; // Do not send after failure
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
                        if (forwardPacket->getMsgType() == DATA) {
                            forwardedDataPackets++;
                        } else if (forwardPacket->getMsgType() == ACK) {
                            forwardedAckPackets++;
                        }
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

        // Log all forwarded transmissions with specific packet type
        if (forwardPacket->getMsgType() == ACK) {
            logPathHop(forwardPacket, "TX_FWD_ACK");
        } else {
            logPathHop(forwardPacket, "TX_FWD_DATA");
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
    if (failed) return 0; // Do not send after failure
    // If global convergence (based on relays) has been reached, stop sending routing beacons from all nodes
    if (globalConvergedFired) return 0;

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

            // Ensure we advertise only end-node routes (strip others first)
            filterRoutesToEndNodes();

            // Build a unique set of destination IDs from the current routing table,
            // so we can advertise ALL known destinations (including end-node IDs like 1000/1001),
            // not only the relay index range [0..numberOfNodes-1].
            {
                std::set<int> destIds;
                for (const auto &r : singleMetricRoutingTable) {
                    if (r.id != nodeId)
                        destIds.insert(r.id);
                }

                // Count the number of best routes among known destinations
                for (int did : destIds) {
                    if (getBestRouteIndexTo(did) >= 0)
                        numberOfRoutes++;
                }

                // Make room for numberOfRoutes routes
                routingPacket->setRoutingTableArraySize(numberOfRoutes);

                // Add the best route for each known destination
                for (int did : destIds) {
                    int bestIdx = getBestRouteIndexTo(did);
                    if (bestIdx >= 0) {
                        LoRaRoute thisLoRaRoute;
                        thisLoRaRoute.setId(singleMetricRoutingTable[bestIdx].id);
                        thisLoRaRoute.setPriMetric(singleMetricRoutingTable[bestIdx].metric);
                        routingPacket->setRoutingTable(numberOfRoutes - 1, thisLoRaRoute);
                        numberOfRoutes--;
                    }
                }
            }

            break;

        case TIME_ON_AIR_HC_CAD_SF:
        case TIME_ON_AIR_SF_CAD_SF:
            transmit = true;

            // Ensure we advertise only end-node routes (strip others first)
            filterRoutesToEndNodes();

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

simtime_t LoRaNodeApp::sendAckPacket(int destinationNode, int originalDataSeq) {
    if (failed) return 0; // Do not send after failure

    LoRaAppPacket *ackPacket = new LoRaAppPacket("ACKFrame");
    simtime_t txDuration = 0;

    bubble("Sending ACK packet!");
    
    // Set up ACK packet properties
    ackPacket->setMsgType(ACK);
    ackPacket->setSource(nodeId);
    ackPacket->setDestination(destinationNode);
    ackPacket->setDataInt(originalDataSeq);  // Include original packet sequence for tracking
    ackPacket->setTtl(packetTTL);           // Use same TTL as data packets
    ackPacket->setByteLength(11);           // Small ACK packet size
    ackPacket->setDepartureTime(simTime());

    // Name packet for tracking
    std::string fullName = "ACK-";
    fullName += std::to_string(nodeId);
    fullName += "-to-";
    fullName += std::to_string(destinationNode);
    fullName += "-seq-";
    fullName += std::to_string(originalDataSeq);
    ackPacket->setName(fullName.c_str());

    // Add LoRa control info
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);

    // Sanitize routing table before route lookup
    sanitizeRoutingTable();

    // Find route to destination using routing tables
    int routeIndex = getBestRouteIndexTo(destinationNode);

    switch (routingMetric) {
        case FLOODING_BROADCAST_SINGLE_SF:
            ackPacket->setVia(BROADCAST_ADDRESS);
            broadcastDataPackets++;
            break;
        case SMART_BROADCAST_SINGLE_SF:
        case HOP_COUNT_SINGLE_SF:
        case RSSI_SUM_SINGLE_SF:
        case RSSI_PROD_SINGLE_SF:
        case ETX_SINGLE_SF:
            if (routeIndex >= 0) {
                ackPacket->setVia(singleMetricRoutingTable[routeIndex].via);
                EV << "ACK routed to " << destinationNode << " via " << singleMetricRoutingTable[routeIndex].via << endl;
            } else {
                // Fallback to broadcast if no route known
                ackPacket->setVia(BROADCAST_ADDRESS);
                broadcastDataPackets++;
                EV << "No route to " << destinationNode << " for ACK, using broadcast fallback" << endl;
            }
            break;
        case TIME_ON_AIR_HC_CAD_SF:
        case TIME_ON_AIR_SF_CAD_SF:
            if (routeIndex >= 0) {
                ackPacket->setVia(dualMetricRoutingTable[routeIndex].via);
                cInfo->setLoRaSF(dualMetricRoutingTable[routeIndex].sf);
                EV << "ACK routed to " << destinationNode << " via " << dualMetricRoutingTable[routeIndex].via << endl;
            } else {
                ackPacket->setVia(BROADCAST_ADDRESS);
                broadcastDataPackets++;
                EV << "No route to " << destinationNode << " for ACK, using broadcast fallback" << endl;
            }
            break;
        default:
            ackPacket->setVia(BROADCAST_ADDRESS);
            break;
    }

    // Log ACK transmission
    logPathHop(ackPacket, "TX_ACK");

    ackPacket->setControlInfo(cInfo);
    txDuration = calculateTransmissionDuration(ackPacket);

    // Update statistics
    sentPackets++;
    sentAckPackets++;
    allTxPacketsSFStats.collect(loRaSF);

    // Send the ACK packet
    send(ackPacket, "appOut");
    
    EV << "Sent ACK from " << nodeId << " to " << destinationNode << " for data seq " << originalDataSeq << endl;
    
    return txDuration;
}

void LoRaNodeApp::generateDataPackets() {
    if (failed) return; // Do not generate after failure

    EV << "DEBUG: generateDataPackets() called for node " << nodeId << " (originalIndex=" << originalNodeIndex << ")" << endl;
    std::cout << "DEBUG: generateDataPackets() called for node " << nodeId << " (originalIndex=" << originalNodeIndex << ")" << std::endl;

    if (!onlyNode0SendsPackets || originalNodeIndex == 0) {
        EV << "DEBUG: Packet generation condition met for node " << nodeId << endl;
        std::cout << "DEBUG: Packet generation condition met for node " << nodeId << std::endl;
        
        std::vector<int> destinations = { };
        // If configured, force this node to send only to a specific destination (supports any node ID, including 1000/1001)
        bool forceSingleDestination = par("forceSingleDestination");
        int forcedDestinationId = par("forcedDestinationId");
        if (forceSingleDestination && forcedDestinationId >= 0 && forcedDestinationId != nodeId) {
            destinations.push_back(forcedDestinationId);
            EV << "DEBUG: Using forced destination " << forcedDestinationId << " for node " << nodeId << endl;
            std::cout << "DEBUG: Using forced destination " << forcedDestinationId << " for node " << nodeId << std::endl;
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

                EV << "DEBUG: Created packet from " << nodeId << " to " << destinations[j] << " (seq=" << dataPacket->getDataInt() << ")" << endl;
                std::cout << "DEBUG: Created packet from " << nodeId << " to " << destinations[j] << " (seq=" << dataPacket->getDataInt() << ")" << std::endl;

                switch (routingMetric) {
    //            case 0:
    //                dataPacket->setTtl(1);
    //                break;
                default:
                    dataPacket->setTtl(packetTTL);
                    break;
                }

                LoRaPacketsToSend.push_back(*dataPacket);
                EV << "DEBUG: Added packet to send queue, queue size now: " << LoRaPacketsToSend.size() << endl;
                std::cout << "DEBUG: Added packet to send queue, queue size now: " << LoRaPacketsToSend.size() << std::endl;
                delete dataPacket;
            }
        }
    } else {
        EV << "DEBUG: Packet generation condition NOT met for node " << nodeId << " (onlyNode0SendsPackets=" << onlyNode0SendsPackets << ", originalIndex=" << originalNodeIndex << ")" << endl;
        std::cout << "DEBUG: Packet generation condition NOT met for node " << nodeId << " (onlyNode0SendsPackets=" << onlyNode0SendsPackets << ", originalIndex=" << originalNodeIndex << ")" << std::endl;
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

// ---------------------------------------------------------------
// Helper: Remove any routing entries that do NOT correspond to end
// nodes. End nodes have been offset to IDs >= 1000. We optionally
// detect an upper bound if parameter 'numberOfEndNodes' exists.
// ---------------------------------------------------------------
void LoRaNodeApp::filterRoutesToEndNodes() {
    // Skip if already frozen AND table only contains end nodes (fast path)
    bool allEnd = true;
    for (const auto &r : singleMetricRoutingTable) {
        if (r.id < 1000) { allEnd = false; break; }
    }
    if (allEnd && routingFrozen) return;

    int endCount = -1;
    if (hasPar("numberOfEndNodes")) {
        try { endCount = par("numberOfEndNodes"); } catch (...) { endCount = -1; }
    }
    int endMin = 1000;
    int endMax = (endCount > 0) ? (endMin + endCount - 1) : INT_MAX;

    // Single metric filtering
    if (!singleMetricRoutingTable.empty()) {
        std::vector<singleMetricRoute> filtered;
        filtered.reserve(singleMetricRoutingTable.size());
        for (const auto &r : singleMetricRoutingTable) {
            if (r.id >= endMin && r.id <= endMax) {
                filtered.push_back(r);
            }
        }
        singleMetricRoutingTable.swap(filtered);
    }

    // Dual metric filtering
    if (!dualMetricRoutingTable.empty()) {
        std::vector<dualMetricRoute> filtered2;
        filtered2.reserve(dualMetricRoutingTable.size());
        for (const auto &r : dualMetricRoutingTable) {
            if (r.id >= endMin && r.id <= endMax) {
                filtered2.push_back(r);
            }
        }
        dualMetricRoutingTable.swap(filtered2);
    }
}

void LoRaNodeApp::sanitizeRoutingTable() {
    // When frozen, keep tables intact
    if (routingFrozen) return;
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

// ---------------- Failure handling & export helpers ----------------

void LoRaNodeApp::scheduleFailure() {
    // Apply jitter if configured
    double base = timeToFailureParam.dbl();
    if (base < 0) return;
    double jitterFrac = std::max(0.0, failureJitterFracParam);
    double jitterPortion = 0.0;
    if (jitterFrac > 0) {
        jitterPortion = uniform(-jitterFrac, jitterFrac) * base;
    }
    double scheduleDelay = std::max(0.0, base + jitterPortion);
    failureEvent = new cMessage("failureEvent");
    scheduleAt(simTime() + scheduleDelay, failureEvent);
}

void LoRaNodeApp::performFailure() {
    if (failed) return; // idempotent
    failed = true;
    failureTime = simTime();
    // Cancel any future transmissions
    if (selfPacket && selfPacket->isScheduled()) cancelEvent(selfPacket);
    // Release failureEvent (processed)
    if (failureEvent) {
        delete failureEvent;
        failureEvent = nullptr;
    }
    bubble("Node FAILED (simulated random failure)");
    // Record immediate scalars if not already
    recordScalar("failed", 1);
    recordScalar("failureTime", failureTime);
    // Visual indication in GUI (if running Qtenv)
    cModule *parentNode = getParentModule();
    if (parentNode) {
        try {
            cDisplayString &ds = parentNode->getDisplayString();
            // Add semi-transparent red background halo and tooltip
            ds.setTagArg("b", 0, "30");
            ds.setTagArg("b", 1, "#FF000080");
            ds.setTagArg("tt", 0, (std::string("FAILED at ")+failureTime.str()).c_str());
            // Attempt to tint icon
            ds.setTagArg("i", 1, "#ff0000");
        } catch(...) {}
    }
}

void LoRaNodeApp::exportRoutingTables() {
    // Ensure directory exists (reuse logic similar to openRoutingCsv)
#if 0
    // Legacy unconditional export removed; now guarded at call site and here we provide
    // an internal guard as a safety net. (Disabled block retained for reference.)
#endif
    if (!(hasPar("exportDetailedRoutingTables") && par("exportDetailedRoutingTables").boolValue())) {
        return; // Skip generating extra routing table artifacts
    }
#ifdef _WIN32
    const char sep = '\\';
#else
    const char sep = '/';
#endif
    std::string folder = std::string("routing_tables");
#ifdef _WIN32
    _mkdir(folder.c_str());
#else
    mkdir(folder.c_str(), 0775);
#endif

    // CSV (single metric)
    {
        std::stringstream fcsv;
        fcsv << folder << sep << "node" << nodeId << "_single.csv";
        std::ofstream ofs(fcsv.str(), std::ios::out | std::ios::trunc);
        if (ofs.is_open()) {
            ofs << "id,via,metric,validUntil" << std::endl;
            for (auto &r : singleMetricRoutingTable) {
                ofs << r.id << ',' << r.via << ',' << r.metric << ',' << r.valid << std::endl;
            }
        }
    }
    // CSV (dual metric)
    {
        std::stringstream fcsv2;
        fcsv2 << folder << sep << "node" << nodeId << "_dual.csv";
        std::ofstream ofs2(fcsv2.str(), std::ios::out | std::ios::trunc);
        if (ofs2.is_open()) {
            ofs2 << "id,via,sf,priMetric,secMetric,validUntil" << std::endl;
            for (auto &r : dualMetricRoutingTable) {
                ofs2 << r.id << ',' << r.via << ',' << r.sf << ',' << r.priMetric << ',' << r.secMetric << ',' << r.valid << std::endl;
            }
        }
    }
    // TXT human readable
    {
        std::stringstream ftxt;
        ftxt << folder << sep << "node" << nodeId << "_routing_table.txt";
        std::ofstream txt(ftxt.str(), std::ios::out | std::ios::trunc);
        if (txt.is_open()) {
            txt << "Node " << nodeId << " Routing Table (simTime=" << simTime() << ")\n";
            txt << "Single-metric entries: " << singleMetricRoutingTable.size() << "\n";
            for (auto &r : singleMetricRoutingTable) {
                txt << " dest=" << r.id << " via=" << r.via << " metric=" << r.metric << " validUntil=" << r.valid << "\n";
            }
            txt << "Dual-metric entries: " << dualMetricRoutingTable.size() << "\n";
            for (auto &r : dualMetricRoutingTable) {
                txt << " dest=" << r.id << " via=" << r.via << " sf=" << r.sf << " pri=" << r.priMetric << " sec=" << r.secMetric << " validUntil=" << r.valid << "\n";
            }
            if (failed) {
                txt << "Node failed at: " << failureTime << "\n";
            }
        }
    }
}

// -------- Global failure selection static members --------
bool LoRaNodeApp::globalFailureInitialized = false;
std::vector<int> LoRaNodeApp::globalFailingNodes = {};
int LoRaNodeApp::globalFailureSubsetCountParam = -1;
double LoRaNodeApp::globalFailureStartTimeParam = -1; // seconds
double LoRaNodeApp::globalFailureExpMeanParam = 0;    // seconds mean
int LoRaNodeApp::globalTotalNodesObserved = 0;

// -------- Global routing convergence static members --------
int LoRaNodeApp::globalNodesExpectingConvergence = 0;
int LoRaNodeApp::globalNodesConverged = 0;
bool LoRaNodeApp::globalConvergedFired = false;
std::string LoRaNodeApp::globalConvergenceCsvPath = std::string();
bool LoRaNodeApp::globalConvergenceCsvReady = false;

void LoRaNodeApp::initGlobalFailureSelection() {
    // Read parameters (each instance sees same values); perform selection once
    int subsetCount = par("globalFailureSubsetCount");
    simtime_t startTime = par("globalFailureStartTime");
    simtime_t expMean = par("globalFailureExpMean");
    if (subsetCount <= 0) {
        // Nothing to do
        globalFailureSubsetCountParam = -1;
        return;
    }
    // Track max nodes observed (for late module creation scenarios)
    if (nodeId + 1 > globalTotalNodesObserved)
        globalTotalNodesObserved = nodeId + 1;
    if (!globalFailureInitialized) {
        globalFailureSubsetCountParam = subsetCount;
    globalFailureStartTimeParam = (startTime >= SIMTIME_ZERO) ? startTime.dbl() : 0.0;
    globalFailureExpMeanParam = expMean.dbl();
        // Build list of all node indices we have now; assume numberOfNodes covers relay nodes
        int total = par("numberOfNodes");
        if (total <= 0) total = globalTotalNodesObserved; // fallback
        std::vector<int> all;
        all.reserve(total);
        for (int i = 0; i < total; ++i) all.push_back(i);
        // Shuffle and take first subsetCount (cMersenneTwister via intrand?)
        for (int i = 0; i < (int)all.size(); ++i) {
            int j = intuniform(i, (int)all.size()-1);
            std::swap(all[i], all[j]);
        }
        if (subsetCount > total) subsetCount = total;
        globalFailingNodes.assign(all.begin(), all.begin()+subsetCount);
        globalFailureInitialized = true;
        EV_INFO << "[GlobalFailure] Selected " << subsetCount << " failing nodes out of " << total 
                << ". StartOffset=" << globalFailureStartTimeParam << "s"
                << " expMean=" << globalFailureExpMeanParam << "s Nodes=";
        for (size_t k=0;k<globalFailingNodes.size();++k) EV_INFO << globalFailingNodes[k] << (k+1<globalFailingNodes.size()?",":"");
        EV_INFO << endl;
    }
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

