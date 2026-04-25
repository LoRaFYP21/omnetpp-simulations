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
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <set>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif

#include "LoRaNodeApp.h"
#include "inet/common/FSMA.h"
#include "../LoRa/LoRaMac.h"
#include "../aodv/AodvLite_m.h"


#include "inet/mobility/static/StationaryMobility.h"
#include "inet/mobility/contract/IMobility.h"
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
    return false;
}

Define_Module (LoRaNodeApp);

bool LoRaNodeApp::globalFailureSelectionDone = false;
std::set<int> LoRaNodeApp::globalFailureSelectedRelayIndices;
bool LoRaNodeApp::failureLogClearedForRun = false;
bool LoRaNodeApp::anyNodeFailuresLogged = false;

bool LoRaNodeApp::isRelayNodeModule() const {
    cModule *hostMod = getContainingNode(const_cast<LoRaNodeApp*>(this));
    if (!hostMod) return false;
    const char *n = hostMod->getName();
    return n && strncmp(n, "loRaNodes", 8) == 0;
}

bool LoRaNodeApp::isFailureLogMaster() const {
    if (!isRelayNodeModule()) return false;
    cModule *hostMod = getContainingNode(const_cast<LoRaNodeApp*>(this));
    return hostMod && hostMod->getIndex() == 0;
}

const char* LoRaNodeApp::getNodeRole() const {
    if (isRescueNodeHost(const_cast<LoRaNodeApp*>(this))) return "rescue";
    if (isEndNodeHost(const_cast<LoRaNodeApp*>(this))) return "end";
    return "relay";
}

void LoRaNodeApp::ensureDeliveredPacketsDir() const {
#ifdef _WIN32
    _mkdir("simulations");
    _mkdir("simulations/delivered_packets");
#else
    mkdir("simulations", 0777);
    mkdir("simulations/delivered_packets", 0777);
#endif
}

void LoRaNodeApp::clearFailureLogForRun() const {
    if (failureLogClearedForRun) return;
    ensureDeliveredPacketsDir();
    std::ofstream out("simulations/delivered_packets/node_failures.txt", std::ios::trunc);
    if (!out.is_open()) return;
    out << "simTime,nodeId,role" << std::endl;
    out.close();
    failureLogClearedForRun = true;
    anyNodeFailuresLogged = false;
}

void LoRaNodeApp::appendNodeFailure(simtime_t when) const {
    if (!failureLogClearedForRun) {
        clearFailureLogForRun();
    }
    std::ofstream out("simulations/delivered_packets/node_failures.txt", std::ios::app);
    if (!out.is_open()) return;
    out << std::setprecision(13) << when.dbl() << "," << nodeId << "," << getNodeRole() << std::endl;
    out.close();
    anyNodeFailuresLogged = true;
}

void LoRaNodeApp::finalizeFailureLogIfNone() const {
    if (!failureLogClearedForRun) {
        clearFailureLogForRun();
    }
    if (!anyNodeFailuresLogged) {
        std::ofstream out("simulations/delivered_packets/node_failures.txt", std::ios::app);
        if (!out.is_open()) return;
        out << "no node failures" << std::endl;
        out.close();
    }
}

void LoRaNodeApp::initGlobalFailureSelection() {
    if (globalFailureSelectionDone) return;

    // Only relay nodes participate in coordinated subset failure selection.
    if (!isRelayNodeModule()) {
        return;
    }

    int candidateCount = par("numberOfNodes");
    int subsetCount = globalFailureSubsetCount;
    if (candidateCount <= 0 || subsetCount <= 0) {
        globalFailureSelectionDone = true;
        return;
    }
    if (subsetCount > candidateCount) subsetCount = candidateCount;

    std::vector<int> indices;
    indices.reserve(candidateCount);
    for (int i = 0; i < candidateCount; ++i) indices.push_back(i);

    // Fisher-Yates shuffle using OMNeT++ RNG
    for (int i = candidateCount - 1; i > 0; --i) {
        int j = intrand(i + 1);
        std::swap(indices[i], indices[j]);
    }

    globalFailureSelectedRelayIndices.clear();
    for (int i = 0; i < subsetCount; ++i) {
        globalFailureSelectedRelayIndices.insert(indices[i]);
    }
    globalFailureSelectionDone = true;
}

bool LoRaNodeApp::isSelectedForGlobalFailure() const {
    if (!isRelayNodeModule()) return false;
    cModule *hostMod = getContainingNode(const_cast<LoRaNodeApp*>(this));
    if (!hostMod) return false;
    return globalFailureSelectedRelayIndices.count(hostMod->getIndex()) != 0;
}

void LoRaNodeApp::scheduleFailure() {
    if (nodeFailed) return;
    if (!failureTimer) {
        failureTimer = new cMessage("failureTimer");
    }
    if (failureTimer->isScheduled()) cancelEvent(failureTimer);

    simtime_t scheduledTime = -1;

    // Coordinated subset mode (typically enabled only on **.loRaNodes[*])
    if (globalFailureSubsetCount > 0) {
        if (globalFailureStartTime < SIMTIME_ZERO) return;
        initGlobalFailureSelection();
        if (!isSelectedForGlobalFailure()) return;

        if (globalFailureEndTime > globalFailureStartTime) {
            scheduledTime = uniform(globalFailureStartTime, globalFailureEndTime);
        } else if (globalFailureExpMean > SIMTIME_ZERO) {
            scheduledTime = globalFailureStartTime + exponential(globalFailureExpMean);
        } else {
            scheduledTime = globalFailureStartTime;
        }
    }
    // Per-node mode
    else if (timeToFailure >= SIMTIME_ZERO) {
        if (failureJitterFrac > 0) {
            double lo = std::max(0.0, 1.0 - failureJitterFrac);
            double hi = 1.0 + failureJitterFrac;
            scheduledTime = timeToFailure * uniform(lo, hi);
        } else {
            scheduledTime = timeToFailure;
        }
    }

    if (scheduledTime >= SIMTIME_ZERO) {
        scheduleAt(scheduledTime, failureTimer);
    }
}

void LoRaNodeApp::performFailure() {
    if (nodeFailed) return;
    nodeFailed = true;

    // GUI-only: tint the containing node red in Qtenv
    if (hasGUI()) {
        if (cModule *hostMod = getContainingNode(this)) {
            cDisplayString& disp = hostMod->getDisplayString();
            // i[1]=icon tint color, i[2]=icon tint percent
            disp.setTagArg("i", 1, "red");
            disp.setTagArg("i", 2, "60");
        }
    }

    // Cancel own periodic/self scheduling
    if (selfPacket && selfPacket->isScheduled()) cancelEvent(selfPacket);
    if (configureLoRaParameters && configureLoRaParameters->isScheduled()) cancelEvent(configureLoRaParameters);

    routingPacketsDue = false;
    dataPacketsDue = false;
    forwardPacketsDue = false;
    aodvPacketsDue = false;

    // Stop any queued transmissions/forwards
    LoRaPacketsToSend.clear();
    LoRaPacketsToForward.clear();

    // Cancel AODV retry timers
    for (auto &kv : aodvRetryTimers) {
        if (kv.second) {
            if (kv.second->isScheduled()) cancelEvent(kv.second);
            delete kv.second;
        }
    }
    aodvRetryTimers.clear();
    aodvRetryCount.clear();
    aodvDiscoveryInProgress.clear();

    // Cancel RREP-ACK pending timers
    for (auto &kv : aodvPendingRrepAcks) {
        if (kv.second.timer) {
            if (kv.second.timer->isScheduled()) cancelEvent(kv.second.timer);
            delete kv.second.timer;
            kv.second.timer = nullptr;
        }
    }
    aodvPendingRrepAcks.clear();

    appendNodeFailure(simTime());
    bubble("NODE FAILED");
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
        // Reset shared/static state once per simulation run.
        // OMNeT++ can execute multiple runs per process; without this,
        // the selected relay subset and the failure log state could leak across runs.
        static cSimulation *lastSim = nullptr;
        if (getSimulation() != lastSim) {
            globalFailureSelectionDone = false;
            globalFailureSelectedRelayIndices.clear();
            failureLogClearedForRun = false;
            anyNodeFailuresLogged = false;
            lastSim = getSimulation();
        }

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
            
            EV << "DEBUG_INIT_LOCAL_V2: Starting node ID assignment for index " << nodeId << endl;
            EV << "DEBUG_INIT_LOCAL_V2: hostMod = " << (hostMod ? hostMod->getFullPath() : "NULL") << endl;
            
            // Check iAmRescue parameter first
            if (hostMod && hostMod->hasPar("iAmRescue")) {
                isRescue = (bool)hostMod->par("iAmRescue");
                EV << "DEBUG_INIT_LOCAL_V2: iAmRescue parameter found = " << isRescue << endl;
            } else {
                EV << "DEBUG_INIT_LOCAL_V2: iAmRescue parameter NOT found" << endl;
            }
            
            // Check iAmEnd parameter
            if (hostMod && hostMod->hasPar("iAmEnd")) {
                isEnd = (bool)hostMod->par("iAmEnd");
                EV << "DEBUG_INIT_LOCAL_V2: iAmEnd parameter found = " << isEnd << endl;
            } else {
                EV << "DEBUG_INIT_LOCAL_V2: iAmEnd parameter NOT found" << endl;
            }
            
            // Fallback: detect by parent module name if parameters not set
            if (!isRescue && !isEnd) {
                cModule *parent = getParentModule();
                if (parent) {
                    const char *baseName = parent->getName();
                    EV << "DEBUG_INIT_LOCAL_V2: Using fallback, parent module name = " << baseName << endl;
                    if (strcmp(baseName, "loRaEndNodes") == 0) isEnd = true;
                    if (strcmp(baseName, "loRaRescueNodes") == 0) isRescue = true;
                }
            }

            EV << "DEBUG_INIT_LOCAL_V2: Final decision - isRescue=" << isRescue << ", isEnd=" << isEnd << ", routingMetric=" << routingMetric << endl;
            
            // Rescue nodes take priority over end nodes for ID offset
            if (isRescue) {
                nodeId += 2000;
                EV << "INIT_LOCAL: Rescue node detected, nodeId set to " << nodeId << " (index=" << originalNodeIndex << ")" << endl;
            } else if (isEnd && routingMetric != 0) {
                nodeId += 1000;
                EV << "INIT_LOCAL: End node detected, nodeId set to " << nodeId << " (index=" << originalNodeIndex << ")" << endl;
            } else {
                EV << "INIT_LOCAL: Relay node, nodeId = " << nodeId << endl;
            }
        }
        
        std::pair<double, double> coordsValues = std::make_pair(-1, -1);
        cModule *host = getContainingNode(this);

        // Generate random location for nodes if circle deployment type
        // Updated to work with any mobility model (not just StationaryMobility)
        if (strcmp(host->par("deploymentType").stringValue(), "circle") == 0) {
            coordsValues = generateUniformCircleCoordinates(
                    host->par("maxGatewayDistance").doubleValue(),
                    host->par("gatewayX").doubleValue(),
                    host->par("gatewayY").doubleValue());
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
                double newX, newY;
                if (nodeId == 0 && routingMetric == 0){ // end node 0 at middle
                    newX = minX + sepX * (cols/2);
                    newY = minY + sepY * (cols/2) + uniform(0,100);
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
            bool end = host->par("iAmEnd");
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
    // Optional: force a single destination per node (primarily for node 0 in demos)
    forceSingleDestination = par("forceSingleDestination");
    forcedDestinationId = par("forcedDestinationId");
    aodvOnly = par("aodvOnly");

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

        // AODV retry configuration
        aodvRreqBackoff = par("aodvRreqBackoff");
        aodvRreqMaxRetries = par("aodvRreqMaxRetries");
        aodvRrepJitter = par("aodvRrepJitter");
        
        // RREP-ACK configuration (RFC3561) - optional, default disabled
        aodvEnableRrepAck = par("aodvEnableRrepAck").boolValue();
        if (aodvEnableRrepAck) {
            aodvMaxRrepRetries = par("aodvMaxRrepRetries").intValue();
            aodvRrepAckTimeout = par("aodvRrepAckTimeout").doubleValue();
            aodvBlacklistTimeout = par("aodvBlacklistTimeout").doubleValue();
            EV << "RREP-ACK enabled: maxRetries=" << aodvMaxRrepRetries 
               << " timeout=" << aodvRrepAckTimeout << " blacklist=" << aodvBlacklistTimeout << endl;
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

        //Node identifier (re-assign and apply rescue/end offset consistently)
        nodeId = getContainingNode(this)->getIndex();
        originalNodeIndex = nodeId;  // Store original index
        {
            cModule *hostMod = getContainingNode(this);
            bool isRescue = false;
            bool isEnd = false;
            
            EV << "DEBUG_INIT_APP_V2: Starting node ID assignment for index " << nodeId << endl;
            EV << "DEBUG_INIT_APP_V2: hostMod = " << (hostMod ? hostMod->getFullPath() : "NULL") << endl;
            
            // Check iAmRescue parameter first
            if (hostMod && hostMod->hasPar("iAmRescue")) {
                isRescue = (bool)hostMod->par("iAmRescue");
                EV << "DEBUG_INIT_APP_V2: iAmRescue parameter found = " << isRescue << endl;
            } else {
                EV << "DEBUG_INIT_APP_V2: iAmRescue parameter NOT found" << endl;
            }
            
            // Check iAmEnd parameter
            if (hostMod && hostMod->hasPar("iAmEnd")) {
                isEnd = (bool)hostMod->par("iAmEnd");
                EV << "DEBUG_INIT_APP_V2: iAmEnd parameter found = " << isEnd << endl;
            } else {
                EV << "DEBUG_INIT_APP_V2: iAmEnd parameter NOT found" << endl;
            }
            
            // Fallback: detect by parent module name if parameters not set
            if (!isRescue && !isEnd) {
                cModule *parent = getParentModule();
                if (parent) {
                    const char *baseName = parent->getName();
                    EV << "DEBUG_INIT_APP_V2: Using fallback, parent module name = " << baseName << endl;
                    if (strcmp(baseName, "loRaEndNodes") == 0) isEnd = true;
                    if (strcmp(baseName, "loRaRescueNodes") == 0) isRescue = true;
                }
            }

            EV << "DEBUG_INIT_APP_V2: Final decision - isRescue=" << isRescue << ", isEnd=" << isEnd << ", routingMetric=" << routingMetric << endl;
            
            // Rescue nodes take priority over end nodes for ID offset
            if (isRescue) {
                nodeId += 2000;
                EV << "INIT_APP: Rescue node confirmed, nodeId = " << nodeId << " (index=" << originalNodeIndex << ")" << endl;
            } else if (isEnd && routingMetric != 0) {
                nodeId += 1000;
                EV << "INIT_APP: End node confirmed, nodeId = " << nodeId << " (index=" << originalNodeIndex << ")" << endl;
            } else {
                EV << "INIT_APP: Relay node, nodeId = " << nodeId << endl;
            }
        }

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
        
        // Rescue nodes are destination-only: they don't generate data packets, only receive and respond
        bool iAmRescue = isRescueNodeHost(this);
        if (iAmRescue) {
            EV << "Node " << nodeId << " is a RESCUE NODE - will not generate data or routing packets" << endl;
        }
        
        if (!iAmRescue) {
            generateDataPackets();
        }

        // Routing packets timer: suppress if AODV-only mode is enabled OR if this is a rescue node (destination-only)
        if (!aodvOnly && !iAmRescue) {
            timeToFirstRoutingPacket = math::max(5, par("timeToFirstRoutingPacket"))+getTimeToNextRoutingPacket();
            switch (routingMetric) {
                case NO_FORWARDING:
                case FLOODING_BROADCAST_SINGLE_SF:
                case SMART_BROADCAST_SINGLE_SF:
                    break;
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
                scheduleSelfAt(simTime() + timeToFirstDataPacket);
                EV << "Self packet triggered by due data packet" << endl;
            }
            // Only forward packet due
            else if (routingPacketsDue && !dataPacketsDue && !forwardPacketsDue) {
                scheduleSelfAt(simTime() + timeToFirstRoutingPacket);
                EV << "Self packet triggered by due routing packet" << endl;
            }
            // Only routing packet due
            else if (forwardPacketsDue && !dataPacketsDue && !routingPacketsDue) {
                scheduleSelfAt(simTime() + timeToFirstForwardPacket);
                EV << "Self packet triggered by due forward packet" << endl;
            }
            // Data packet due earlier
            else if (timeToFirstDataPacket < timeToFirstForwardPacket && timeToFirstDataPacket < timeToFirstRoutingPacket ) {
                scheduleSelfAt(simTime() + timeToFirstDataPacket);
                EV << "Self packet triggered by due data packet before other due packets" << endl;
            }
            // Forward packet due earlier
                else if (timeToFirstForwardPacket < timeToFirstDataPacket && timeToFirstForwardPacket < timeToFirstRoutingPacket ) {
                scheduleSelfAt(simTime() + timeToFirstForwardPacket);
                EV << "Self packet triggered by due forward packet before other due packets" << endl;
            }
            else {
                scheduleSelfAt(simTime() + timeToFirstRoutingPacket);
                EV << "Self packet triggered by due routing packet before other due packets" << endl;
            }
        }

        dutyCycleEnd = simTime();

        // --- Node failure initialization ---
        nodeFailed = false;
        timeToFailure = par("timeToFailure");
        failureJitterFrac = par("failureJitterFrac");
        globalFailureSubsetCount = par("globalFailureSubsetCount");
        globalFailureStartTime = par("globalFailureStartTime");
        globalFailureEndTime = par("globalFailureEndTime");
        globalFailureExpMean = par("globalFailureExpMean");

        if (isFailureLogMaster()) {
            clearFailureLogForRun();
        }
        scheduleFailure();
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
        if (auto mobIface = dynamic_cast<inet::IMobility *>(mobMod)) {
            coord = mobIface->getCurrentPosition();
        } else if (mobMod->hasPar("initialX") && mobMod->hasPar("initialY")) {
            coord = Coord(mobMod->par("initialX").doubleValue(), mobMod->par("initialY").doubleValue(), 0);
        }
        recordScalar("positionX", coord.x);
        recordScalar("positionY", coord.y);
    }
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

    if (isFailureLogMaster()) {
        finalizeFailureLogIfNone();
    }

    dataPacketsForMeLatency.recordAs("dataPacketsForMeLatency");
    dataPacketsForMeUniqueLatency.recordAs("dataPacketsForMeUniqueLatency");
}

void LoRaNodeApp::handleMessage(cMessage *msg) {

    if (msg->isSelfMessage()) {
        if (msg == failureTimer) {
            performFailure();
            return;
        }

        if (nodeFailed) {
            // Ignore other timers after failure. Only delete dynamically allocated timer messages.
            if (msg != selfPacket && msg != configureLoRaParameters) {
                delete msg;
            }
            return;
        }

        // Dedicated AODV retry timers use a name prefix
        const char* n = msg->getName();
        if (n && strncmp(n, "AODV_RETRY_", 11) == 0) {
            handleAodvRetryTimer(msg);
            return;
        }
        // RREP-ACK timeout timers
        // (timer name created by scheduleRrepAckTimer)
        if (n && strcmp(n, "RREP-ACK-TIMEOUT") == 0) {
            handleRrepAckTimer(msg);
            return;
        }
        // Backward/alternate naming (keep in case older runs/scripts used it)
        if (n && strncmp(n, "AODV_RREPACK_TIMEOUT_", 21) == 0) {
            handleRrepAckTimer(msg);
            return;
        }
        handleSelfMessage(msg);
    } else {
        handleMessageFromLowerLayer(msg);
    }
}


void LoRaNodeApp::handleSelfMessage(cMessage *msg) {

    if (nodeFailed) {
        return;
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
    bool sendAodv = false;

        // Check if there are data packets to send, and if it is time to send them
        // TODO: Not using dataPacketsDue ???
        if ( LoRaPacketsToSend.size() > 0 && simTime() >= nextDataPacketTransmissionTime ) {
            sendData = true;
            EV << "AODV_DEBUG: Data ready to send at node " << nodeId << ", simTime=" << simTime() << ", nextDataTime=" << nextDataPacketTransmissionTime << ", queueSize=" << LoRaPacketsToSend.size() << endl;
        } else if (LoRaPacketsToSend.size() > 0) {
            EV << "AODV_DEBUG: Data waiting at node " << nodeId << ", simTime=" << simTime() << ", nextDataTime=" << nextDataPacketTransmissionTime << " (blocked until " << (nextDataPacketTransmissionTime - simTime()) << "s)" << endl;
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
        // Check if there are AODV packets to send
        if ( aodvPacketsDue && simTime() >= nextAodvPacketTransmissionTime ) {
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

        // Send AODV control packet
        else if (sendAodv) {
            txDuration = sendAodvPacket();
            if (enforceDutyCycle) {
                dutyCycleEnd = simTime() + txDuration/dutyCycle;
                nextAodvPacketTransmissionTime = simTime() + math::max(SIMTIME_ZERO.dbl(), txDuration.dbl()/dutyCycle);
            } else {
                nextAodvPacketTransmissionTime = simTime() + txDuration;
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
        // then based on AODV packets, if earlier
        if (aodvPacketsDue) {
            nextScheduleTime = std::min(nextScheduleTime.dbl(), nextAodvPacketTransmissionTime.dbl());
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
            scheduleSelfAt(nextScheduleTime + 10*simTimeResolution);
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
    scheduleSelfAt(simTime() + 0.00002);
    }
}

void LoRaNodeApp::handleMessageFromLowerLayer(cMessage *msg) {
    if (nodeFailed) {
        delete msg;
        return;
    }
    receivedPackets++;

    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);

    // Handle encapsulated AODV control messages first (separate packet types)
    if (cPacket *inner = packet->getEncapsulatedPacket()) {
        if (auto rreq = dynamic_cast<aodv::Rreq*>(inner)) {
            handleAodvPacket(packet); // envelope-aware handler
            delete msg;
            return;
        }
        if (auto rrep = dynamic_cast<aodv::Rrep*>(inner)) {
            handleAodvPacket(packet); // envelope-aware handler
            delete msg;
            return;
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

        // Log per-node processing and destination reception
        logDataProcessing(packet, "deliver");
        logDataReception(packet);

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
        // which we may forward
        // Log that this node is processing a data packet it has received (potential forwarder)
        logDataProcessing(packet, "recv");

        // If it is being broadcast and we are in route discovery mode
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
                        newNeighbour.dstSeq = 0; newNeighbour.dstSeqValid = false;
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

                    singleMetricRoutingTable.push_back(newNeighbour);
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

                            singleMetricRoutingTable.push_back(newRoute);
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
                    newNeighbour.dstSeq = 0; newNeighbour.dstSeqValid = false;
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
                    newNeighbour.dstSeq = 0; newNeighbour.dstSeqValid = false;
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
    }

    EV << "## Routing table at node " << nodeId << "##" << endl;
    for (int i=0; i<singleMetricRoutingTable.size(); i++) {
        EV << "Node " << singleMetricRoutingTable[i].id << " via " << singleMetricRoutingTable[i].via << " with cost " << singleMetricRoutingTable[i].metric << endl;
    }
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

    // AODV PATH ENFORCEMENT: If this node is NOT on the locked reverse path for (source->destination), suppress forwarding.
    // Condition applies only after source has locked a next hop (aodvLockedNextHop at source) and data is unicast (via != BROADCAST)
    // We detect path membership by ensuring we have an installed route back to the source OR we are the locked next hop itself.
    int flowSrc = packet->getSource();
    int flowDst = packet->getDestination();
    bool enforcePath = aodvOnly; // in AODV-only mode, enforce strict path usage
    if (enforcePath) {
        // If I'm neither the locked first hop (next after source) nor have a valid route entry toward flowDst nor am I the destination, drop.
        bool isDestination = (nodeId == flowDst);
        bool isLockedFirstHop = (aodvLockedNextHop.count(flowDst) && nodeId == aodvLockedNextHop[flowDst]);
        bool haveRouteToDst = (getBestRouteIndexTo(flowDst) >= 0);
        if (!isDestination && !isLockedFirstHop && !haveRouteToDst) {
            EV << "AODV: Suppress off-path forwarding in AODV-only mode for dst=" << flowDst << " at node " << nodeId << endl;
            delete dataPacket;
            return; // Entirely suppress off-path replication
        }
    }

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
                        // AODV-only: do not keep broadcast duplicates; convert to unicast via known next hop or drop
                        if (aodvOnly && dataPacket->getVia() == BROADCAST_ADDRESS) {
                            int idx = getBestRouteIndexTo(flowDst);
                            if (idx >= 0) {
                                dataPacket->setVia(singleMetricRoutingTable[idx].via);
                            } else {
                                // No known next hop: drop this broadcast copy in strict AODV-only mode
                                delete dataPacket;
                                break;
                            }
                        }
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

    // Initialize hopTrace with this transmitter
    dataPacket->setHopTraceArraySize(1);
    dataPacket->setHopTrace(0, nodeId);
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

                    // Carry over hopTrace and append this transmitter
                    int old = LoRaPacketsToForward.front().getHopTraceArraySize();
                    dataPacket->setHopTraceArraySize(old+1);
                    for (int i=0;i<old;i++) dataPacket->setHopTrace(i, LoRaPacketsToForward.front().getHopTrace(i));
                    dataPacket->setHopTrace(old, nodeId);
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

        // If table-based routing and no route exists, trigger AODV discovery and buffer this DATA
        bool tableBased = (routingMetric == HOP_COUNT_SINGLE_SF || routingMetric == RSSI_SUM_SINGLE_SF || routingMetric == RSSI_PROD_SINGLE_SF || routingMetric == ETX_SINGLE_SF || routingMetric == TIME_ON_AIR_HC_CAD_SF || routingMetric == TIME_ON_AIR_SF_CAD_SF);
        if (tableBased && routeIndex < 0) {
            EV << "AODV_DEBUG: No route to dest " << dataPacket->getDestination() << " at node " << nodeId << ", buffering data at simTime=" << simTime() << endl;
            maybeStartDiscoveryFor(dataPacket->getDestination());
            aodvBufferedData[dataPacket->getDestination()].push_back(*dataPacket);
            // Do not send this DATA now
            delete dataPacket;
            // Generate more packets if needed
            if (sendPacketsContinuously && LoRaPacketsToSend.size() == 0) {
                generateDataPackets();
            }
            return 0;
        }

        // If we have a locked next hop (from RREP arrival) and it hasn't expired, override route selection to keep same path
        int destIdForLock = dataPacket->getDestination();
        if (aodvLockedNextHop.count(destIdForLock)) {
            if (simTime() <= aodvLockedNextHopExpiry[destIdForLock]) {
                // Force the via to locked next hop
                dataPacket->setVia(aodvLockedNextHop[destIdForLock]);
                // Bypass switching based on metric; treat as unicast
            } else {
                // Expired lock, remove
                aodvLockedNextHop.erase(destIdForLock);
                aodvLockedNextHopExpiry.erase(destIdForLock);
            }
        }

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
                    // Only set via if not already forced by lock
                    if (!aodvLockedNextHop.count(dataPacket->getDestination()) || dataPacket->getVia() == BROADCAST_ADDRESS) {
                        dataPacket->setVia(singleMetricRoutingTable[routeIndex].via);
                    }
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
                    if (!aodvLockedNextHop.count(dataPacket->getDestination()) || dataPacket->getVia() == BROADCAST_ADDRESS) {
                        dataPacket->setVia(dualMetricRoutingTable[routeIndex].via);
                    }
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


        // Log transmission if this is a local data packet from this source
        if (localData && dataPacket->getMsgType() == DATA) {
            logDataTransmission(dataPacket);
        }

        // Log per-node processing for any DATA packet we are about to transmit
        if (dataPacket->getMsgType() == DATA) {
            logDataProcessing(dataPacket, localData ? "send" : "fwd-tx");
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

// AODV: wrapper that dispatches RREQ/RREP
// Helpers for AODV destination sequence comparison (handle wrap if needed)
static inline bool seqNewer(uint32_t a, uint32_t b) {
    return (int32_t)(a - b) > 0; // true if a is newer than b
}
static inline bool seqOlder(uint32_t a, uint32_t b) {
    return (int32_t)(a - b) < 0; // true if a is older than b
}

void LoRaNodeApp::handleAodvPacket(cMessage *msg) {
    // envelope contains AODV inner packet (aodv::Rreq or aodv::Rrep or aodv::RrepAck)
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);
    cPacket *inner = packet->getEncapsulatedPacket();
    
    // Handle RREP-ACK messages
    if (auto rrepAck = dynamic_cast<aodv::RrepAck*>(inner)) {
        handleRrepAck(msg);
        return;
    }
    
    if (auto rreq = dynamic_cast<aodv::Rreq*>(inner)) {
        // Check if sender is blacklisted (unidirectional link)
        if (aodvEnableRrepAck && isNodeBlacklisted(packet->getLastHop())) {
            EV << "AODV: Drop RREQ from blacklisted node " << packet->getLastHop() 
               << " at node " << nodeId << endl;
            bubble("Drop RREQ from blacklisted");
            return;
        }
        
        // Duplicate suppression based on (src, bcastId)
        long long key = (static_cast<long long>(rreq->getSrcId()) << 32) | (static_cast<unsigned int>(rreq->getBcastId()));
        if (aodvSeenRreqs.count(key)) {
            EV << "AODV: Drop duplicate RREQ src=" << rreq->getSrcId() << " bcastId=" << rreq->getBcastId() << " at node " << nodeId << endl;
            bubble("Drop duplicate RREQ");
            return; // drop duplicate
        }
    aodvSeenRreqs.insert(key);
    // Track path hop at this node
    int psz = rreq->getPathArraySize();
    rreq->setPathArraySize(psz+1);
    rreq->setPath(psz, nodeId);

    // Install/refresh reverse route to RREQ origin via last hop
        int src = rreq->getSrcId();
    int via = packet->getLastHop();
        int idx = getRouteIndexInSingleMetricRoutingTable(src, via);
        if (idx < 0) {
            singleMetricRoute nr; nr.id = src; nr.via = via; nr.metric = 1; nr.valid = simTime() + routeTimeout;
            nr.dstSeq = rreq->getSrcSeq(); nr.dstSeqValid = true; // reverse route stores origin's seq
            nr.hopCount = rreq->getHopCount();
            singleMetricRoutingTable.push_back(nr);
        } else {
            // Discard rule: do not update with older seq
            if (!singleMetricRoutingTable[idx].dstSeqValid || seqNewer((uint32_t)rreq->getSrcSeq(), (uint32_t)singleMetricRoutingTable[idx].dstSeq)) {
                singleMetricRoutingTable[idx].via = via;
                singleMetricRoutingTable[idx].dstSeq = rreq->getSrcSeq();
                singleMetricRoutingTable[idx].dstSeqValid = true;
                singleMetricRoutingTable[idx].valid = simTime() + routeTimeout;
                singleMetricRoutingTable[idx].hopCount = rreq->getHopCount();
            } else if (rreq->getSrcSeq() == singleMetricRoutingTable[idx].dstSeq) {
                // equal seq: refresh lifetime only
                singleMetricRoutingTable[idx].valid = simTime() + routeTimeout;
            } else {
                // older: ignore update (no table change)
            }
        }
        // Log snapshot after reverse route update
        logRoutingTableSnapshot("aodv_rreq_processed");

        // Log RREQ processing at this node (destination or intermediate)
        logRreqAtDestination(rreq);

        // Track first-seen parent per-origin for this discovery wave
        auto bIt = aodvReverseBcastId.find(src);
        if (bIt == aodvReverseBcastId.end() || bIt->second < rreq->getBcastId()) {
            // New discovery wave, reset parent
            aodvReverseBcastId[src] = rreq->getBcastId();
            aodvReverseParent[src] = via;
        } else if (bIt->second == rreq->getBcastId()) {
            // Same wave: keep only the first parent; ignore competing first-hops
            if (aodvReverseParent.find(src) == aodvReverseParent.end()) {
                aodvReverseParent[src] = via;
            }
        }

        // Debug: Log RREQ destination check (especially important for rescue nodes)
        EV << "DEBUG_RREQ_V2: ========== RREQ DESTINATION CHECK ==========" << endl;
        EV << "DEBUG_RREQ_V2: RREQ dstId = " << rreq->getDstId() << endl;
        EV << "DEBUG_RREQ_V2: My nodeId = " << nodeId << endl;
        EV << "DEBUG_RREQ_V2: Match? " << (rreq->getDstId() == nodeId ? "YES" : "NO") << endl;
        EV << "AODV RREQ: Node " << nodeId << " checking if destination (RREQ dst=" << rreq->getDstId() << ", my nodeId=" << nodeId << ")" << endl;
        if (isRescueNodeHost(this)) {
            EV << "  -> This is a RESCUE NODE with nodeId=" << nodeId << endl;
        }

        if (rreq->getDstId() == nodeId) {
            EV << "DEBUG_RREQ_V2: *** I AM THE DESTINATION *** Generating RREP..." << endl;
            // I'm the destination: send RREP back to source
            int parent = aodvReverseParent.count(src) ? aodvReverseParent[src] : via;
            EV << "AODV: DEST received first RREQ from src=" << src << ", replying via parent=" << parent << endl;
            bubble("RREP via first-seen parent");
            aodv::Rrep *innerRrep = new aodv::Rrep("RREP");
            innerRrep->setSrcId(src);           // originator
            innerRrep->setDstId(nodeId);        // destination (me)
            // Per AODV: before sending RREP, ensure own seq >= requested dstSeq
            if (rreq->getDstSeq() > aodvSeq) aodvSeq = rreq->getDstSeq();
            innerRrep->setDstSeq(aodvSeq);
            innerRrep->setHopCount(0);
            innerRrep->setLifetime(routeTimeout.dbl());
            innerRrep->setByteLength(12);
            innerRrep->setPathArraySize(1);
            innerRrep->setPath(0, nodeId);
            
            // RFC3561 RREP-ACK: Set A-bit when enabled (for unidirectional link detection)
            if (aodvEnableRrepAck) {
                innerRrep->setAckRequired(true);
                EV << "AODV: Setting RREP A-bit for next hop " << parent << endl;
            }

            // Immediately log a 'create' event so rrep_path.csv exists even if forwarding later fails
            // originSrc=src, finalDst=nodeId, envelopeDest=src, envelopeVia=parent, nextHop=parent, hopCount=0
            logRrepEvent("create", src, nodeId, src, parent, parent, innerRrep->getHopCount(), innerRrep);

            LoRaAppPacket rrepEnv("AODV-RREP");
            rrepEnv.setMsgType(ROUTING);
            rrepEnv.setSource(nodeId);
            rrepEnv.setDestination(src);
            rrepEnv.setVia(parent); // strictly via recorded first parent
            rrepEnv.setLastHop(nodeId);
            rrepEnv.setTtl(packetTTL);
            rrepEnv.encapsulate(innerRrep);
            
            // RFC3561 RREP-ACK: Schedule timeout if A-bit is set
            if (aodvEnableRrepAck && innerRrep->getAckRequired()) {
                scheduleRrepAckTimer(parent, src, nodeId, &rrepEnv);
                EV << "AODV: Scheduled RREP-ACK timer for next hop " << parent << " (destination)" << endl;
            }
            
            aodvPacketsToSend.push_back(rrepEnv);
            // Apply RREP jitter: destination delays reply to let RREQ flood settle
            aodvPacketsDue = true; nextAodvPacketTransmissionTime = simTime() + aodvRrepJitter;
        } else {
            EV << "DEBUG_RREQ_V2: *** NOT the destination *** Will forward RREQ as intermediate node" << endl;
            // Intermediate: DO NOT send RREP; only destination replies
            // Forward RREQ if TTL allows
            if (packet->getTtl() > 1) {
                LoRaAppPacket fwd = *packet; // copy envelope and its encapsulated AODV
                fwd.setTtl(packet->getTtl()-1);
                fwd.setLastHop(nodeId);
                fwd.setVia(BROADCAST_ADDRESS);
                // update inner hop count and ttl
                if (auto fwdInner = dynamic_cast<aodv::Rreq*>(fwd.getEncapsulatedPacket())) {
                    fwdInner->setHopCount(fwdInner->getHopCount()+1);
                    fwdInner->setTtl(std::max(0, fwdInner->getTtl()-1));
                }
                EV << "AODV: Forward RREQ (ttl=" << fwd.getTtl() << ") from node " << nodeId << endl;
                aodvPacketsToSend.push_back(fwd);
                aodvPacketsDue = true; nextAodvPacketTransmissionTime = simTime();
            }
        }
        if (aodvPacketsDue || dataPacketsDue || forwardPacketsDue || routingPacketsDue) {
            scheduleSelfAt(simTime());
        }
    }
    else if (auto rrep = dynamic_cast<aodv::Rrep*>(inner)) {
        // Only the intended next hop should process/forward this RREP.
        // Accept if I'm the final destination (original source of RREQ), otherwise require via==nodeId.
        if (packet->getDestination() != nodeId && packet->getVia() != nodeId) {
            EV << "AODV: Drop RREP not for me (via=" << packet->getVia() << ", dest=" << packet->getDestination() << ") at node " << nodeId << endl;
            bubble("Drop RREP not for me");
            return; // not for me to forward
        }
        
        // RFC3561 RREP-ACK: Send acknowledgment if A-bit is set
        if (aodvEnableRrepAck && rrep->getAckRequired()) {
            int sender = packet->getLastHop();
            sendRrepAck(sender);
            EV << "AODV: Sent RREP-ACK to " << sender << " (A-bit was set)" << endl;
        }
        
    // Log RREP reception event
        logRrepEvent("recv", rrep->getSrcId(), rrep->getDstId(), packet->getDestination(), packet->getVia(), -1, rrep->getHopCount(), rrep);
    int rps = rrep->getPathArraySize();
    rrep->setPathArraySize(rps+1);
    rrep->setPath(rps, nodeId);
    // Install/refresh forward route to destination (RREP destination) via last hop
        int dest = rrep->getDstId(); // final destination of the path
    int via = packet->getLastHop();
        int idx = getRouteIndexInSingleMetricRoutingTable(dest, via);
        if (idx < 0) {
            singleMetricRoute nr; nr.id = dest; nr.via = via; nr.metric = 1; nr.valid = simTime() + routeTimeout;
            nr.dstSeq = rrep->getDstSeq(); nr.dstSeqValid = true;
            nr.hopCount = rrep->getHopCount();
            singleMetricRoutingTable.push_back(nr);
        } else {
            if (!singleMetricRoutingTable[idx].dstSeqValid || seqNewer((uint32_t)rrep->getDstSeq(), (uint32_t)singleMetricRoutingTable[idx].dstSeq)) {
                singleMetricRoutingTable[idx].via = via;
                singleMetricRoutingTable[idx].dstSeq = rrep->getDstSeq();
                singleMetricRoutingTable[idx].dstSeqValid = true;
                singleMetricRoutingTable[idx].valid = simTime() + routeTimeout;
                singleMetricRoutingTable[idx].hopCount = rrep->getHopCount();
            } else if (rrep->getDstSeq() == singleMetricRoutingTable[idx].dstSeq) {
                singleMetricRoutingTable[idx].valid = simTime() + routeTimeout;
            } else {
                // older seq: ignore
            }
        }
        // Log snapshot after forward route update
        logRoutingTableSnapshot("aodv_rrep_processed");

        if (packet->getDestination() == nodeId) {
            // RREP reached original source: flush buffered data
            logRrepFinalPath(rrep);
            // Cancel any pending retry timer and clear discovery state for this destination
            auto tmIt = aodvRetryTimers.find(dest);
            if (tmIt != aodvRetryTimers.end()) {
                if (tmIt->second && tmIt->second->isScheduled()) cancelEvent(tmIt->second);
                delete tmIt->second;
                aodvRetryTimers.erase(tmIt);
            }
            aodvRetryCount.erase(dest);
            aodvDiscoveryInProgress.erase(dest);
            auto it = aodvBufferedData.find(dest);
            if (it != aodvBufferedData.end()) {
                // Lock the first-hop next hop toward destination based on the reverse route just installed
                int lockedIdx = getBestRouteIndexTo(dest);
                if (lockedIdx >= 0) {
                    aodvLockedNextHop[dest] = singleMetricRoutingTable[lockedIdx].via;
                    aodvLockedNextHopExpiry[dest] = simTime() + routeTimeout; // expire with route timeout
                    EV << "AODV: Lock next hop toward dest=" << dest << " via=" << aodvLockedNextHop[dest] << " at source node " << nodeId << endl;
                }
                int bufferedCount = it->second.size();
                EV << "AODV_DEBUG: RREP final at node " << nodeId << " releasing " << bufferedCount << " buffered packets to dest " << dest << " at simTime=" << simTime() << endl;
                for (auto &dp : it->second) {
                    // Force via to locked next hop if available (preserve path of RREP)
                    if (aodvLockedNextHop.count(dest)) {
                        dp.setVia(aodvLockedNextHop[dest]);
                    }
                    LoRaPacketsToSend.push_back(dp);
                }
                aodvBufferedData.erase(it);
                dataPacketsDue = true;
                // Reset data packet transmission time to allow immediate sending
                nextDataPacketTransmissionTime = simTime();
                EV << "AODV_DEBUG: Reset nextDataPacketTransmissionTime to " << nextDataPacketTransmissionTime << " for immediate transmission" << endl;
            }
        } else if (packet->getTtl() > 1) {
            // Forward RREP towards original source strictly along reverse path
            LoRaAppPacket fwd = *packet;
            fwd.setTtl(packet->getTtl()-1);
            fwd.setLastHop(nodeId);
            // choose reverse next hop from routing table (installed on RREQ)
            int revIdx = getBestRouteIndexTo(fwd.getDestination());
            if (revIdx >= 0) {
                fwd.setVia(singleMetricRoutingTable[revIdx].via);
                // update inner hop count
                if (auto fwdInner = dynamic_cast<aodv::Rrep*>(fwd.getEncapsulatedPacket())) {
                    fwdInner->setHopCount(fwdInner->getHopCount()+1);
                }
                EV << "AODV: Forward RREP unicast to via=" << fwd.getVia() << " at node " << nodeId << endl;
                bubble("Fwd RREP unicast");
                int hopc = 0; 
                aodv::Rrep* fi = dynamic_cast<aodv::Rrep*>(fwd.getEncapsulatedPacket());
                if (fi) hopc = fi->getHopCount();
                logRrepEvent("fwd", rrep->getSrcId(), rrep->getDstId(), fwd.getDestination(), packet->getVia(), fwd.getVia(), hopc, fi);
                
                // RFC3561 RREP-ACK: Schedule timeout if A-bit is set
                if (aodvEnableRrepAck && rrep->getAckRequired()) {
                    scheduleRrepAckTimer(fwd.getVia(), rrep->getSrcId(), rrep->getDstId(), &fwd);
                    EV << "AODV: Scheduled RREP-ACK timer for next hop " << fwd.getVia() << endl;
                }
                
                aodvPacketsToSend.push_back(fwd);
                aodvPacketsDue = true; nextAodvPacketTransmissionTime = simTime();
            } else {
                // No reverse route known: drop instead of broadcasting
                EV << "AODV: Drop RREP (no reverse route) at node " << nodeId << endl;
                bubble("Drop RREP (no reverse route)");
            }
        }

        if (aodvPacketsDue || dataPacketsDue || forwardPacketsDue || routingPacketsDue) {
            scheduleSelfAt(simTime());
        }
    }
}
// Log the full RREP path once it reaches the original source (this node)
void LoRaNodeApp::logRrepFinalPath(const aodv::Rrep* rrep) {
    if (!rrep) return;
    // Use unified logRrepEvent with "final" event type
    logRrepEvent("final", rrep->getSrcId(), rrep->getDstId(), -1, -1, -1, rrep->getHopCount(), rrep);
}

// Log AODV RREQ processing at any node (destination or intermediate)
void LoRaNodeApp::logRreqAtDestination(const aodv::Rreq* rreq) {
    static bool rreqFileCleared = false;
    try {
        ensurePathsDir();
        char path[512];
        sprintf(path, "results/paths/rreq_path.csv");
        bool writeHeader = !rreqFileCleared;
        std::ofstream out(path, rreqFileCleared ? std::ios::app : std::ios::trunc);
        if (!out.is_open()) return;
        rreqFileCleared = true;
        if (writeHeader) {
            out << "simTime,atNode,srcId,dstId,bcastId,hopCount,srcSeq,path" << std::endl;
        }
        std::string p=""; int ps = rreq->getPathArraySize();
        for (int i=0;i<ps;i++){ if(i) p+="-"; p+=std::to_string(rreq->getPath(i)); }
        out << simTime() << "," << nodeId << "," << rreq->getSrcId() << "," << rreq->getDstId() << "," << rreq->getBcastId() << "," << rreq->getHopCount() << "," << rreq->getSrcSeq() << "," << p << std::endl;
        out.close();
    } catch (...) {
        // Ignore logging failures
    }
}

// Send one queued AODV control packet
simtime_t LoRaNodeApp::sendAodvPacket() {
    simtime_t txDuration = 0;
    if (aodvPacketsToSend.empty()) { aodvPacketsDue = false; return 0; }

    LoRaAppPacket pkt = aodvPacketsToSend.front();
    aodvPacketsToSend.erase(aodvPacketsToSend.begin());

    LoRaAppPacket *out = new LoRaAppPacket(pkt);

    // Control info
    LoRaMacControlInfo *cInfo = new LoRaMacControlInfo;
    cInfo->setLoRaTP(loRaTP);
    cInfo->setLoRaCF(loRaCF);
    cInfo->setLoRaSF(loRaSF);
    cInfo->setLoRaBW(loRaBW);
    cInfo->setLoRaCR(loRaCR);
    out->setControlInfo(cInfo);

    // Route according to inner type
    if (cPacket *inner = out->getEncapsulatedPacket()) {
        if (dynamic_cast<aodv::Rreq*>(inner)) {
            out->setVia(BROADCAST_ADDRESS);
            out->setLastHop(nodeId);
        } else if (dynamic_cast<aodv::Rrep*>(inner)) {
            // Preserve via if already set; otherwise pick reverse path; never broadcast RREPs
            if (out->getVia() == BROADCAST_ADDRESS) {
                int idx = getBestRouteIndexTo(out->getDestination());
                if (idx >= 0 && idx < (int)singleMetricRoutingTable.size()) {
                    out->setVia(singleMetricRoutingTable[idx].via);
                } else {
                    // No known reverse path; abort sending this control packet
                    delete out;
                    aodvPacketsDue = !aodvPacketsToSend.empty();
                    return 0;
                }
            }
            out->setLastHop(nodeId);
        }
    }

    out->setDepartureTime(simTime());
    out->setByteLength(std::max(16, dataPacketSize/2));
    txDuration = calculateTransmissionDuration(out);
    send(out, "appOut");
    aodvPacketsDue = !aodvPacketsToSend.empty();
    if (aodvPacketsDue) nextAodvPacketTransmissionTime = simTime() + txDuration;
    return txDuration;
}

// Unified logger for RREP events (reception / forwarding)
void LoRaNodeApp::logRrepEvent(const char *eventType, int originSrc, int finalDst, int envelopeDest, int envelopeVia, int nextHop, int hopCount, const aodv::Rrep* rrep) {
    static bool rrepFileCleared = false;
    try {
        ensurePathsDir();
        char path[512];
        sprintf(path, "results/paths/rrep_path.csv");
        bool writeHeader = !rrepFileCleared;
        std::ofstream out(path, rrepFileCleared ? std::ios::app : std::ios::trunc);
        if (!out.is_open()) {
            EV << "AODV: Failed to open RREP forwarders log file: " << path << endl;
            return;
        }
        rrepFileCleared = true;
        if (writeHeader) {
            out << "simTime,event,nodeId,originSrc,finalDst,envelopeDest,envelopeVia,nextHop,hopCount,path" << std::endl;
        }
        std::string p="";
        if (rrep) {
            int ps = rrep->getPathArraySize();
            for (int i=0;i<ps;i++){ if(i) p+="-"; p+=std::to_string(rrep->getPath(i)); }
        }
        out << simTime() << "," << eventType << "," << nodeId << "," << originSrc << "," << finalDst << "," << envelopeDest << "," << envelopeVia << "," << nextHop << "," << hopCount << "," << p << std::endl;
        out.close();
    } catch (...) {
        // swallow
    }
}

void LoRaNodeApp::scheduleAodvRetry(int destination) {
    // Cancel and delete any existing timer for this destination
    auto it = aodvRetryTimers.find(destination);
    if (it != aodvRetryTimers.end()) {
        if (it->second && it->second->isScheduled()) cancelEvent(it->second);
        delete it->second;
        aodvRetryTimers.erase(it);
    }
    // Create and schedule a new timer message
    char name[64];
    sprintf(name, "AODV_RETRY_%d", destination);
    cMessage *m = new cMessage(name);
    aodvRetryTimers[destination] = m;
    scheduleAt(simTime() + aodvRreqBackoff, m);
}

void LoRaNodeApp::handleAodvRetryTimer(cMessage *msg) {
    // Extract destination from the message name suffix
    int destination = -1;
    const char* n = msg->getName();
    if (n && strlen(n) > 11) {
        destination = atoi(n + 11);
    }
    // Remove from timers and delete message
    for (auto it = aodvRetryTimers.begin(); it != aodvRetryTimers.end(); ++it) {
        if (it->second == msg) {
            aodvRetryTimers.erase(it);
            break;
        }
    }
    delete msg;

    if (destination < 0) return;

    // If discovery completed or next hop locked, stop
    if (!aodvDiscoveryInProgress.count(destination) || aodvLockedNextHop.count(destination)) {
        aodvDiscoveryInProgress.erase(destination);
        aodvRetryCount.erase(destination);
        return;
    }

    // Retry if under max retries
    int count = 0;
    if (aodvRetryCount.count(destination)) count = aodvRetryCount[destination];
    if (count >= aodvRreqMaxRetries) {
        aodvDiscoveryInProgress.erase(destination);
        aodvRetryCount.erase(destination);
        return;
    }
    aodvRetryCount[destination] = count + 1;

    // Build a new RREQ
    aodv::Rreq *innerRreq = new aodv::Rreq("RREQ");
    // Per AODV: increment own seq before originating discovery
    innerRreq->setSrcId(nodeId);
    innerRreq->setDstId(destination);
    innerRreq->setSrcSeq(++aodvSeq);
    // Set requested destination seq if known
    {
        int reqSeq = 0; bool known = false;
        // find any route entry to destination with a known dstSeq
        for (auto &r : singleMetricRoutingTable) {
            if (r.id == destination && r.dstSeqValid) { reqSeq = r.dstSeq; known = true; break; }
        }
        innerRreq->setDstSeq(known ? reqSeq : 0);
    }
    innerRreq->setHopCount(0);
    innerRreq->setBcastId(++aodvRreqId);
    innerRreq->setTtl(packetTTL);
    innerRreq->setByteLength(12);
    innerRreq->setPathArraySize(1);
    innerRreq->setPath(0, nodeId);

    LoRaAppPacket rreqEnv("AODV-RREQ");
    rreqEnv.setMsgType(ROUTING);
    rreqEnv.setSource(nodeId);
    rreqEnv.setDestination(BROADCAST_ADDRESS);
    rreqEnv.setVia(BROADCAST_ADDRESS);
    rreqEnv.setLastHop(nodeId);
    rreqEnv.setTtl(packetTTL);
    rreqEnv.encapsulate(innerRreq);

    aodvPacketsToSend.push_back(rreqEnv);
    aodvPacketsDue = true;
    nextAodvPacketTransmissionTime = simTime();
    scheduleSelfAt(simTime());

    // Schedule next retry
    scheduleAodvRetry(destination);
}

void LoRaNodeApp::maybeStartDiscoveryFor(int destination) {
    if (aodvDiscoveryInProgress.count(destination)) return;
    aodvDiscoveryInProgress.insert(destination);
    aodvRetryCount[destination] = 0;

    // Build inner AODV RREQ
    aodv::Rreq *innerRreq = new aodv::Rreq("RREQ");
    innerRreq->setSrcId(nodeId);
    innerRreq->setDstId(destination);
    innerRreq->setSrcSeq(++aodvSeq);
    {
        int reqSeq = 0; bool known = false;
        for (auto &r : singleMetricRoutingTable) {
            if (r.id == destination && r.dstSeqValid) { reqSeq = r.dstSeq; known = true; break; }
        }
        innerRreq->setDstSeq(known ? reqSeq : 0);
    }
    innerRreq->setHopCount(0);
    innerRreq->setBcastId(++aodvRreqId);
    innerRreq->setTtl(packetTTL);
    innerRreq->setByteLength(12);
    // Seed path with originator
    innerRreq->setPathArraySize(1);
    innerRreq->setPath(0, nodeId);

    // Build envelope LoRa packet
    LoRaAppPacket rreqEnv("AODV-RREQ");
    rreqEnv.setMsgType(ROUTING);
    rreqEnv.setSource(nodeId);
    rreqEnv.setDestination(BROADCAST_ADDRESS); // broadcast envelope
    rreqEnv.setVia(BROADCAST_ADDRESS);
    rreqEnv.setLastHop(nodeId);
    rreqEnv.setTtl(packetTTL);
    rreqEnv.encapsulate(innerRreq);

    aodvPacketsToSend.push_back(rreqEnv);
    aodvPacketsDue = true; nextAodvPacketTransmissionTime = simTime();
    scheduleSelfAt(simTime());

    // Schedule first retry timer
    scheduleAodvRetry(destination);
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

                    // Carry over hopTrace and append this transmitter
                    int old = LoRaPacketsToForward.front().getHopTraceArraySize();
                    forwardPacket->setHopTraceArraySize(old+1);
                    for (int i=0;i<old;i++) forwardPacket->setHopTrace(i, LoRaPacketsToForward.front().getHopTrace(i));
                    forwardPacket->setHopTrace(old, nodeId);
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

        // AODV PATH ENFORCEMENT for forwarding (generalized): in AODV-only mode, avoid broadcast and require a unicast next hop
        int fpSrc = forwardPacket->getSource();
        int fpDst = forwardPacket->getDestination();

        switch (routingMetric) {
            case FLOODING_BROADCAST_SINGLE_SF:
                if (aodvOnly) {
                    if (routeIndex >= 0) {
                        forwardPacket->setVia(singleMetricRoutingTable[routeIndex].via);
                    } else {
                        EV << "AODV: Drop forward packet (no route in AODV-only flooding mode) at node " << nodeId << endl;
                        delete forwardPacket; forwardPacket = nullptr; transmit = false; goto after_forward_send_logic;
                    }
                } else {
                    forwardPacket->setVia(BROADCAST_ADDRESS);
                    broadcastForwardedPackets++;
                }
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
                    if (aodvOnly) {
                        EV << "AODV: Drop forward packet (no route to enforce unicast) at node " << nodeId << endl;
                        delete forwardPacket; forwardPacket = nullptr; transmit = false; goto after_forward_send_logic;
                    } else {
                        forwardPacket->setVia(BROADCAST_ADDRESS);
                        broadcastForwardedPackets++;
                    }
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
after_forward_send_logic:
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

            // Ensure we advertise only end-node routes (strip relay routes first)
            filterRoutesToEndNodes();

            // Build a unique set of destination IDs from the current routing table
            // and ALWAYS include a self-route (id=nodeId, metric=0) so neighbors learn routes to us.
            {
                std::set<int> destIds;
                destIds.insert(nodeId); // self
                for (const auto &r : singleMetricRoutingTable) {
                    if (r.id != nodeId)
                        destIds.insert(r.id);
                }

                // Count routable destinations: self + best known routes
                numberOfRoutes = 0;
                for (int did : destIds) {
                    if (did == nodeId) {
                        numberOfRoutes++;
                    } else if (getBestRouteIndexTo(did) >= 0) {
                        numberOfRoutes++;
                    }
                }

                // Make room for numberOfRoutes routes
                routingPacket->setRoutingTableArraySize(numberOfRoutes);

                // Add routes to packet
                int idx = 0;
                for (int did : destIds) {
                    LoRaRoute thisLoRaRoute;
                    if (did == nodeId) {
                        // Self-route: I am reachable with metric 0
                        thisLoRaRoute.setId(nodeId);
                        thisLoRaRoute.setPriMetric(0);
                        routingPacket->setRoutingTable(idx++, thisLoRaRoute);
                    } else {
                        int bestIdx = getBestRouteIndexTo(did);
                        if (bestIdx >= 0) {
                            thisLoRaRoute.setId(singleMetricRoutingTable[bestIdx].id);
                            thisLoRaRoute.setPriMetric(singleMetricRoutingTable[bestIdx].metric);
                            routingPacket->setRoutingTable(idx++, thisLoRaRoute);
                        }
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

        if (numberOfDestinationsPerNode == 0 )
            numberOfDestinationsPerNode = numberOfNodes-1;

    // If configured, force a single known destination id (e.g., for node 0 demo)
    // Note: destination IDs can be in ranges: 0-999 (relay), 1000-1999 (end), 2000-2999 (rescue)
    if (forceSingleDestination && forcedDestinationId >= 0 && forcedDestinationId != nodeId) {
        destinations.push_back(forcedDestinationId);
        EV << "Node " << nodeId << " forcing destination to " << forcedDestinationId << endl;
    }

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
    for (auto it = neighbourNodes.begin(); it < neighbourNodes.end(); ++it) {
        if (*it == neighbourId) return true;
    }
    return false;
}

bool LoRaNodeApp::isRouteInSingleMetricRoutingTable(int id, int via) {
    return getRouteIndexInSingleMetricRoutingTable(id, via) >= 0;
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
            // Prefer freshest destination sequence if available
            int availableRoutesCount = end(availableRoutes) - begin(availableRoutes);
            bool anySeq = false;
            int bestRoute = 0;
            int bestSeq = 0;
            for (int j = 0; j < availableRoutesCount; j++) {
                if (availableRoutes[j].dstSeqValid) {
                    if (!anySeq || seqNewer((uint32_t)availableRoutes[j].dstSeq, (uint32_t)bestSeq)) {
                        anySeq = true;
                        bestSeq = availableRoutes[j].dstSeq;
                        bestRoute = j;
                    }
                }
            }
            if (anySeq) {
                // Among equal-best seq, choose best metric then latest validity
                for (int j = 0; j < availableRoutesCount; j++) {
                    if (availableRoutes[j].dstSeqValid && availableRoutes[j].dstSeq == bestSeq) {
                        if (availableRoutes[j].metric < availableRoutes[bestRoute].metric ||
                            (availableRoutes[j].metric == availableRoutes[bestRoute].metric && availableRoutes[j].valid > availableRoutes[bestRoute].valid)) {
                            bestRoute = j;
                        }
                    }
                }
                return getRouteIndexInSingleMetricRoutingTable(availableRoutes[bestRoute].id, availableRoutes[bestRoute].via);
            }
            // No seq info: fall back to original metric-based selection
            bestRoute = 0;
            int bestMetric = availableRoutes[0].metric;
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

void LoRaNodeApp::filterRoutesToEndNodes() {
    // Accept all destination IDs mapped to end-like nodes:
    // - Classic end nodes: 1000 .. (1000 + numberOfEndNodes - 1)
    // - Rescue end nodes: 2000 .. (2000 + numberOfRescueNodes - 1)
    // LoRaNodeApp does not declare numberOfRescueNodes; to avoid
    // excluding rescues, keep any id >= 1000. This includes both
    // classic end nodes (1000+) and rescue nodes (2000+).
    int endMin = 1000;

    // Single metric filtering
    if (!singleMetricRoutingTable.empty()) {
        std::vector<singleMetricRoute> filtered;
        filtered.reserve(singleMetricRoutingTable.size());
        for (const auto &r : singleMetricRoutingTable) {
            if (r.id >= endMin) {
                filtered.push_back(r);
            }
        }
        singleMetricRoutingTable = std::move(filtered);
    }

    // Dual metric filtering
    if (!dualMetricRoutingTable.empty()) {
        std::vector<dualMetricRoute> filtered2;
        filtered2.reserve(dualMetricRoutingTable.size());
        for (const auto &r : dualMetricRoutingTable) {
            if (r.id >= endMin) {
                filtered2.push_back(r);
            }
        }
        dualMetricRoutingTable = std::move(filtered2);
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

// Ensure results/paths directory exists for CSV outputs
void LoRaNodeApp::ensurePathsDir() {
#ifdef _WIN32
    _mkdir("results");
    _mkdir("results/paths");
#else
    struct stat st{};
    if (stat("results", &st) != 0) { mkdir("results", 0755); }
    if (stat("results/paths", &st) != 0) { mkdir("results/paths", 0755); }
#endif
}

// Ensure routing_tables directory exists for routing table CSVs (relative to current working dir)
static void ensureRoutingTablesDir() {
#ifdef _WIN32
    _mkdir("routing_tables");
#else
    struct stat st{};
    if (stat("routing_tables", &st) != 0) { mkdir("routing_tables", 0755); }
#endif
}

// Log a snapshot of the current routing table (AODV-aware fields)
void LoRaNodeApp::logRoutingTableSnapshot(const char* eventType) {
    try {
        ensureRoutingTablesDir();
        char path[256];
        sprintf(path, "routing_tables/node_%d_routing.csv", nodeId);
        bool writeHeader = false;
        {
            std::ifstream check(path);
            writeHeader = !check.good();
        }
        std::ofstream out(path, std::ios::app);
        if (!out.is_open()) return;
        if (writeHeader) {
            out << "simTime,event,nodeId,tableSize,id,dstSeq,dstSeqValid,netIf,hopCount,nextHop,validUntil" << std::endl;
        }
        int tableSize = (int)singleMetricRoutingTable.size();
        for (const auto &r : singleMetricRoutingTable) {
            out << simTime() << "," << eventType << "," << nodeId << "," << tableSize << "," << r.id << ",";
            if (r.dstSeqValid) out << r.dstSeq << ",valid"; else out << ",invalid";
            out << ",LoRa"; // network interface
            out << "," << r.hopCount;
            out << "," << r.via;
            out << "," << r.valid;
            out << std::endl;
        }
        out.close();
    } catch (...) {}
}

// Log data packet transmission at source node
void LoRaNodeApp::logDataTransmission(const LoRaAppPacket* packet) {
    if (!packet || packet->getMsgType() != DATA) return;
    if (packet->getSource() != nodeId) return; // Only log at source
    static bool txFileCleared = false;
    try {
        ensurePathsDir();
        char path[512];
        sprintf(path, "results/paths/data_tx.csv");
        bool writeHeader = !txFileCleared;
        std::ofstream out(path, txFileCleared ? std::ios::app : std::ios::trunc);
        if (!out.is_open()) return;
        txFileCleared = true;
        if (writeHeader) {
            out << "simTime,srcId,dstId,seqNum,len,via" << std::endl;
        }
        out << simTime() << "," << packet->getSource() << "," << packet->getDestination() 
            << "," << packet->getDataInt() << "," << packet->getByteLength() 
            << "," << packet->getVia() << std::endl;
        out.close();
    } catch (...) {}
}

// Log data packet reception at destination only
void LoRaNodeApp::logDataReception(const LoRaAppPacket* packet) {
    if (!packet || packet->getMsgType() != DATA) return;
    if (packet->getDestination() != nodeId) return; // Only log at destination
    static bool rxFileCleared = false;
    try {
        ensurePathsDir();
        char path[512];
        sprintf(path, "results/paths/data_path.csv");
        bool writeHeader = !rxFileCleared;
        std::ofstream out(path, rxFileCleared ? std::ios::app : std::ios::trunc);
        if (!out.is_open()) return;
        rxFileCleared = true;
        if (writeHeader) {
            out << "simTime,atNode,srcId,dstId,seqNum,len,hops,path" << std::endl;
        }
        std::string p;
        int n = packet->getHopTraceArraySize();
        for (int i = 0; i < n; ++i) {
            if (i) p += "-";
            p += std::to_string(packet->getHopTrace(i));
        }
        out << simTime() << "," << nodeId << "," << packet->getSource() << "," << packet->getDestination() 
            << "," << packet->getDataInt() << "," << packet->getByteLength() << "," << n << "," << p << std::endl;
        out.close();
    } catch (...) {}
}

// Log every node that processes a DATA packet (send, forward, deliver)
void LoRaNodeApp::logDataProcessing(const LoRaAppPacket* packet, const char *event) {
    if (!packet || packet->getMsgType() != DATA) return;
    if (!event) event = "unknown";
    static bool procFileCleared = false;
    try {
        ensurePathsDir();
        char path[512];
        sprintf(path, "results/paths/data_process.csv");
        bool writeHeader = !procFileCleared;
        std::ofstream out(path, procFileCleared ? std::ios::app : std::ios::trunc);
        if (!out.is_open()) return;
        procFileCleared = true;
        if (writeHeader) {
            out << "simTime,event,atNode,srcId,dstId,seqNum,len,via,hops,path" << std::endl;
        }
        std::string p;
        int n = packet->getHopTraceArraySize();
        for (int i = 0; i < n; ++i) {
            if (i) p += "-";
            p += std::to_string(packet->getHopTrace(i));
        }
        out << simTime() << "," << event << "," << nodeId << "," << packet->getSource() << "," << packet->getDestination()
            << "," << packet->getDataInt() << "," << packet->getByteLength() << "," << packet->getVia()
            << "," << n << "," << p << std::endl;
        out.close();
    } catch (...) {}
}

// Log final path of delivered DATA packets (kept for compatibility, now delegates to logDataReception)
void LoRaNodeApp::logDataDeliveryPath(const LoRaAppPacket* packet) {
    logDataReception(packet);
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

//=============================================================================
// RREP-ACK Implementation (RFC3561 Section 6.7)
//=============================================================================

/**
 * Check if a node is currently blacklisted (unidirectional link detected)
 */
bool LoRaNodeApp::isNodeBlacklisted(int nodeId) {
    auto it = aodvBlacklistedNodes.find(nodeId);
    if (it != aodvBlacklistedNodes.end()) {
        if (simTime() < it->second) {
            return true; // still blacklisted
        } else {
            // blacklist expired, remove
            aodvBlacklistedNodes.erase(it);
        }
    }
    return false;
}

/**
 * Blacklist a node for BLACKLIST_TIMEOUT duration
 */
void LoRaNodeApp::blacklistNode(int nodeId) {
    aodvBlacklistedNodes[nodeId] = simTime() + aodvBlacklistTimeout;
    EV << "RREP-ACK: Blacklisted node " << nodeId << " until " << aodvBlacklistedNodes[nodeId] << endl;
}

/**
 * Send RREP-ACK to acknowledge receipt of RREP
 */
void LoRaNodeApp::sendRrepAck(int viaNode) {
    if (!aodvEnableRrepAck) return;
    
    EV << "RREP-ACK: Sending ACK to node " << viaNode << " from node " << nodeId << endl;
    
    aodv::RrepAck *innerAck = new aodv::RrepAck("RREP-ACK");
    innerAck->setByteLength(2); // minimal packet
    
    LoRaAppPacket ackEnv("AODV-RREP-ACK");
    ackEnv.setMsgType(ROUTING);
    ackEnv.setSource(nodeId);
    ackEnv.setDestination(viaNode); // unicast to previous hop
    ackEnv.setVia(viaNode);
    ackEnv.setLastHop(nodeId);
    ackEnv.setTtl(1); // single hop
    ackEnv.encapsulate(innerAck);
    
    aodvPacketsToSend.push_back(ackEnv);
    aodvPacketsDue = true;
    nextAodvPacketTransmissionTime = simTime(); // send immediately
    scheduleSelfAt(simTime());
}

/**
 * Handle received RREP-ACK message
 */
void LoRaNodeApp::handleRrepAck(cMessage *msg) {
    LoRaAppPacket *packet = check_and_cast<LoRaAppPacket *>(msg);
    cPacket *inner = packet->getEncapsulatedPacket();
    
    if (auto ack = dynamic_cast<aodv::RrepAck*>(inner)) {
        int from = packet->getSource();
        EV << "RREP-ACK: Received ACK from node " << from << " at node " << nodeId << endl;
        
        // Find and cancel pending RREP-ACK timer for this node
        std::vector<std::string> toRemove;
        for (auto &pending : aodvPendingRrepAcks) {
            if (pending.second.nextHop == from) {
                EV << "RREP-ACK: Canceling pending ACK timer for key " << pending.first << endl;
                if (pending.second.timer && pending.second.timer->isScheduled()) {
                    cancelEvent(pending.second.timer);
                }
                delete pending.second.timer;
                toRemove.push_back(pending.first);
            }
        }
        for (auto &key : toRemove) {
            aodvPendingRrepAcks.erase(key);
        }
    }
}

/**
 * Schedule RREP-ACK timeout timer
 */
void LoRaNodeApp::scheduleRrepAckTimer(int nextHop, int origSrc, int finalDst, LoRaAppPacket *rrepCopy) {
    if (!aodvEnableRrepAck) return;
    
    std::string key = std::to_string(nextHop) + "_" + std::to_string(origSrc) + "_" + std::to_string(finalDst);
    
    // Cancel existing timer if any
    cancelRrepAckTimer(nextHop, origSrc, finalDst);
    
    cMessage *timer = new cMessage("RREP-ACK-TIMEOUT");
    timer->setContextPointer((void*)strdup(key.c_str())); // store key for retrieval
    scheduleAt(simTime() + aodvRrepAckTimeout, timer);
    
    auto &pending = aodvPendingRrepAcks[key];
    pending.timer = timer;
    pending.nextHop = nextHop;
    pending.origSrc = origSrc;
    pending.finalDst = finalDst;
    pending.rrepCopy = *rrepCopy;  // Store copy for retransmission
    
    EV << "RREP-ACK: Scheduled timeout for nextHop=" << nextHop << " key=" << key << endl;
}

/**
 * Cancel pending RREP-ACK timer
 */
void LoRaNodeApp::cancelRrepAckTimer(int nextHop, int origSrc, int finalDst) {
    std::string key = std::to_string(nextHop) + "_" + std::to_string(origSrc) + "_" + std::to_string(finalDst);
    
    auto it = aodvPendingRrepAcks.find(key);
    if (it != aodvPendingRrepAcks.end()) {
        if (it->second.timer && it->second.timer->isScheduled()) {
            cancelEvent(it->second.timer);
        }
        delete it->second.timer;
        aodvPendingRrepAcks.erase(it);
    }
}

/**
 * Handle RREP-ACK timeout - retransmit or blacklist
 */
void LoRaNodeApp::handleRrepAckTimer(cMessage *msg) {
    if (!aodvEnableRrepAck) {
        delete msg;
        return;
    }
    
    const char *keyPtr = (const char *)msg->getContextPointer();
    if (!keyPtr) {
        delete msg;
        return;
    }
    std::string key(keyPtr);
    free((void*)keyPtr);
    
    auto it = aodvPendingRrepAcks.find(key);
    if (it == aodvPendingRrepAcks.end()) {
        delete msg;
        return; // already handled
    }
    
    auto &pending = it->second;
    pending.retryCount++;
    
    EV << "RREP-ACK: Timeout for key=" << key 
       << " retry=" << pending.retryCount << "/" << aodvMaxRrepRetries << endl;
    
    if (pending.retryCount >= aodvMaxRrepRetries) {
        // Max retries reached: blacklist the next hop
        EV << "RREP-ACK: Max retries reached, blacklisting node " << pending.nextHop << endl;
        blacklistNode(pending.nextHop);
        
        // Remove the route through this next hop
        for (size_t i = 0; i < singleMetricRoutingTable.size(); ) {
            if (singleMetricRoutingTable[i].via == pending.nextHop) {
                EV << "RREP-ACK: Removing route to " << singleMetricRoutingTable[i].id 
                   << " via blacklisted " << pending.nextHop << endl;
                singleMetricRoutingTable.erase(singleMetricRoutingTable.begin() + i);
            } else {
                ++i;
            }
        }
        
        delete msg;
        aodvPendingRrepAcks.erase(it);
    } else {
        // Retransmit RREP
        EV << "RREP-ACK: Retransmitting RREP to " << pending.nextHop << endl;
        
        if (pending.rrepCopy.getByteLength() > 0) {
            aodvPacketsToSend.push_back(pending.rrepCopy);
            aodvPacketsDue = true;
            nextAodvPacketTransmissionTime = simTime();
        }
        
        // Reschedule timer
        scheduleAt(simTime() + aodvRrepAckTimeout, msg);
    }
}
} // namespace inet