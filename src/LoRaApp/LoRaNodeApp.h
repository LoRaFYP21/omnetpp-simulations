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

#ifndef __LORA_OMNET_LORANODEAPP_H_
#define __LORA_OMNET_LORANODEAPP_H_

#include <omnetpp.h>
#include <string>
// DSDV state containers
#include <unordered_set>
#include <unordered_map>
#include <cstdint>
// CSV logging
#include <fstream>

#include "inet/common/lifecycle/ILifecycle.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/LifecycleOperation.h"
#include "inet/common/FSMA.h"

#include "LoRaAppPacket_m.h"
#include "LoRa/LoRaMacControlInfo_m.h"

using namespace omnetpp;

namespace inet {

/**
 * TODO - Generated class
 */
class INET_API LoRaNodeApp : public cSimpleModule, public ILifecycle
{
    protected:
        // Forward declaration so we can reference the nested type in prototypes above its definition
        class singleMetricRoute;

        virtual void initialize(int stage) override;
        void finish() override;
        virtual int numInitStages() const override { return NUM_INIT_STAGES; }
        virtual void handleMessage(cMessage *msg) override;
        virtual bool handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback) override;
        virtual bool isNeighbour(int neighbourId);
        virtual bool isRouteInSingleMetricRoutingTable(int id, int via);
        virtual int  getRouteIndexInSingleMetricRoutingTable(int id, int via);
        virtual bool isRouteInDualMetricRoutingTable(int id, int via, int sf);
        virtual int  getRouteIndexInDualMetricRoutingTable(int id, int via, int sf);
        virtual bool isKnownNode(int knownNodeId);
        virtual bool isACKed(int nodeId);
        virtual bool isPacketForwarded(cMessage *msg);
        virtual bool isPacketToBeForwarded(cMessage *msg);
        virtual bool isDataPacketForMeUnique(cMessage *msg);

        void handleMessageFromLowerLayer(cMessage *msg);
        void handleSelfMessage(cMessage *msg);

        simtime_t getTimeToNextDataPacket();
        simtime_t getTimeToNextForwardPacket();
        simtime_t getTimeToNextRoutingPacket();

        simtime_t sendDataPacket();
        simtime_t sendForwardPacket();
        simtime_t sendRoutingPacket();
        simtime_t sendDSDVRoutingPacket(bool fullDump);
        simtime_t sendAckPacket(int destinationNode, int originalDataSeq);
        void manageReceivedPacketForMe(cMessage *msg);
        void manageReceivedAckPacketForMe(cMessage *msg);
        void manageReceivedDataPacketForMe(cMessage *msg);
        void manageReceivedPacketToForward(cMessage *msg);
        void manageReceivedAckPacketToForward(cMessage *msg);
        void manageReceivedDataPacketToForward(cMessage *msg);
        void manageReceivedRoutingPacket(cMessage *msg);
        std::pair<double,double> generateUniformCircleCoordinates(double radius, double gatewayX, double gatewayY);
        void sendJoinRequest();
        void sendDownMgmtPacket();
        void generateDataPackets();
        void sanitizeRoutingTable();
    void filterRoutesToEndNodes(); // keep only end-node (ID>=1000) routes
    // When enabled (storeBestRouteOnly), ensure at most one route per destination id
    void addOrReplaceBestSingleRoute(const singleMetricRoute &candidate);
        int pickCADSF();
        int getBestRouteIndexTo(int destination);
        int getSFTo(int destination);

        // Failure simulation helpers
        void scheduleFailure();
        void performFailure();
        bool failed = false;
        cMessage *failureEvent = nullptr;
        simtime_t failureTime = -1;
        double failureJitterFracParam = 0;
        simtime_t timeToFailureParam = -1;

        simtime_t calculateTransmissionDuration(cMessage *msg);


    // Routing / delivery logging helpers
    void openRoutingCsv();
    void logRoutingSnapshot(const char *eventName);
    void openDeliveredCsv();
    void logDeliveredPacket(const LoRaAppPacket *packet);
    // Path logging (per-hop) for ALL data packets (filter by destination offline)
    void logPathHop(const LoRaAppPacket *packet, const char *eventTag);
    void ensurePathLogInitialized();

    // Global failure subset (shared across instances)
    static bool globalFailureInitialized;        // whether subset was chosen
    static std::vector<int> globalFailingNodes;  // chosen node indices
    static int globalFailureSubsetCountParam;    // cached param
    static double globalFailureStartTimeParam; // cached (seconds)
    static double globalFailureExpMeanParam;   // cached mean (seconds)
    static int globalTotalNodesObserved;          // track highest node index+1 seen
    void initGlobalFailureSelection();            // choose subset if needed


    void exportRoutingTables(); // export routing tables at finish


        bool sendPacketsContinuously;
        bool onlyNode0SendsPackets;
        bool enforceDutyCycle;
        double dutyCycle;
        int numberOfDestinationsPerNode;
        int numberOfPacketsPerDestination;

        int numberOfPacketsToForward;

        int sentPackets;
        int sentDataPackets;
        int sentRoutingPackets;
        int sentAckPackets;
        int receivedPackets;
        int receivedPacketsForMe;
        int receivedPacketsFromMe;
        int receivedPacketsToForward;
        int receivedDataPackets;
        int receivedDataPacketsForMe;
        int receivedDataPacketsForMeUnique;
        int receivedDataPacketsFromMe;
        int receivedDataPacketsToForward;
        int receivedDataPacketsToForwardCorrect;
        int receivedDataPacketsToForwardExpired;
        int receivedDataPacketsToForwardUnique;
        int receivedAckPackets;
        int receivedAckPacketsForMe;
        int receivedAckPacketsFromMe;
        int receivedAckPacketsToForward;
        int receivedAckPacketsToForwardCorrect;
        int receivedAckPacketsToForwardExpired;
        int receivedAckPacketsToForwardUnique;
        int receivedRoutingPackets;
        int receivedADRCommands;
        int forwardedPackets;
        int forwardedDataPackets;
        int forwardedAckPackets;
        int forwardPacketsDuplicateAvoid;
        int broadcastDataPackets;
        int broadcastForwardedPackets;
        int lastSentMeasurement;
        int deletedRoutes;
        int forwardBufferFull;
    // Strict unicast diagnostics
    int unicastNoRouteDrops;          // packets we originated but dropped due to no route
    int unicastWrongNextHopDrops;     // packets received for which we are neither destination nor intended via
    int unicastFallbackBroadcasts;    // legacy fallback occurrences (should remain 0 after strict mode)

        simtime_t timeToFirstDataPacket;
        std::string timeToNextDataPacketDist;
        simtime_t timeToNextDataPacketMin;
        simtime_t timeToNextDataPacketMax;
        simtime_t timeToNextDataPacketAvg;

        simtime_t timeToFirstForwardPacket;
        std::string timeToNextForwardPacketDist;
        simtime_t timeToNextForwardPacketMin;
        simtime_t timeToNextForwardPacketMax;
        simtime_t timeToNextForwardPacketAvg;

        simtime_t timeToFirstRoutingPacket;
        std::string timeToNextRoutingPacketDist;
        simtime_t timeToNextRoutingPacketMin;
        simtime_t timeToNextRoutingPacketMax;
        simtime_t timeToNextRoutingPacketAvg;

        simtime_t dutyCycleEnd;

        simtime_t nextRoutingPacketTransmissionTime;
        simtime_t nextDataPacketTransmissionTime;
        simtime_t nextForwardPacketTransmissionTime;

        bool dataPacketsDue;
        bool forwardPacketsDue;
        bool routingPacketsDue;

        cHistogram allTxPacketsSFStats;
        cHistogram routingTxPacketsSFStats;
        cHistogram owndataTxPacketsSFStats;
        cHistogram fwdTxPacketsSFStats;

        cHistogram dataPacketsForMeLatency;
        cHistogram dataPacketsForMeUniqueLatency;

        cHistogram routingTableSize;

        simtime_t firstDataPacketTransmissionTime;
        simtime_t lastDataPacketTransmissionTime;
        simtime_t firstDataPacketReceptionTime;
        simtime_t lastDataPacketReceptionTime;


        simtime_t simTimeResolution;

        cMessage *configureLoRaParameters;
        cMessage *selfPacket;

        //history of sent packets;
        cOutVector txSfVector;
        cOutVector txTpVector;

        // History of received packets
        cOutVector rxRssiVector;
        cOutVector rxSfVector;

        // cOutVector DistanceX;
        // cOutVector DistanceY;

        //variables to control ADR
        bool evaluateADRinNode;
        int ADR_ACK_CNT = 0;
        int ADR_ACK_LIMIT = 64; //64;
        int ADR_ACK_DELAY = 32; //32;
        bool sendNextPacketWithADRACKReq = false;
        void increaseSFIfPossible();

        int currDataInt;

        //General network variables
        int numberOfNodes;
        int numberOfEndNodes;

        //Packet sizes
        int dataPacketSize;
        int routingPacketMaxSize;

        //Routing variables
        int routingMetric;
        bool routeDiscovery;
        int windowSize;
        simtime_t routeTimeout;
        bool storeBestRoutesOnly;
        bool getRoutesFromDataPackets;
        simtime_t stopRoutingAfterDataDone;

        double routingPacketPriority;
        double ownDataPriority;
        int packetTTL;

        //Node info
        int nodeId;
        int originalNodeIndex;  // Original index before ID offset for end nodes

        std::vector<int> neighbourNodes;
        std::vector<int> knownNodes;
        std::vector<int> ACKedNodes;
        std::vector<LoRaAppPacket> LoRaPacketsToSend;
        std::vector<LoRaAppPacket> LoRaPacketsToForward;
        std::vector<LoRaAppPacket> LoRaPacketsForwarded;
        std::vector<LoRaAppPacket> DataPacketsForMe;


        //Application parameters
        bool requestACKfromApp;
        bool stopOnACK;
        bool AppACKReceived;
        int firstACK;

        //Spreading factor
        bool increaseSF;
        int firstACKSF;
        int packetsPerSF;
        int packetsInSF;

        //LoRa settings
        int minLoRaSF;
        int maxLoRaSF;

        //Forwarded packets vector size
        int forwardedPacketVectorSize;

        //Forward packets buffer max vector size
        int packetsToForwardMaxVectorSize;

        // Routing tables
        class singleMetricRoute {

            public:
                int id;
                int via;
                double metric;
                int window[33];
                simtime_t valid;       // existing validity timestamp
                // DSDV additions
                std::uint32_t seqNum;  // destination sequence number
                bool isValid;          // explicit valid/invalid flag (distinct from timestamp)
                simtime_t installTime; // when this route was installed/updated
        };
        std::vector<singleMetricRoute> singleMetricRoutingTable;

        class dualMetricRoute {

            public:
                int id;
                int via;
                double priMetric;
                double secMetric;
                int window[33];
                int sf;
                simtime_t valid;
        };
        std::vector<dualMetricRoute> dualMetricRoutingTable;

    // CSV logging state
    std::ofstream routingCsv;
    bool routingCsvReady = false;
    std::string routingCsvPath;

    // Delivered packets CSV state
    std::ofstream deliveredCsv;
    bool deliveredCsvReady = false;
    std::string deliveredCsvPath;

    // Path log state (shared single file across all nodes)
    bool pathLogReady = false;
    std::string pathLogFile; // delivered_packets/paths.csv

    // Convergence instrumentation: time when singleMetricRoutingTable first reaches threshold
    simtime_t firstTimeReached16 = -1; // -1 indicates not yet reached (legacy name kept)
    bool convergenceCsvReady = false;
    std::string convergenceCsvPath; // delivered_packets/routing_convergence.csv

    // Routing freeze feature
    bool freezeRoutingAtThreshold = false;        // parameter value
    int routingFreezeUniqueCount = 16;            // parameter value (legacy protocols)
    int expectedUniqueDestinations = -1;          // calculated: total nodes - self (for DSDV freeze)
    int dsdvFreezeUniqueCount = -1;               // parameter: DSDV freeze threshold (-1 = auto-calculate)
    bool routingFrozen = false;                   // becomes true once threshold reached (if feature enabled)
    simtime_t routingFrozenTime = -1;             // when frozen
    simtime_t freezeValidityHorizon = 0;          // horizon added to simTime when freezing routes

    // Global routing convergence stop feature
    bool stopRoutingWhenAllConverged = true;      // parameter value
    // Track whether THIS node has already announced local convergence
    bool locallyConverged = false;
    // Static shared counters to coordinate a global stop across all nodes
    static int globalNodesExpectingConvergence;   // total nodes considered for convergence
    static int globalNodesConverged;              // how many have reached threshold
    static bool globalConvergedFired;             // whether global stop already triggered
    static std::string globalConvergenceCsvPath;  // shared path for events
    static bool globalConvergenceCsvReady;        // header initialized
    void announceLocalConvergenceIfNeeded(int uniqueCount);
    void tryStopRoutingGlobally();

    // DSDV node-local state
    bool useDSDV = false;                                   // true if DSDV protocol selected
    cMessage *dsdvIncrementalTimer = nullptr;               // periodic incremental update timer
    cMessage *dsdvFullTimer = nullptr;                      // periodic full-dump timer
    std::uint32_t ownSeqNum = 0;                            // our own destination seq
    std::unordered_set<int> changedSet;                     // destinations changed since last ad
    std::unordered_map<int, simtime_t> lastHeard;           // neighborId -> last time heard
    simtime_t lastTriggeredUpdateTime = 0;                  // debounce for triggered updates
    bool dsdvPacketDue = false;                             // flag: DSDV packet ready to send
    bool dsdvSendFullDump = false;                          // flag: send full dump (vs incremental)
    simtime_t nextDsdvPacketTransmissionTime = 0;           // when DSDV packet can be sent
    // Metric sentinel used to denote unreachable in DSDV
    static const int INFINITE_METRIC = 0x3FFF;


        /**
         * @name CsmaCaMac state variables
         * Various state information checked and modified according to the state machine.
         */
        //@{
        enum State {
            IDLE,
            TRANSMIT,
            WAIT_DELAY_1,
            LISTENING_1,
            RECEIVING_1,
            WAIT_DELAY_2,
            LISTENING_2,
            RECEIVING_2,
        };


    public:
        LoRaNodeApp() {}
        simsignal_t LoRa_AppPacketSent;
        simsignal_t LoRa_AppPacketDelivered;
        //LoRa physical layer parameters
        double loRaTP;
        units::values::Hz loRaCF;
        int loRaSF;
        units::values::Hz loRaBW;
        int loRaCR;
        bool loRaUseHeader;
        bool loRaCAD;
        double loRaCADatt;

};

}

#endif