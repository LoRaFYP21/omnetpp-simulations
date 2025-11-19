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

package loranetwork.LoRaApp;

//
// TODO auto-generated module
//
simple LoRaNodeApp
{
    parameters:
        @signal[LoRa_AppPacketSent](type=long); // optional
        @statistic[LoRa_AppPacketSent](source=LoRa_AppPacketSent; record=count);
        int numberOfPacketsToForward = default(0);

        volatile double timeToFirstDataPacket @unit(s) = default(300s);
        string timeToNextDataPacketDist = default("uniform");
        volatile double timeToNextDataPacketMin @unit(s) = default(0s);
        volatile double timeToNextDataPacketMax @unit(s) = default(60s);
        volatile double timeToNextDataPacketAvg @unit(s) = default(30s);

        volatile double timeToFirstForwardPacket @unit(s) = default(300s);
        string timeToNextForwardPacketDist = default("uniform");
        volatile double timeToNextForwardPacketMin @unit(s) = default(0s);
        volatile double timeToNextForwardPacketMax @unit(s) = default(60s);
        volatile double timeToNextForwardPacketAvg @unit(s) = default(30s);

        volatile double timeToFirstRoutingPacket @unit(s) = default(60s);
        string timeToNextRoutingPacketDist = default("uniform");
		volatile double timeToNextRoutingPacketMin @unit(s) = default(0s);
		volatile double timeToNextRoutingPacketMax @unit(s) = default(600s);
		volatile double timeToNextRoutingPacketAvg @unit(s) = default(300s);

        double initialLoRaTP @unit(dBm) = default(14dBm);
        double initialLoRaCF @unit(Hz) = default(923MHz);
        int initialLoRaSF = default(12);
        int minLoRaSF = default(7);
        int maxLoRaSF = default(12);
        double initialLoRaBW @unit(Hz) = default(125kHz);
        int initialLoRaCR = default(4);
        bool initialUseHeader = default(true);
        bool initialLoRaCAD = default(false);
        bool iAmEnd = default(false);
        double initialLoRaCADatt @unit(dB) = default(10dB);
        @class(inet::LoRaNodeApp);
        bool evaluateADRinNode = default(false);
        int numberOfNodes;
        int numberOfEndNodes;
        int routingMetric = default(0);
        bool routeDiscovery = default(true);
        double ownDataPriority = default(1);
        int windowSize = default(10);
        int packetTTL = default(1);
        double routingPacketPriority = default(0.5);
        bool storeBestRouteOnly = default(false);
        bool getRoutesFromDataPackets = default(true);
        volatile double routeTimeout @unit(s) = default(60s);
        bool requestACKfromApp = default(false);
        bool stopOnACK = default(true);
        bool AppACKReceived = default(false);
        bool increaseSF = default(false);
        int packetsPerSF = default(1);
        int packetsInSF = default(0);
        bool sendPacketsContinuously = default(false);
        bool onlyNode0SendsPackets = default(false);
    // If true on a node, always send DATA to a specific destination id set by forcedDestinationId
    bool forceSingleDestination = default(false);
    int forcedDestinationId = default(-1);
        bool enforceDutyCycle = default(true);
        double dutyCycle = default(0.01);
        int numberOfDestinationsPerNode = default(1);
        int numberOfPacketsPerDestination = default(1);
        int dataPacketDefaultSize @unit(B) = default(50B);
        int routingPacketMaxSize @unit(B) = default(16B);
        volatile double stopRoutingAfterDataDone @unit(s) = default(3600s);
        int forwardedPacketVectorSize = default(10);
        int packetsToForwardMaxVectorSize = default(0);
    // When true, do not send proactive routing beacons; routes are discovered via AODV only
    bool aodvOnly = default(false);
    // Failure simulation parameters
    double timeToFailure @unit(s) = default(-1s);            // -1s disables per-node failure
    double failureJitterFrac = default(0);                   // Fractional jitter applied to deterministic failures
    int failureSubsetCount = default(0);                     // If >0, enables global subset coordinated failures
    double failureStartTime @unit(s) = default(0s);          // Base offset when subset failure process may start
    double failureExpMean @unit(s) = default(0s);            // Mean for exponential tail; 0 disables tail
    // Global coordinated failure subset parameters
    int globalFailureSubsetCount = default(0);               // >0 enables selecting that many nodes for coordinated failures
    double globalFailureStartTime @unit(s) = default(0s);    // Base time offset before failures may start
    double globalFailureExpMean @unit(s) = default(0s);      // Exponential tail mean for coordinated failures (0 = disabled)
    // Routing freeze feature: when enabled, once a node reaches routingFreezeUniqueCount unique destinations, stop modifying table
    bool freezeRoutingAtThreshold = default(false);
    int routingFreezeUniqueCount = default(16);
    // Horizon (in seconds) to extend route validity when freezing. Must stay within simtime range.
    // Use something comfortably below ~9.22e8 s max (2^63 * 1e-10 scale). Default ~1e7 s (~116 days).
    double freezeValidityHorizon @unit(s) = default(10000000s);
    // If true, once all participating nodes have a unique route to every destination (global convergence),
    // all nodes stop sending routing packets immediately.
    bool stopRoutingWhenAllConverged = default(true);
        // Export controls: when true, also write detailed per-node routing tables (single/dual/txt)
        bool exportDetailedRoutingTables = default(false);
        // Keep routing table focused only on end-node destinations (IDs in [endIdMin, endIdMax])
        bool keepOnlyEndDestinations = default(false);
        int endIdMin = default(1000);
        int endIdMax = default(1001);
    gates:
        output appOut @labels(LoRaAppPacket/down);
        input appIn @labels(LoRaAppPacket/up);
}
