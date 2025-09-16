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
        bool enforceDutyCycle = default(true);
        double dutyCycle = default(0.01);
        int numberOfDestinationsPerNode = default(1);
        int numberOfPacketsPerDestination = default(1);
        int dataPacketDefaultSize @unit(B) = default(50B);
        int routingPacketMaxSize @unit(B) = default(16B);
        volatile double stopRoutingAfterDataDone @unit(s) = default(3600s);
        int forwardedPacketVectorSize = default(10);
        int packetsToForwardMaxVectorSize = default(0);
        volatile double timeToFailure @unit(s) = default(-1s); // <0 means never fail
        double failureJitterFrac = default(0); // fraction of timeToFailure for +/- uniform jitter
    // Global failure control (optional): if globalFailureSubsetCount > 0 then exactly that many nodes
    // will be chosen at runtime to fail. Their failure time will be:
    //   globalFailureStartTime + (exponential(globalFailureExpMean) if globalFailureExpMean>0 else 0)
    // All other nodes get timeToFailure = -1 (disabled). These parameters let you set the number
    // of failing nodes and a common start offset directly from the run configuration dialog.
    int globalFailureSubsetCount = default(-1);          // <=0 disables global subset logic
    volatile double globalFailureStartTime @unit(s) = default(-1s); // >=0 enables start offset logic
    volatile double globalFailureExpMean @unit(s) = default(0s);    // mean for exponential tail; 0 => deterministic
    gates:
        output appOut @labels(LoRaAppPacket/down);
        input appIn @labels(LoRaAppPacket/up);
}
