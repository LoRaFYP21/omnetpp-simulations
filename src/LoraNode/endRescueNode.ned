//
// endRescueNode.ned
// Variant of endNode using LoRaEndRescueNodeApp
//
package loranetwork.LoraNode;

import inet.mobility.static.StationaryMobility; // default stationary model
import inet.mobility.contract.IMobility;        // interface so we can parametrize mobility type
import inet.mobility.single.LinearMobility; // fallback simple mobility
import inet.mobility.single.CircleMobility; // fallback simple mobility
import inet.mobility.single.MassMobility; // fallback simple mobility
import inet.networklayer.common.InterfaceTable;
import loranetwork.LoRa.LoRaNic;
import loranetwork.LoRaApp.LoRaEndRescueNodeApp;

module endRescueNode
{
    parameters:
        string deploymentType = default("");
        double minX @unit(m) = default(0m);
        double maxX @unit(m) = default(1000m);
        double sepX @unit(m) = default(1000m);
        double minY @unit(m) = default(0m);
        double maxY @unit(m) = default(1000m);
        double sepY @unit(m) = default(1000m);
        double initialX @unit(m) = default(0m);
        double initialY @unit(m) = default(0m);
        bool iAmEnd = default(true); // Added so LoRaEndRescueNodeApp can query host->par("iAmEnd")
        int cols = default(10);
        double maxGatewayDistance = default(320.0);
        double gatewayX = default(320.0);
        double gatewayY = default(320.0);
    // New: allow choosing a mobility model at runtime.
    // Set via **.mobilityType in .ini (e.g. "LinearMobility" or any available model).
    // Falls back to StationaryMobility if not overridden.
    string mobilityType = default("StationaryMobility");
        @networkNode();
        *.interfaceTableModule = default(absPath(".interfaceTable"));
        @display("bgb=297,313;i=device/accesspoint;is=vs");
    submodules:
        interfaceTable: InterfaceTable {
            @display("p=30,26");
        }
        // Mobility is now a parametric submodule; select via **.mobilityType in ini file.
        mobility: <mobilityType> like IMobility {
            @display("p=24,88");
        }
       
        LoRaNic: LoRaNic {
            @display("p=137,239");
        }
        // Use the expected submodule name 'LoRaNodeApp' so lower layers can locate it
        LoRaNodeApp: LoRaEndRescueNodeApp {
            @display("p=137,32");
        }
    connections allowunconnected:
    LoRaNodeApp.appOut --> LoRaNic.upperLayerIn;
    LoRaNic.upperLayerOut --> LoRaNodeApp.appIn;
}
