package loranetwork.LoraNode;

import inet.mobility.static.StationaryMobility;
import inet.networklayer.common.InterfaceTable;
import loranetwork.LoRa.LoRaNic;

// A LoRa node whose application submodule type is parameterized
module LoRaFragNode
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
        bool iAmEnd = default(false);
        int cols = default(10);
        double maxGatewayDistance = default(320.0);
        double gatewayX = default(320.0);
        double gatewayY = default(320.0);

        @networkNode();
        *.interfaceTableModule = default(absPath(".interfaceTable"));
        @display("bgb=297,313;i=device/enddevice;is=vs");

    submodules:
        interfaceTable: InterfaceTable {
            @display("p=30,26");
        }
        mobility: StationaryMobility {
            @display("p=24,88");
        }
        LoRaNic: LoRaNic {
            @display("p=137,239");
        }
        // Fixed app type (generic LoRaNodeApp); not used by LoRaFragLink
        app: loranetwork.LoRaApp.LoRaNodeApp {
            @display("p=137,32");
        }
    connections allowunconnected:
        app.appOut --> LoRaNic.upperLayerIn;
        LoRaNic.upperLayerOut --> app.appIn;
}
