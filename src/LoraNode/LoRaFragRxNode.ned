package loranetwork.LoraNode;

import inet.mobility.static.StationaryMobility;
import inet.networklayer.common.InterfaceTable;
import loranetwork.LoRa.LoRaNic;
import loranetwork.LoRaApp.AckingSinkApp;

module LoRaFragRxNode
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
        @networkNode();
        *.interfaceTableModule = default(absPath(".interfaceTable"));
        @display("bgb=297,313;i=device/enddevice;is=vs");

    submodules:
        interfaceTable: InterfaceTable { @display("p=30,26"); }
        mobility: StationaryMobility { @display("p=24,88"); }
        LoRaNic: LoRaNic { @display("p=137,239"); }
        app: AckingSinkApp { @display("p=137,32"); }
    connections allowunconnected:
        app.appOut --> LoRaNic.upperLayerIn;
        LoRaNic.upperLayerOut --> app.appIn;
}

