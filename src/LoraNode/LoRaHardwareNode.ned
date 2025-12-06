//
// LoRaHardwareNode.ned - Node module for hardware-in-the-loop integration
//

package loranetwork.LoraNode;

import inet.mobility.static.StationaryMobility;
import inet.networklayer.common.InterfaceTable;
import loranetwork.LoRa.LoRaNic;
import loranetwork.LoRaApp.LoRaHardwareApp;

module LoRaHardwareNode
{
    parameters:
        @networkNode();
        @display("i=device/accesspoint;bgb=297,313");
        
        // Serial port configuration
        string serialPort = default("COM3");
        int baudRate = default(115200);
        
        // Position parameters
        double latitude = default(0);
        double longitude = default(0);
        
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
        
        LoRaHardwareApp: LoRaHardwareApp {
            @display("p=137,32");
            serialPort = parent.serialPort;
            baudRate = parent.baudRate;
        }
        
    connections allowunconnected:
        LoRaHardwareApp.appOut --> LoRaNic.upperLayerIn;
        LoRaNic.upperLayerOut --> LoRaHardwareApp.appIn;
}
