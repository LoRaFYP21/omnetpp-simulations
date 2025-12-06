//
// HardwareNetwork.ned - Network definition with hardware integration
//

package loranetwork.simulations.networks;

import inet.networklayer.configurator.ipv4.IPv4NetworkConfigurator;
import inet.node.inet.StandardHost;
import loranetwork.LoRaPhy.LoRaMedium;
import loranetwork.LoraNode.LoRaNode;
import loranetwork.LoraNode.LoRaGW;
import loranetwork.LoraNode.LoRaHardwareNode;

network LoRaMeshWithHardware
{
    parameters:
        int numberOfNodes = default(5);
        int numberOfEndNodes = default(0);
        int numberOfGateways = default(1);
        int numberOfHardwareNodes = default(1);
        
        int networkSizeX = default(500);
        int networkSizeY = default(500);
        
        @display("bgb=2500,2500;bgl=2");
        
    submodules:
        // Regular simulated nodes
        N[numberOfNodes]: LoRaNode {
            @display("p=83.664,233.856;is=n");
        }
        
        // Gateway(s)
        loRaGW[numberOfGateways]: LoRaGW {
            @display("p=100,100;is=n");
        }
        
        // Hardware-in-the-loop node(s)
        hardwareNode[numberOfHardwareNodes]: LoRaHardwareNode {
            @display("p=250,250;is=n;i=device/wifilaptop");
        }
        
        // LoRa Medium
        LoRaMedium: LoRaMedium {
            @display("p=20,35;is=l;i=misc/sun");
        }
        
        // Network configurator (for gateway connectivity)
        configurator: IPv4NetworkConfigurator {
            parameters:
                assignDisjunctSubnetAddresses = false;
                @display("p=207.648,88.704");
        }
        
        // Network server (connected to gateway)
        networkServer: StandardHost {
            parameters:
                @display("p=208.24,27.216");
        }
        
    connections allowunconnected:
}
