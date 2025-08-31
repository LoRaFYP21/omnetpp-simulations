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

package loranetwork.LoraNode;

import inet.mobility.static.StationaryMobility;
import inet.networklayer.common.InterfaceTable;
import loranetwork.LoRa.LoRaNic;
import loranetwork.LoRaApp.LoRaNodeApp;


module LoRaNode
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
        double rad =  default(320.0);
        double centX = default(320.0);
        double centY = default(320.0);
        
        @networkNode();
        *.interfaceTableModule = default(absPath(".interfaceTable"));
        @display("bgb=297,313;i=device/accesspoint;is=vs");
//		@display("bgb=297,313;i=device/accesspoint;i2=device/enddevice;i=!iAmEnd?device/accesspoint:device/enddevice;is=vs");
		
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
        LoRaNodeApp: LoRaNodeApp {
            @display("p=137,32");
        }
    connections allowunconnected:

        LoRaNodeApp.appOut --> LoRaNic.upperLayerIn;
        LoRaNic.upperLayerOut --> LoRaNodeApp.appIn;
}
