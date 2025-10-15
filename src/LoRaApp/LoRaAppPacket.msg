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

//
// TODO generated message class
//

cplusplus {{
#include "../misc/DevAddr.h"
#include <inet/common/Units.h>
}}

class noncobject inet::units::values::W;
class noncobject inet::units::values::Hz;

enum AppPacketType
{
    JOIN_REQUEST = 1;
    JOIN_REPLY = 2;
    DATA = 3;
    TXCONFIG = 4;
    ACK = 5;
    ROUTING = 6;
}

class LoRaOptions {
    double LoRaTP = -1;
    inet::units::values::Hz LoRaCF;
    int LoRaSF = -1;
    inet::units::values::Hz LoRaBW;
    int LoRaCR = -1;
    double RSSI;
    bool UseHeader = true;
    bool ADRACKReq;
    bool AppACKReq;
}

class LoRaRoute {
    int id;
    int priMetric;
    int secMetric;
}

packet LoRaAppPacket {
    int msgType @enum(AppPacketType);
    int dataInt;
    LoRaOptions options;
    int source;
    int destination;
    int ttl;
    int via;
    // Previous hop that forwarded this envelope (used by AODV to install routes)
    int lastHop;
	LoRaRoute routingTable[];
	simtime_t departureTime;
    // Delivery path trace: append nodeId at each transmitter and optionally receiver
    int hopTrace[];
}
