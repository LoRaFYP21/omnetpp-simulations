// AODV-lite control messages for LoRaMesh
// Defines RREQ and RREP packets used by LoRaNodeApp AODV logic

namespace aodv;

packet Rreq {
	int srcId;        // Originator of the request
	int dstId;        // Target destination
	int srcSeq;       // Originator sequence number
	int dstSeq;       // Destination sequence number (optional)
	int hopCount;     // Hop count from origin
	int bcastId;      // Broadcast ID to suppress duplicates
	int ttl;          // Time-to-live for propagation control
	// Accumulated path of node IDs from originator to current hop (append per forward)
	int path[];
}

packet Rrep {
	int srcId;        // Originator of the original RREQ
	int dstId;        // Destination node ID for the route
	int dstSeq;       // Destination sequence number (optional)
	int hopCount;     // Hop count back to the originator
	double lifetime;  // Route validity duration (seconds)
	// Accumulated path of node IDs from destination back to origin (append per forward)
	int path[];
}

