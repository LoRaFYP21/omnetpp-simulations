// Minimal AODV-lite control packets used by LoRaNodeApp
// Generates aodv::Rreq and aodv::Rrep message classes

namespace aodv;

packet Rreq {
    int srcId;
    int dstId;
    int srcSeq;
    int dstSeq;
    int hopCount;
    int bcastId;
    int ttl;
}

packet Rrep {
    int srcId;      // originator id
    int dstId;      // destination id
    int dstSeq;
    int hopCount;
    double lifetime;
}
