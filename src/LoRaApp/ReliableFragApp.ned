package loranetwork.LoRaApp;

simple ReliableFragApp
{
    parameters:
        @class(inet::ReliableFragApp);
        @display("i=block/app");

        int destId;                                   // destination node index
        int payloadBytes @unit(B) = default(1000B);    // total app bytes per message
        double sendInterval @unit(s) = default(5s);   // interval between messages

        // Fragmentation / ARQ knobs (mirror Arduino sketch)
        int fragmentSize @unit(B) = default(200B);
        int fragMaxTries = default(8);
        double fragAckTimeout @unit(s) = default(1s);
        int msgMaxTries = default(3);
        double finalAckBaseTimeout @unit(s) = default(1.8s);
        double perFragSpacing @unit(s) = default(0.015s);

        // LoRa PHY/MAC options are configured on LoRaNic; app is PHY-agnostic
        // Control info attached per packet
        double initialLoRaTP @unit(dBm) = default(17dBm);
        double initialLoRaCF @unit(Hz) = default(923MHz);
        int initialLoRaSF = default(8);
        double initialLoRaBW @unit(Hz) = default(125kHz);
        int initialLoRaCR = default(5);
        bool initialUseHeader = default(true);

    gates:
        output appOut @labels(LoRaAppPacket/down);
        input appIn @labels(LoRaAppPacket/up);
}
