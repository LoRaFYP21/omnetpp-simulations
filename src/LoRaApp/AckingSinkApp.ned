package loranetwork.LoRaApp;

simple AckingSinkApp
{
    parameters:
        @class(inet::AckingSinkApp);
        @display("i=block/app");
        // LoRa control info for ACKs
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
