//
// LoRaHardwareApp.ned - NED definition for hardware-in-the-loop LoRa application
//

package loranetwork.LoRaApp;

simple LoRaHardwareApp
{
    parameters:
        @signal[LoRa_HW_PacketSent](type=long);
        @statistic[LoRa_HW_PacketSent](source=LoRa_HW_PacketSent; record=count);
        @signal[LoRa_HW_PacketReceived](type=long);
        @statistic[LoRa_HW_PacketReceived](source=LoRa_HW_PacketReceived; record=count);
        
        // Serial port configuration
        string serialPort = default("COM3");  // Windows COM port (e.g., COM3, COM4)
        int baudRate = default(115200);
        double pollInterval @unit(s) = default(0.01s);  // 10ms polling interval
        
        // LoRa physical layer parameters
        double initialLoRaTP @unit(dBm) = default(14dBm);
        double initialLoRaCF @unit(Hz) = default(923MHz);
        int initialLoRaSF = default(12);
        
        @class(inet::LoRaHardwareApp);
        
    gates:
        input appIn @labels(LoRaAppPacket/up);
        output appOut @labels(LoRaAppPacket/down);
}
