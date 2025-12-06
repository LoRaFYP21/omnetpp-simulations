//
// SerialPortInterface.h - Serial communication interface for hardware integration
// Provides cross-platform serial port communication for Windows
//

#ifndef __LORA_OMNET_SERIALPORTINTERFACE_H_
#define __LORA_OMNET_SERIALPORTINTERFACE_H_

#include <string>
#include <vector>
#include <windows.h>

namespace inet {

class SerialPortInterface {
private:
    HANDLE hSerial;
    bool connected;
    std::string portName;
    DWORD baudRate;
    
public:
    SerialPortInterface();
    ~SerialPortInterface();
    
    /**
     * Open serial port connection
     * @param port - Port name (e.g., "COM3")
     * @param baud - Baud rate (e.g., 115200)
     * @return true if successful
     */
    bool openPort(const std::string& port, DWORD baud = CBR_115200);
    
    /**
     * Close serial port connection
     */
    void closePort();
    
    /**
     * Check if port is connected
     */
    bool isConnected() const { return connected; }
    
    /**
     * Write data to serial port
     * @param data - Data to write
     * @return number of bytes written
     */
    int writeData(const std::vector<uint8_t>& data);
    
    /**
     * Read data from serial port
     * @param buffer - Buffer to store read data
     * @param maxBytes - Maximum bytes to read
     * @return number of bytes read
     */
    int readData(std::vector<uint8_t>& buffer, int maxBytes = 256);
    
    /**
     * Check if data is available to read
     */
    bool dataAvailable();
    
    /**
     * Flush port buffers
     */
    void flush();
};

} // namespace inet

#endif
