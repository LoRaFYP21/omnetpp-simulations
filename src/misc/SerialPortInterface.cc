//
// SerialPortInterface.cc - Implementation of serial port communication
//

#include "SerialPortInterface.h"
#include <iostream>

namespace inet {

SerialPortInterface::SerialPortInterface() : 
    hSerial(INVALID_HANDLE_VALUE), 
    connected(false), 
    baudRate(CBR_115200) 
{
}

SerialPortInterface::~SerialPortInterface() {
    closePort();
}

bool SerialPortInterface::openPort(const std::string& port, DWORD baud) {
    // Close existing connection if any
    closePort();
    
    portName = port;
    baudRate = baud;
    
    // Open serial port
    std::wstring widePort(port.begin(), port.end());
    hSerial = CreateFileW(
        widePort.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );
    
    if (hSerial == INVALID_HANDLE_VALUE) {
        DWORD error = GetLastError();
        std::cerr << "Error opening serial port " << port << ": " << error << std::endl;
        return false;
    }
    
    // Configure port parameters
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Error getting port state" << std::endl;
        closePort();
        return false;
    }
    
    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;
    
    if (!SetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Error setting port state" << std::endl;
        closePort();
        return false;
    }
    
    // Set timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    
    if (!SetCommTimeouts(hSerial, &timeouts)) {
        std::cerr << "Error setting port timeouts" << std::endl;
        closePort();
        return false;
    }
    
    connected = true;
    std::cout << "Successfully opened serial port " << port << " at " << baud << " baud" << std::endl;
    return true;
}

void SerialPortInterface::closePort() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
    }
    connected = false;
}

int SerialPortInterface::writeData(const std::vector<uint8_t>& data) {
    if (!connected || data.empty()) {
        return 0;
    }
    
    DWORD bytesWritten;
    if (!WriteFile(hSerial, data.data(), data.size(), &bytesWritten, NULL)) {
        std::cerr << "Error writing to serial port" << std::endl;
        return 0;
    }
    
    return bytesWritten;
}

int SerialPortInterface::readData(std::vector<uint8_t>& buffer, int maxBytes) {
    if (!connected) {
        return 0;
    }
    
    buffer.resize(maxBytes);
    DWORD bytesRead;
    
    if (!ReadFile(hSerial, buffer.data(), maxBytes, &bytesRead, NULL)) {
        std::cerr << "Error reading from serial port" << std::endl;
        return 0;
    }
    
    buffer.resize(bytesRead);
    return bytesRead;
}

bool SerialPortInterface::dataAvailable() {
    if (!connected) {
        return false;
    }
    
    DWORD errors;
    COMSTAT status;
    
    if (!ClearCommError(hSerial, &errors, &status)) {
        return false;
    }
    
    return status.cbInQue > 0;
}

void SerialPortInterface::flush() {
    if (connected) {
        PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    }
}

} // namespace inet
