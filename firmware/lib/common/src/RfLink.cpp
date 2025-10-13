#include "common/RfLink.h"

namespace trackit {

namespace {
uint8_t computeChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum ^= data[i];
    }
    return checksum;
}
}

RfLink::RfLink(HardwareSerial& serialPort) : serial_(serialPort) {}

void RfLink::begin(uint32_t baudRate) {
    serial_.begin(baudRate);
}

void RfLink::sendPacket(const TrackingPacket& packet) {
    uint8_t buffer[kPacketSize];
    buffer[0] = kHeaderByte;
    buffer[1] = kVersion;
    memcpy(buffer + 2, &packet, sizeof(TrackingPacket));
    buffer[2 + sizeof(TrackingPacket)] = computeChecksum(buffer, 2 + sizeof(TrackingPacket));
    serial_.write(buffer, sizeof(buffer));
}

bool RfLink::receivePacket(TrackingPacket* outPacket) {
    if (!serial_.available()) {
        return false;
    }

    // Look for header alignment. This simple parser expects contiguous bytes, which is fine for HC-12.
    while (serial_.available()) {
        if (serial_.peek() == kHeaderByte) {
            break;
        }
        serial_.read();
    }

    if (serial_.available() < static_cast<int>(kPacketSize)) {
        return false;  // Not enough bytes yet
    }

    uint8_t buffer[kPacketSize];
    serial_.readBytes(buffer, kPacketSize);

    const uint8_t checksum = buffer[kPacketSize - 1];
    const uint8_t expected = computeChecksum(buffer, kPacketSize - 1);
    if (checksum != expected) {
        return false;
    }

    if (buffer[1] != kVersion) {
        return false;  // Future proofing: ignore packets from incompatible firmware
    }

    memcpy(outPacket, buffer + 2, sizeof(TrackingPacket));
    return true;
}

}  // namespace trackit
