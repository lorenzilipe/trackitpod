#pragma once

#include <HardwareSerial.h>
#include "types.h"

namespace trackit {

// Implements a minimal binary protocol over the HC-12 UART radio modules.
class RfLink {
  public:
    explicit RfLink(HardwareSerial& serialPort);

    void begin(uint32_t baudRate = 9600);

    // Serialize and transmit a tracking packet.
    void sendPacket(const TrackingPacket& packet);

    // Returns true when a full, checksum-validated packet is received.
    bool receivePacket(TrackingPacket* outPacket);

  private:
    static constexpr uint8_t kHeaderByte = 0xA5;
    static constexpr uint8_t kVersion = 1;
    static constexpr size_t kPacketSize = sizeof(TrackingPacket) + 3;  // header + version + checksum

    HardwareSerial& serial_;
};

}  // namespace trackit
