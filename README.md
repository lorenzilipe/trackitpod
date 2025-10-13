# TrackItPod Firmware Infrastructure

TrackItPod is a two-part GPS tracking system inspired by the Soloshot camera robot. The wearable **tag** publishes its GPS position over an HC-12 radio. The tripod-mounted **base station** listens for updates, computes the required yaw/pitch angles, and drives two 28BYJ-48 stepper motors so a camera always faces the subject.

This repository currently provides the Arduino-style firmware skeleton, project scaffolding, and documentation to get you started quickly—even if you mainly write Python.

## Repository layout

```
firmware/
  base_station/        PlatformIO project for the tripod controller
    src/main.cpp       Main control loop with calibration & tracking
    platformio.ini     Build configuration targeting Seeed XIAO ESP32-C3
  tag_unit/            PlatformIO project for the wearable tag
    src/main.cpp       Broadcasts GPS fixes over HC-12
    platformio.ini
  lib/common/          Shared modules used by both projects
    include/common/    Headers for GPS, radio, motors, OLED, math helpers
    src/               Implementations
hardware/              Drop schematics, wiring notes, photos here
docs/                  Additional design notes and future specifications
```

Each PlatformIO project builds independently, but they share the `firmware/lib/common` directory via `lib_extra_dirs` to avoid duplication.

## Choosing Arduino C++

The Seeed Studio XIAO ESP32-C3 runs both Arduino and MicroPython. We selected **Arduino (C++)** because:

- The ecosystem already has mature libraries for TinyGPS++, Adafruit SSD1306, and the HC-12 radio.
- Deterministic motor control is easier with Arduino's tighter timing and `Stepper` support.
- PlatformIO provides a repeatable build, dependency management, and IntelliSense.

The code is heavily commented to reduce the C++ learning curve. Every hardware abstraction (GPS, RF, motors, OLED) has a small wrapper class in `firmware/lib/common`.

## Firmware walkthrough

### Base station (tripod)

1. Boots, initialises GPS (NEO-6M/GT-U7), OLED, radio, and steppers.
2. Waits for a calibration button press. During calibration you:
   - Level the tripod and point the camera front toward the tag.
   - Hold the button so the base captures the instantaneous bearing to the tag.
   - That bearing becomes the mechanical zero offset for subsequent tracking.
3. During tracking:
   - The base reads its own GPS fix and the latest packet from the tag.
   - `computeAimSolution()` converts the lat/lon pair into a yaw/pitch requirement.
   - `MotorController` advances the 28BYJ-48 steppers to maintain the aim.
   - The OLED shows live coordinates and pointing solution.

Safety guardrails:

- Tag packets older than two seconds are ignored to avoid random swings.
- Motor movements smaller than half a degree are skipped to reduce jitter.

### Tag unit (wearable)

1. Boots GPS, OLED, and radio.
2. Broadcasts a `TrackingPacket` containing the latest GPS fix every 250 ms.
3. OLED displays sequence number and coordinates for quick validation.

## Calibration workflow

1. Power up both base and tag outdoors with a clear view of the sky.
2. Wait for “ACQUIRING GPS” to disappear on both OLEDs (you may see coarse coordinates once a lock is obtained).
3. Physically aim the tripod’s camera front at the subject/tag (this is your mechanical zero).
4. Hold the calibration button (configured on GPIO 2) until the OLED shows the stored offset.
5. Release the button—tracking starts as soon as fresh tag packets arrive.

> **Tip:** Add a spring-loaded momentary switch wired to GPIO 2 and ground for the calibration input. INPUT_PULLUP mode keeps wiring simple.

## Building and uploading

Install [PlatformIO](https://platformio.org/) (VS Code extension or command line) and connect the Seeed Studio XIAO ESP32-C3 via USB.

### Base station

```powershell
cd firmware/base_station
pio run --target upload
pio device monitor
```

### Tag unit

```powershell
cd firmware/tag_unit
pio run --target upload
pio device monitor
```

The monitor command lets you view `Serial` debug output at 115200 baud.

## Next steps

- [ ] Validate UART pin mappings on the actual wiring harness; adjust constants if needed.
- [ ] Measure real-world motor gearing and tune `stepsPerRevolution` and `setSpeed`.
- [ ] Add packet loss handling (timeouts, smoothing filters).
- [ ] Implement a higher-level PID or motion planner for smooth tracking.
- [ ] Store calibration offset in non-volatile storage (NVS) to survive reboots.
- [ ] Add indoor positioning support (vision, UWB, etc.) in future iterations.

## Troubleshooting cheatsheet

| Symptom | Likely cause | Quick check |
| ------- | ------------ | ----------- |
| OLED stays blank | I2C wires swapped or incorrect address | Use `Wire.begin()` scan sketch and confirm address 0x3C |
| Base never calibrates | Tag packets stale or GPS fix missing | Watch serial log for `Tag packet seq` updates |
| Motors jitter wildly | Heading offset misconfigured | Re-run calibration after re-aiming tripod |
| No packets received | HC-12 baud mismatch or channel mismatch | Ensure both radios set to 9600 baud, same channel |

Happy building! Contributions, wiring diagrams, photos, and questions are welcome—open an issue or drop notes in the `docs/` folder.
