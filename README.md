# ESP32 PASCO AirLink EC-5 Monitor

This project lets an `ESP32` connect to a PASCO `AirLink (PS-3200)` over BLE, detect an attached `ECH2O EC-5` soil moisture probe, and either print live moisture readings to the serial monitor or publish them to MQTT over cellular.

It reports both PASCO-style values and a simpler normalized value:

- `VWC Potting Soil`
- `VWC Mineral Soil`
- `VWC Rockwool`
- `Relative Wetness` from `0%` to `100%`

## What This Project Is

The ESP32 acts as a BLE client for the AirLink. Once connected, it:

1. scans for an AirLink device
2. connects to PASCO BLE services
3. detects the attached sensor ID
4. confirms the EC-5 path as `PS-2163`
5. polls the AirLink once per second
6. decodes the returned PASCO sample packet
7. prints readable moisture values over serial

This is useful if you want to use a PASCO AirLink and EC-5 probe without needing PASCO software running all the time.

## Hardware Used

- `ESP32` development board
- `LilyGO T-A7670SA` cellular board with `A7670SA`
- `PASCO AirLink PS-3200`
- `ECH2O EC-5` soil moisture probe connected through the AirLink

Observed hardware labels during testing:

- Probe: `EC-5`
- AirLink: `PS-3200`
- Cellular modem: `A7670SA`

## How We Figured It Out

This project started as a generic PASCO BLE reader, but the AirLink plus EC-5 combination needed extra protocol work.

What we learned during reverse-engineering:

- The AirLink advertises as `AirLink`
- The attached sensor event comes back as sensor ID `0x0101`
- Sensor ID `0x0101` maps to PASCO model `PS-2163`
- The real EC-5 readings are returned in a `16-byte` sample payload
- The first three `32-bit little-endian 16.16` fields decode to `VWC Potting Soil`, `VWC Mineral Soil`, and `VWC Rockwool`
- A second alternating frame appears between real readings, so the firmware filters that false frame out

We also compared the ESP32 output against PASCO app exports taken in three conditions:

- probe in air
- probe halfway submerged
- probe fully submerged

That gave us the expected plateaus and let us confirm the decoding was correct.

## Expected Behavior

Typical values look like this:

- In air or very dry media, `Relative Wetness` should be near `0%` and the VWC values should be near `0%`
- Fully submerged, `Relative Wetness` should be near `100%`
- Fully submerged, `VWC Potting Soil` should be near `49-50%`
- Fully submerged, `VWC Mineral Soil` should be near `50%`
- Fully submerged, `VWC Rockwool` should be near `75%`

Important: fully submerged does not mean PASCO `VWC` should read `100%`. These values are calibrated for specific media and saturate below `100`.

## Project Files

- [platformio.ini]: PlatformIO project configuration
- [src/main.cpp]: BLE connection, packet decoding, and serial output
- [src/mqtt_main.cpp]: BLE monitor plus cellular MQTT publishing for the `T-A7670SA`
- [src/mqtt_hello.cpp]: simple cellular MQTT hello test without BLE
- CSV captures in the repo root: reference data exported from PASCO software during testing

## How To Build And Flash

This project uses PlatformIO.

Serial monitor app:

```powershell
pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor
```

If your ESP32 is on a specific port:

```powershell
pio run -e esp32dev -t upload --upload-port COM19
```

Cellular MQTT app:

```powershell
pio run -e esp32dev-mqtt
pio run -e esp32dev-mqtt -t upload
```

Cellular MQTT hello test:

```powershell
pio run -e esp32dev-mqtt-hello
pio run -e esp32dev-mqtt-hello -t upload
```

Before building the cellular MQTT app, check the `esp32dev-mqtt` `build_flags` in [platformio.ini]:

```ini
-DMODEM_DTR_PIN=25
-DMODEM_RX_PIN=27
-DMODEM_TX_PIN=26
-DMODEM_PWRKEY_PIN=4
-DMODEM_RESET_PIN=5
-DMODEM_POWER_ON_PIN=12
-DMODEM_RESET_LEVEL=1
-DMODEM_POWERON_PULSE_WIDTH_MS=100
-DMODEM_START_WAIT_MS=3000
-DCELLULAR_APN=\"internet\"
```

The defaults above are the tested working settings for a `LilyGO T-A7670SA` board. In this setup, `DTR=25` and `POWER_ON=12` were important for battery-only startup.

The cellular sketch uses TinyGSM with the `SIM7600` modem profile because SIMCom documents the A7670 AT command set as compatible with the SIM7500/SIM7600 series.

## How To Use It

1. Power your ESP32.
2. Turn on the PASCO AirLink.
3. Connect the EC-5 probe to the AirLink.
4. Open the serial monitor at `115200` baud.
5. For the cellular MQTT app, wait for cellular and MQTT to connect first.
6. The ESP32 then scans for the AirLink, identifies the sensor, and starts reading.
7. Watch live readings once per second.

Typical serial output:

```text
Connected to AirLink.
Attached sensor event ID from sensor: 0x101
Detected PASCO Soil Moisture Sensor PS-2163 profile.
Relative Wetness: 99.5%  VWC Potting Soil: 49.2%  Mineral Soil: 50.0%  Rockwool: 75.0%
```

For the cellular MQTT app, each successful sample is also published as a retained JSON payload to topic `esp32/pasco` on broker `tcp.ap-southeast-1.clawcloudrun.com:39024`.

Example MQTT payload:

```json
{"sensorId":257,"relativeWetness":99.50,"pottingSoil":49.20,"mineralSoil":50.00,"rockwool":75.00,"waterPotential":0.00}
```

The simple hello test publishes a retained payload like:

```json
{"message":"hello","bootCount":1,"millis":25452,"chip":"esp32-hello-b61815ac","modem":"A7670"}
```

Use this hello test if you want to verify cellular power-up and MQTT on battery without involving BLE or the PASCO sensor path.

## Relative Wetness

`Relative Wetness` is a convenience value added in this project. It is not a PASCO measurement name.

It normalizes the three PASCO VWC outputs against their observed saturation levels:

- Potting Soil normalized to `50`
- Mineral Soil normalized to `50`
- Rockwool normalized to `75`

Then it averages those normalized values into a simple `0-100%` scale.

This makes it easier to treat the probe like a general wet/dry indicator while still preserving the original PASCO-style readings.

## Notes And Limitations

- This project is currently tuned for the AirLink plus EC-5 path identified as `PS-2163`
- The cellular defaults are currently tuned for the `LilyGO T-A7670SA`
- `Water Potential` is not yet shown reliably from the observed AirLink packet stream
- The startup message `No AirLink bridge response received.` may still appear even when normal sampling works

## Source References

Protocol work was based on:

- PASCO BLE UUID patterns
- PASCO public code examples
- live packet inspection from the user's AirLink and EC-5 hardware
- comparison against PASCO app exports collected during dry, half-submerged, and submerged tests

## Summary

This repo is a working ESP32 monitor for a PASCO AirLink with an attached EC-5 probe. It connects over BLE, decodes the AirLink sample payload correctly, filters false alternating frames, and can either print stable live moisture readings locally or publish them over cellular MQTT for remote logging and automation.
