# EvChargingStationFirmware
Arduino firmware (for [Arduino Nano 33 IoT](https://store.arduino.cc/arduino-nano-33-iot)) for my [EV Charging station](https://github.com/sebdehne/EvChargingStationHardware). The firmware connects to my [SmartHomeServer](https://github.com/sebdehne/SmartHomeServer) software via wifi/tcp and listens for incoming requests:

Features (plannned):
- measure and report real-time charging load (voltage & current) per phase
- read and report status from the EVSE-controller
- set max allowed charging current as requested by server

Features (done):
- over-the-air (OTA) firmware updates

Using the measured charging-load, the server calculates how much charging-current is allowed at a time to enable load-sharing between multiple charging-stations (or to disable charging when energy prices are too high) and sends the max charging current back to this controller.


Dependencies:
- [WiFiNINA](https://github.com/arduino-libraries/WiFiNINA)
- [ArduinoOTA](https://github.com/jandrassy/ArduinoOTA) - for writing the firmware to flash
- [CRC32](https://github.com/bakercp/CRC32)
- [javos65/WDTZero](https://github.com/javos65/WDTZero)
