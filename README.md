# DehneEVSE Open source EV Charging Station - Firmware

Arduino firmware (for [Arduino Nano 33 IoT](https://store.arduino.cc/arduino-nano-33-iot)) for my [DehneEVSE EV Charging Station](https://dehnes.com/electronics/2021/03/31/dehneevse_charging_station.html).

Features:
- Over-the-air (OTA) firmware updates
- Control Pilot (CP) handling:
  - drive signal as requested by server
  - notify server as soon as a change in voltage is detected
- measure and report Proximity Pilot (PP) to server
- measure and report real-time charging load (voltage & current) for all three phases

Dependencies:
- [WiFiNINA](https://github.com/arduino-libraries/WiFiNINA)
- [ArduinoOTA](https://github.com/jandrassy/ArduinoOTA) - for writing the firmware to flash
- [CRC32](https://github.com/bakercp/CRC32)
- [javos65/WDTZero](https://github.com/javos65/WDTZero)
