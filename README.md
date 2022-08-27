# DehneEVSE Open source EV Charging Station - Firmware

Arduino firmware (for [Arduino Nano 33 IoT](https://store.arduino.cc/arduino-nano-33-iot)) for my [DehneEVSE EV Charging Station](https://dehnes.com/electronics/2021/03/31/dehneevse_charging_station.html).

This firmware has no charging logic at all. All it does is communicate with my [server](https://github.com/sebdehne/SmartHomeServer/tree/master/src/main/kotlin/com/dehnes/smarthome/ev_charging) via TCP/IP (no standard protocol, just raw bytes). It sends all it's data (like current state and power consumption) to the server in real-time. The server then evaulates what should be done and sends commands back, such as 1) enable/disable charging and at 2) which rate (ampere).

Keeping all the charging logic remote allowes for much more flexable charging logic, such as for example:
- only charge when your solar production is high (and keep charging at the rate which matches the solar-production)
- only charge when the energy prices are low
- if you have multiple charging stations like me, which share the same input cable, you can [balance the power beween them](https://github.com/sebdehne/SmartHomeServer/blob/master/src/main/kotlin/com/dehnes/smarthome/ev_charging/LoadSharing.kt#L50)


Features:
- Over-the-air (OTA) firmware updates
- Control Pilot (CP) handling:
  - drive signal as requested by server
  - notify server as soon as a change in voltage is detected
- measure and report Proximity Pilot (PP) to [server](https://github.com/sebdehne/SmartHomeServer/tree/master/src/main/kotlin/com/dehnes/smarthome/ev_charging)
- measure and report real-time charging load (voltage & current) for all three phases

Dependencies:
- [WiFiNINA](https://github.com/arduino-libraries/WiFiNINA)
- [ArduinoOTA](https://github.com/jandrassy/ArduinoOTA) - for writing the firmware to flash
- [CRC32](https://github.com/bakercp/CRC32)
- [javos65/WDTZero](https://github.com/javos65/WDTZero)
