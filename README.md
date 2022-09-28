# DehneEVSE Open source EV Charging Station - Firmware

Arduino firmware (for [Arduino Nano 33 IoT](https://store.arduino.cc/arduino-nano-33-iot)) for my [DehneEVSE EV Charging Station](https://dehnes.com/electronics/2021/03/31/dehneevse_charging_station.html).

This firmware has no charging logic at all and relies 100% on some external software, which needs to decide when to allow charing and at which rate. Keeping all the charging logic remote allowes for much more flexable charging logic, such as for example:
- only charge when your solar production is high (and keep charging at the rate which matches the solar-production)
- only charge when the energy prices are low
- if you have multiple charging stations like me, which share the same input cable, you can [balance the power beween them](https://github.com/sebdehne/SmartHomeServer/blob/master/src/main/kotlin/com/dehnes/smarthome/ev_charging/LoadSharing.kt#L50)

Communication with this firmware is handled via a binary protocol using TCP/IP (see code for details). If you prefer [MQTT](https://mqtt.org/), I wrote [dehneevse-mqtt-bridge](https://github.com/sebdehne/dehneevse-mqtt-bridge) which translates this binary protocol to simple JSON messages sent over MQTT. That should make it easy to implement your own charging logic and integration into [home-assistant](https://www.home-assistant.io/) or other platforms of your choice.

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
