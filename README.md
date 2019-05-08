# ESP8266-Ant-SW-LED-proto
ESP8266 Remote Antenna Controller Software - used to manage my ham radio SO2R antenna switching requirements

This code base is used for prototyping various controller and network communication enhancements before deploying to 
the actual WiFi relay boards that control my DX Engineering 8x2 ham radio antenna switch.

Current version uses a simple telnet protocol to send commands and receive switch status.

Expecting to change my approach to potentially use an MQTT client / broker approach or a simple COAP communication protocol.

That will be after some enhancements to make the code more resilient to WiFi disconnects.  Intend to add a watchdog reset/restart
capability to reduce the need for power cycling the switches to re-initialize connections/operation.  
