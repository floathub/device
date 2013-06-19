SOURCES := device.cpp libs/Adafruit_BMP/Adafruit_BMP085.cpp libs/AES/AES.cpp libs/Base64/Base64.cpp
LIBRARIES := Wire EEPROM Time AES Base64
BOARD := mega2560
TARGET := device
include ./arduino.mk