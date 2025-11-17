# THEMS Flow Data Logger

This repository now contains a unified Arduino/ESP32 sketch that combines the previous GPS and RTC + LCD examples into a single firmware that:

- Reads GPS data over Serial2
- Maintains real-time timekeeping with a DS3231 RTC
- Displays status on an I²C LCD
- Logs records to an SD card at ~3 Hz
- Captures a placeholder sensor reading for future expansion

## Firmware layout

`firmware/THEMS_flow_logger.ino` contains the sketch plus helper functions for configuring hardware, capturing readings, and writing CSV rows to the SD card. Adjust pin assignments and the LCD I²C address near the top of the file as needed for your board.

## Getting started

1. Install the required Arduino libraries if you do not already have them:
   - **RTClib** (Adafruit)
   - **LiquidCrystal_I2C**
   - **TinyGPSPlus**
   - **SD** (bundled with the ESP32 Arduino core)
2. Wire peripherals:
   - GPS module to `RX2=16`, `TX2=17` (modify in the sketch if different)
   - RTC on I²C (`SDA=21`, `SCL=22`)
   - LCD at I²C address `0x27` (update if necessary)
   - SD card module `CS` default is `GPIO 5`
   - Placeholder sensor connected to `GPIO 34`
3. Open `firmware/THEMS_flow_logger.ino` in the Arduino IDE and flash to the ESP32.
4. The firmware will create `/datalog.csv` on the SD card with the following columns: `date,time,latitude,longitude,altitude_m,speed_kmph,satellites,sensor`.

## Next steps

- Replace the placeholder sensor read with your actual sensor logic.
- Share pin mappings or hardware differences if they differ from the defaults so the sketch can be tuned further.
