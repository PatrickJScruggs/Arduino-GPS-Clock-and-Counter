# Arduino GPS-Disciplined Clock and Counter

This project is an Arduino-based clock and counter that uses GPS time signals to maintain near-atomic-clock accuracy. It features a ten-digit seven-segment display for showing the count or time, and a 20x4 character LCD display for displaying additional information and menus. The program reads NMEA sentences from the Garmin GPS module and uses the PPS signal to set the time accurately. It calculates the total elapsed seconds by subtracting the activation time stored in EEPROM from the current unix time and applying a leap second offset. The LCD display shows various information screens by default and allows the user to select different modes using the rotary encoder. The seven-segment display is controlled through a shift register, allowing the Arduino to control individual segments.

## Features

- Displays total elapsed seconds since device activation on the ten-digit seven-segment display
- Uses GPS PPS signal to set the time with high accuracy
- 20x4 character LCD display cycles through information screens by default:
  - Number of satellites, HDOP, VDOP, and PDOP
  - Clock-time when the main count will reach the next million and billion milestones
- Rotary encoder and LCD screen for selecting other display modes:
  - Current clock time
  - Unix time
  - Countdown to the next New Year's Eve
  - Counting up from 0 at rates faster than 1 count/second

## Hardware

- Arduino MEGA
- Ten-digit seven-segment display with shift register
- 20x4 character LCD display
- Garmin 18x LVC GPS module
- Rotary encoder

## Requirements

- [Time library (modified for 64-bit unix time support)](https://github.com/PatrickJScruggs/Time64)
- [LiquidCrystal_I2C library](https://github.com/johnrickman/LiquidCrystal_I2C) for the 20x4 character LCD display
- [Encoder library](https://github.com/PaulStoffregen/Encoder) for the rotary encoder
- The standard [Arduino EEPROM library](https://docs.arduino.cc/learn/built-in-libraries/eeprom/) for storing the device activation time

## Wiring

### Garmin 18x LVC GPS Module

| Garmin 18x LVC Pin | Arduino MEGA Pin |
|--------------------|------------------|
| VIN                | 5V               |
| GND                | GND              |
| RX                 | TX3 (14)         |
| TX                 | RX3 (15)         |
| PPS                | 21 (INT0)        |

### 20x4 Character LCD Display (I2C)

| LCD Pin | Arduino MEGA Pin |
|---------|------------------|
| VCC     | 5V               |
| GND     | GND              |
| SDA     | 20 (SDA)         |
| SCL     | 21 (SCL)         |

### Rotary Encoder

| Encoder Pin | Arduino MEGA Pin |
|-------------|------------------|
| CLK         | 2                |
| DT          | 3                |
| SW          | 4                |
| +           | 5V               |
| GND         | GND              |

### Ten-Digit Seven-Segment Display with Shift Register

| Shift Register Pin | Arduino MEGA Pin |
|--------------------|------------------|
| VCC                | 5V               |
| GND                | GND              |
| DS (Data)          | 5                |
| SHCP (Clock)       | 6                |
| STCP (Latch)       | 7                |
| OE (Output Enable) | GND              |

Connect the shift register outputs to the corresponding segments of the ten-digit seven-segment display according to the pin mapping in the code.

