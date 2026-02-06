# ArduinOS

<img width="480" height="270" alt="image" src="https://github.com/user-attachments/assets/b73045d6-c49a-4f39-8abd-827e6eb2a04e" />


A unique operating system for **ESP32** that combines a smartwatch-style interface with a Windows 98-inspired desktop environment.

## Features

- **Dual Interface** - Boot into a clean watch interface, then launch a full desktop environment
- **Win98-Style Desktop** - Classic Windows 98 inspired UI with start menu, taskbar, and draggable windows
- **SD Card Support** - Create, edit, and view .txt files with hot-swap detection
- **Preinstalled Apps** - Snake, Tetris, Calculator, Notes, Paint, Timer, File Explorer, and more
- **Watch Interface** - Smartwatch-style home screen with time display and quick app access
- **Sound Effects** - Buzzer support for UI feedback and games
- **Screenshots** - Capture the screen to BMP files on SD card

## Recommended parts

| Component | Description |
|-----------|-------------|
| ESP32-WROOM-32 | Main microcontroller |
| ST7789 TFT Display | 240x135 pixels |
| 5-Way Joystick | Navigation + SET/RST buttons |
| MicroSD Card Module | Storage (HSPI) |
| Buzzer | Audio feedback |

## Wiring

| Component | Pin | ESP32 Pin |
|-----------|-----|-----------|
| **ST7789 Display** | CS | GPIO 5 |
| | RESET | GPIO 17 |
| | DC | GPIO 16 |
| | MOSI | GPIO 23 |
| | SCK | GPIO 18 |
| | BLK | GPIO 22 |
| **5-Way Joystick** | UP | GPIO 32 |
| | DOWN | GPIO 33 |
| | LEFT | GPIO 25 |
| | RIGHT | GPIO 26 |
| | MID | GPIO 27 |
| | SET | GPIO 14 |
| | RST | GPIO 12 |
| **MicroSD (HSPI)** | CS | GPIO 21 |
| | MOSI | GPIO 15 |
| | MISO | GPIO 35 |
| | SCK | GPIO 13 |
| **Buzzer** | Signal | GPIO 4 |

## Required Libraries

- **Adafruit GFX Library**
- **Adafruit ST7735 and ST7789 Library**
- **ESP32 Board Package** (Espressif)

## Installation

1. Download or clone this repository
2. Open `arduinOS.ino` in Arduino IDE
3. Install required libraries via Library Manager
4. Select your ESP32 board and port
5. Upload to your ESP32
6. Insert a FAT32-formatted MicroSD card

## Usage

- **Joystick** - Navigate menus and apps
- **MID button** - Select/confirm
- **SET button** - Back/cancel
- **RST button** - Additional functions (app-specific)
- **SET + RST** - Take screenshot

## License

Open source - feel free to modify and share!

---
*v0.1.0 - First public beta*

### More information on our website: https://arduin-os.vercel.app

#### Also, fun little story, the entire operating system was almost entirely deleted from our computers before backing up to github. There was a good chance we would have had to restart the project if it wasn't for HDD deleted data recovery software.
