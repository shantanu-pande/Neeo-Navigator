#Neeo Navigator - Complete Setup Guide
#### A smart navigation robot powered by Raspberry Pi 4 and ESP32.

## 📌 FULL PROCESS GUIDE
This document includes: 
- Repository cloning
- Raspberry Pi setup
- VS Code remote setup
- PlatformIO setup
- Workflow

## 🚀 1. Clone the Repository
```bash
git clone git@github.com:shantanu-pande/Neeo-Navigator.git
cd Neeo-Navigator
```

## 🖥️ 2. Raspberry Pi Setup
### Verify SSH
Connect via SSH

```bash
ssh pi@<IP_ADDRESS_OF_RPI>
```

## 💻 3. VS Code Remote Setup

1.  Install Visual Studio Code
2.  Install Extension: Remote - SSH
3.  Press Ctrl + Shift + P
4.  Select: Remote-SSH: Connect to Host
5.  Enter: pi@<IP_ADDRESS_OF_RPI>


## ⚙️ 4. PlatformIO Setup
### Install PlatformIO

-   Open VS Code
-   Go to Extensions
-   Install PlatformIO IDE

```bash 
pio project init –board esp32dev
```

Configure platformio.ini
```pio
[env:esp32dev] 
platform = espressif32 
board = esp32dev 
framework = arduino 
monitor_speed = 115200
```

## 🔄 5. DEVELOPMENT WORKFLOW

1.  SSH into Raspberry Pi
2.  Open project using VS Code Remote
3.  Write or modify ESP32 code
4.  Upload using PlatformIO
5.  Monitor logs and debug

## 🛠️ REQUIREMENTS
### Hardware: 
- Raspberry Pi 4 
- ESP32 Devkit V1
- Raspberry Pi Camera 
- VL53LOx Sensors 
- Motors & Driver

### Software: 
- VS Code
- PlatformIO
- SSH
- Python


## 👨‍💻 AUTHOR

Shantanu Pande
https://github.com/shantanu-pande
