# Required Arduino Libraries for SpaceMouse

## Libraries to Install (Working Version)

### 1. Adafruit TinyUSB Library
- **Purpose**: USB HID functionality for 3D mouse emulation and CDC serial communication
- **Installation**: 
  - Open Arduino IDE
  - Go to Tools → Manage Libraries
  - Search for "Adafruit TinyUSB Library"
  - Install the library by Adafruit
- **Version**: Latest version (tested with 2.0.0+)

### 2. OneButton
- **Purpose**: Button handling with click, double-click, and long-press detection
- **Installation**:
  - Go to Tools → Manage Libraries
  - Search for "OneButton"
  - Install the library by Matthias Hertel
- **Version**: Latest version (tested with 2.0.0+)

### 3. TLx493D (XENSIV Library)
- **Purpose**: Infineon TLV493D magnetometer sensor library
- **Installation**:
  - Go to Tools → Manage Libraries
  - Search for "TLx493D"
  - Install the library by Infineon Technologies
- **Version**: Latest version (tested with 1.0.0+)

## Board Requirements

### RP2040 Board Support
- **Primary**: Install "Adafruit RP2040 Boards" 
  - Go to Tools → Board → Boards Manager
  - Search for "Adafruit RP2040"
  - Install "Adafruit RP2040 Boards by Adafruit"
- **Alternative**: Install "Raspberry Pi Pico/RP2040"
  - Go to Tools → Board → Boards Manager
  - Search for "Raspberry Pi Pico"
  - Install "Raspberry Pi Pico/RP2040 by Earle F. Philhower, III"

### Board Settings (Working Configuration)
- **Board**: "Adafruit QT Py RP2040"
- **USB Stack**: "TinyUSB"
- **CPU Speed**: "133 MHz"
- **Upload Mode**: "UF2 Bootloader"

## Working Code Verification

### Current Working Version
The file `Code/2506-diy-spacemouse-simple/2506-diy-spacemouse-simple.ino` uses:
```cpp
#include <Adafruit_TinyUSB.h>
#include <OneButton.h>
#include <TLx493D_inc.hpp>
```

### Test Compilation Steps
1. **Open the working sketch**:
   - `Code/2506-diy-spacemouse-simple/2506-diy-spacemouse-simple.ino`
2. **Select correct board**:
   - Tools → Board → Adafruit RP2040 Boards → Adafruit QT Py RP2040
3. **Set USB Stack**:
   - Tools → USB Stack → TinyUSB
4. **Verify compilation**:
   - Sketch → Verify/Compile
   - Should compile without errors

## Installation Steps

### Step 1: Install Board Support
1. Open Arduino IDE
2. Go to Tools → Board → Boards Manager
3. Search for "Adafruit RP2040"
4. Install "Adafruit RP2040 Boards"

### Step 2: Install Required Libraries
1. Go to Tools → Manage Libraries
2. Install these libraries:
   - **Adafruit TinyUSB Library**
   - **OneButton**
   - **TLx493D**

### Step 3: Configure Board Settings
1. Select Board: "Adafruit QT Py RP2040"
2. Set USB Stack: "TinyUSB"
3. Set Upload Mode: "UF2 Bootloader"

## Troubleshooting

### Common Issues and Solutions

#### 1. "Library not found" Errors
- **Solution**: Install missing libraries via Library Manager
- **Alternative**: Download ZIP from GitHub and install manually

#### 2. "Board not found" Errors
- **Solution**: Install Adafruit RP2040 Boards support
- **Alternative**: Use Raspberry Pi Pico/RP2040 board support

#### 3. Upload Issues
- **Problem**: Device not detected in Arduino IDE
- **Solution**: Use dual-button reset (hold both buttons for 3 seconds)
- **Alternative**: Use BOOTSEL button on RP2040

#### 4. USB Driver Issues
- **Problem**: Device not recognized in Device Manager
- **Solution**: Install RP2040 drivers from Raspberry Pi
- **Alternative**: Use Zadig to install WinUSB drivers

#### 5. Serial Monitor Not Working
- **Problem**: No COM port visible
- **Solution**: Device is in 3D mouse mode, not serial mode
- **Workaround**: Use dual-button reset to enter DFU mode for programming

### Manual Library Installation
If libraries don't appear in Library Manager:
1. **Download ZIP files** from GitHub repositories
2. **Go to Sketch → Include Library → Add .ZIP Library**
3. **Select the downloaded ZIP files**

### Arduino IDE Version Requirements
- **Minimum**: Arduino IDE 1.8.19
- **Recommended**: Arduino IDE 2.x
- **Latest**: Arduino IDE 2.4+ (best compatibility)

## Success Indicators

### When Everything is Working:
- ✅ **Compilation**: No red error lines
- ✅ **Upload**: Successful upload to device
- ✅ **Device Recognition**: Appears as "SpaceMouse Compact" in Device Manager
- ✅ **Serial Communication**: COM port available for debugging
- ✅ **3D Mouse Functionality**: Works in 3D applications
- ✅ **Dual-Button Reset**: Hold both buttons for 3 seconds enters DFU mode

### Version Information
- **Working Code Version**: SIMPLE-1.1.0
- **Last Tested**: July 2025
- **Status**: Fully functional with all features working 