#include <Adafruit_TinyUSB.h>
#include <OneButton.h>
#include <TLx493D_inc.hpp>

// Version information
const char* VERSION = "SIMPLE-1.1.0";

// 3D Mouse HID Report Descriptor (3DConnexion compatible) - REMOVED KEYBOARD
uint8_t const desc_hid_report[] = {
  // 3DConnexion SpaceMouse compatible descriptor only
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x08,        // Usage (Multi-Axis Controller)
  0xA1, 0x01,        // Collection (Application)
  0x85, 0x02,        // Report ID (2)
  0x09, 0x01,        // Usage (Pointer)
  0xA1, 0x00,        // Collection (Physical)
  0x09, 0x30,        // Usage (X)
  0x09, 0x31,        // Usage (Y)
  0x09, 0x32,        // Usage (Z)
  0x09, 0x33,        // Usage (Rx) - Rotation X
  0x09, 0x34,        // Usage (Ry) - Rotation Y
  0x09, 0x35,        // Usage (Rz) - Rotation Z
  0x15, 0x81,        // Logical Minimum (-127)
  0x25, 0x7F,        // Logical Maximum (127)
  0x75, 0x08,        // Report Size (8)
  0x95, 0x06,        // Report Count (6) - X,Y,Z,Rx,Ry,Rz
  0x81, 0x06,        // Input (Data, Variable, Relative)
  0xC0,              // End Collection
  0xC0               // End Collection
};

// USB HID object
Adafruit_USBD_HID usb_hid;

using namespace ifx::tlx493d;
TLx493D_A1B6 mag(Wire, TLx493D_IIC_ADDR_A0_e);

// Setup buttons - using QT Py RP2040 pins (SWAPPED for testing)
// Try different button configuration to fix interference
OneButton button1(24, false);  // Changed from true to false (inverted logic)
OneButton button2(27, false);  // Changed from true to false (inverted logic)

// Simple variables (no complex processing)
double xOffset = 0, yOffset = 0, zOffset = 0;
double xCurrent = 0, yCurrent = 0, zCurrent = 0;

// Basic parameters
int calSamples = 100;  // Reduced from 300
int sensitivity = 16;   // Increased from 8 for faster movement
double xyThreshold = 0.4; // Increased from 0.3 (based on working examples)
double deadzone = 0.15;  // Increased from 0.1 for better stability
double zThreshold = xyThreshold * 2.5; // Z threshold for orbit mode

bool wasMoving = false;
unsigned long count = 0;  // Changed from uint8_t to prevent overflow

// Button callback functions
void button1Click() {
  Serial.println("=== Button 1 clicked ===");
  Serial.print("Button 1 state: "); Serial.println(digitalRead(24));
  Serial.print("Button 2 state: "); Serial.println(digitalRead(27));
  // Send home command using mouse button (alternative to keyboard)
  if (usb_hid.ready()) {
    // Use right mouse button as home command alternative
    usb_hid.mouseButtonPress(2, MOUSE_BUTTON_RIGHT);
    delay(10);
    usb_hid.mouseButtonRelease(2);
  }
  Serial.println("Sent home command (right mouse button)");
  Serial.println("=======================");
}

void button2Click() {
  Serial.println("Button 2 clicked");
  // Double middle click for fit to screen (like original example)
  if (usb_hid.ready()) {
    usb_hid.mouseButtonPress(2, MOUSE_BUTTON_MIDDLE);
    delay(10);
    usb_hid.mouseButtonRelease(2);
    delay(10);
    usb_hid.mouseButtonPress(2, MOUSE_BUTTON_MIDDLE);
    delay(10);
    usb_hid.mouseButtonRelease(2);
  }
  Serial.println("Sent fit to screen command");
}

// Dual button reset function
void checkDualButtonReset() {
  static bool button1Pressed = false;
  static bool button2Pressed = false;
  static unsigned long dualPressStart = 0;
  static bool resetTriggered = false;
  
  bool currentButton1 = (digitalRead(24) == 0);  // Inverted logic
  bool currentButton2 = (digitalRead(27) == 0);  // Inverted logic
  
  // Detect button press states
  if (currentButton1 && !button1Pressed) {
    button1Pressed = true;
    Serial.println("Button 1 pressed");
  } else if (!currentButton1 && button1Pressed) {
    button1Pressed = false;
    Serial.println("Button 1 released");
  }
  
  if (currentButton2 && !button2Pressed) {
    button2Pressed = true;
    Serial.println("Button 2 pressed");
  } else if (!currentButton2 && button2Pressed) {
    button2Pressed = false;
    Serial.println("Button 2 released");
  }
  
  // Check for dual button press
  if (button1Pressed && button2Pressed && !resetTriggered) {
    if (dualPressStart == 0) {
      dualPressStart = millis();
      Serial.println("=== DUAL BUTTON PRESS DETECTED ===");
      Serial.println("Hold both buttons for 3 seconds to reset...");
    } else if (millis() - dualPressStart >= 3000) {  // 3 second hold
      Serial.println("=== ENTERING DFU MODE ===");
      Serial.println("Dual button DFU reset triggered!");
      Serial.println("Device will now appear as RPI-RP2 in Arduino IDE");
      delay(100);  // Brief delay for serial output
      rp2040.rebootToBootloader();  // Enter DFU mode for programming
      resetTriggered = true;
    }
  } else if (!button1Pressed || !button2Pressed) {
    // Reset the dual press timer if either button is released
    dualPressStart = 0;
    resetTriggered = false;
  }
}

void setup() {
  // Initialize USB HID with proper 3D mouse support
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  // Set 3DConnexion device identification
  TinyUSBDevice.setID(0x256F, 0xC62E);  // 3DConnexion SpaceMouse Compact VID/PID
  TinyUSBDevice.setManufacturerDescriptor("3Dconnexion");
  TinyUSBDevice.setProductDescriptor("SpaceMouse Compact");
  TinyUSBDevice.setSerialDescriptor("DIY-SpaceMouse-2025");
  
  // Enable CDC (Serial over USB) for debugging
  //TinyUSBDevice.enableCDC();

  // Set up HID
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.setBootProtocol(HID_ITF_PROTOCOL_NONE);
  usb_hid.setPollInterval(2);
  usb_hid.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  button1.attachClick(button1Click);
  button2.attachClick(button2Click);

  Serial.begin(115200);
  delay(1000);
  
  // Test button pins (SWAPPED for testing)
  pinMode(24, INPUT_PULLUP);
  pinMode(27, INPUT_PULLUP);
  Serial.print("Initial Button 1 state (pin 24): "); Serial.println(digitalRead(24));
  Serial.print("Initial Button 2 state (pin 27): "); Serial.println(digitalRead(27));
  Serial.println("Note: With inverted logic, pressed = 0, not pressed = 1");
  
  // Print version information
  Serial.println("=== SpaceMouse Simple Startup ===");
  Serial.print("Version: "); Serial.println(VERSION);
  Serial.println("NEW FEATURES:");
  Serial.println("- Dual button DFU reset: Hold both buttons for 3 seconds");
  Serial.println("- USB keyboard interference fixed (keyboard HID removed)");
  Serial.println("- Button 1: Right mouse button (home command)");
  Serial.println("- Button 2: Double middle click (fit to screen)");
  Serial.println("================================");

  // Initialize sensor
  Serial.println("Initializing sensor...");
  mag.begin();
  Serial.println("Sensor initialized");

  // Simple calibration
  Serial.println("Calibrating sensor...");
  for (int i = 1; i <= calSamples; i++) {
    double x, y, z;
    mag.getMagneticField(&x, &y, &z);
    
    xOffset += x;
    yOffset += y;
    zOffset += z;

    if (i % 20 == 0) Serial.print(".");
  }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;

  Serial.println();
  Serial.print("Calibration complete - X: "); Serial.print(xOffset, 3);
  Serial.print(" Y: "); Serial.print(yOffset, 3);
  Serial.print(" Z: "); Serial.println(zOffset, 3);
}

void loop() {
  // Simple loop indicator with timing debug
  static unsigned long lastBlink = 0;
  static unsigned long lastLoopTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastBlink > 1000) {
    Serial.print("Loop running... (loop time: ");
    Serial.print(currentTime - lastLoopTime);
    Serial.println("ms)");
    lastBlink = currentTime;
  }
  lastLoopTime = currentTime;
  
  // Update button states
  button1.tick();
  button2.tick();
  
  // Check for dual button reset
  checkDualButtonReset();
  
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();

  // Debug button states (SWAPPED for testing)
  if (count % 500 == 0) {
    Serial.print("Button 1 state (pin 24): "); Serial.println(digitalRead(24));
    Serial.print("Button 2 state (pin 27): "); Serial.println(digitalRead(27));
    Serial.print("Raw sensor - X: "); Serial.print(xCurrent, 3);
    Serial.print(" Y: "); Serial.print(yCurrent, 3);
    Serial.print(" Z: "); Serial.println(zCurrent, 3);
    Serial.print("Movement threshold: "); Serial.println(xyThreshold);
    Serial.print("Above threshold: "); Serial.println((abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold) ? "YES" : "NO");
  }

  // Read sensor (simplified) - add protection against button interference
  double x, y, z;
  
  // Read sensor with Button 1 interference protection (with debounce)
  static bool lastButton1State = true;  // Start with not pressed
  bool currentButton1State = digitalRead(24);
  
  if (currentButton1State == 0 && lastButton1State == 1) {  // Button 1 just pressed
    x = xOffset; y = yOffset; z = zOffset;
  } else if (currentButton1State == 0) {  // Button 1 held down
    x = xOffset; y = yOffset; z = zOffset;
  } else {
    mag.getMagneticField(&x, &y, &z);
  }
  
  lastButton1State = currentButton1State;

  // Simple offset correction (no Kalman filters) with Y-axis inversion
  xCurrent = x - xOffset;
  yCurrent = -(y - yOffset);  // Inverted Y-axis
  zCurrent = z - zOffset;
  
  // Debug: Show what values are being used for movement detection (simplified)
  static bool lastDebugButton1State = true;
  if (currentButton1State == 0 && lastDebugButton1State == 1) {  // Button 1 just pressed
    Serial.println("Button 1 pressed - using offset values (no movement)");
  }
  lastDebugButton1State = currentButton1State;

    // Check for orbit mode based on Z-axis (like working examples)
  bool shouldOrbit = abs(zCurrent) > zThreshold;
  
  // Improved movement detection based on working examples
  bool aboveThreshold = (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold);
  
  if (aboveThreshold) {
    
    if (!wasMoving) {
      Serial.print("Movement started - X: "); Serial.print(xCurrent, 3);
      Serial.print(" Y: "); Serial.print(yCurrent, 3);
      Serial.print(" Z: "); Serial.print(zCurrent, 3);
      Serial.print(" Orbit: "); Serial.print(shouldOrbit ? "YES" : "NO");
      Serial.print(" Threshold: "); Serial.println(xyThreshold);
      wasMoving = true;
    }

    // Simple mapping (no complex calculations) with increased sensitivity
    int xMove = (int)(xCurrent * sensitivity);
    int yMove = (int)(yCurrent * sensitivity);
    int zMove = (int)(zCurrent * sensitivity / 2);  // Reduced Z sensitivity to prevent jumpiness

    // Limit movement range
    xMove = constrain(xMove, -127, 127);
    yMove = constrain(yMove, -127, 127);
    zMove = constrain(zMove, -127, 127);
    
    // Debug movement values
    if (count % 100 == 0) {  // Show every 100th movement
      Serial.print("Movement values - X: "); Serial.print(xMove);
      Serial.print(" Y: "); Serial.print(yMove);
      Serial.print(" Z: "); Serial.print(zMove);
      Serial.print(" Orbit: "); Serial.println(shouldOrbit ? "YES" : "NO");
    }
    
    // Send movement with proper 3DConnexion compatible reports
    if (usb_hid.ready()) {
      // Create 6-axis 3D mouse report data (X,Y,Z,Rx,Ry,Rz)
      uint8_t report_data[6];
      report_data[0] = (uint8_t)(xMove/2);  // X axis (translation)
      report_data[1] = (uint8_t)(shouldOrbit ? -yMove/2 : yMove/2);  // Y axis (translation)
      report_data[2] = (uint8_t)(shouldOrbit ? 0 : zMove/4);  // Z axis (translation)
      report_data[3] = (uint8_t)(xMove/4);  // Rx axis (rotation X)
      report_data[4] = (uint8_t)(yMove/4);  // Ry axis (rotation Y)
      report_data[5] = (uint8_t)(zMove/4);  // Rz axis (rotation Z)
      
      // Send the 3DConnexion compatible report
      usb_hid.sendReport(2, report_data, sizeof(report_data));
    }

  } else {
    if (wasMoving) {
      Serial.println("Movement stopped");
      wasMoving = false;
    }
    // No buttons to release since we're not using drag mode
  }

  // Simple monitoring
  if (count % 1000 == 0) {
    Serial.print("Count: "); Serial.println(count);
    Serial.print("Uptime: "); Serial.print(millis() / 1000); Serial.println(" seconds");
    Serial.print("Free memory: "); Serial.print(rp2040.getFreeHeap()); Serial.println(" bytes");
    Serial.println("---");
  }

  count++;
  
  // Debug loop execution
  if (count % 100 == 0) {
    Serial.print("Loop count: "); Serial.println(count);
  }
  
  delay(20); // 50Hz loop rate
} 