#include <TinyUSB_Mouse_and_Keyboard.h>
#include <OneButton.h>
#include <TLx493D_inc.hpp>
#include <SimpleKalmanFilter.h>

// Version information
const char* VERSION = "1.0.0";
const char* BUILD_DATE = __DATE__;
const char* BUILD_TIME = __TIME__;



// TODO: Optimize Kalman filter parameters for better responsiveness
// Kalman filter parameters - adjusted for better responsiveness
// Parameters: (measurement noise, estimation uncertainty, process variance)
// measurement noise = 0.2 (reduced to be more responsive to real movement)
// estimation uncertainty = 0.2 (same as measurement noise)
// process variance = 0.02 (increased to respond faster to changes)
SimpleKalmanFilter xFilter(0.2, 0.2, 0.02), yFilter(0.2, 0.2, 0.02), zFilter(0.2, 0.2, 0.02);
SimpleKalmanFilter rxFilter(0.2, 0.2, 0.02), ryFilter(0.2, 0.2, 0.02), rzFilter(0.2, 0.2, 0.02);

// TODO: Verify button pin assignments and test responsiveness
// Setup buttons - using QT Py RP2040 pins
OneButton button1(27, true);  // Changed from 27 to 9
OneButton button2(24, true); // Changed from 24 to 10

// Define offet ranges
double xOffset = 0, yOffset = 0, zOffset = 0;
double xCurrent = 0, yCurrent = 0, zCurrent = 0;
double rxCurrent = 0, ryCurrent = 0, rzCurrent = 0;  // Rotation values
double xLast = 0, yLast = 0, zLast = 0;  // Track last values for drift detection
double xRaw = 0, yRaw = 0, zRaw = 0;     // Store raw values

// Calculate origin/home position and desired range of motion
int calSamples = 300;
int sensitivity = 12;     // Increased from 8 to 12 for more responsive movement
int magRange = 1.7;      // Set desired range of motion
int outRange = 127;      // Max allowed in HID report
double xyThreshold = 0.2; // Reduced from 0.3 to 0.2 for earlier movement detection
double deadzone = 0.05;  // Reduced from 0.1 to 0.05 for more precise control
double driftThreshold = 0.05; // Maximum allowed change between readings
double rotationScale = 0.5;   // Scale factor for rotation values

int inRange = magRange * sensitivity;
double zThreshold = xyThreshold * 1.5;

bool isOrbit = false;
bool wasMoving = false;  // Track previous movement state
bool isButtonPressed = false;  // Track button state

// 3D Mouse HID Report
struct {
  int8_t x, y, z;        // Translation
  int8_t rx, ry, rz;     // Rotation
  uint8_t buttons;       // Button states
} mouseReport;

/* Setup Mag sensor with XENSIV library with hangup recovery
Based on Burke111-DEV's fix: https://github.com/Burke111-DEV/TLV493D-3DMagnetic-Sensor-With-Hangup-Recovery */
using namespace ifx::tlx493d;

const uint8_t POWER_PIN = 15;
TLx493D_A1B6 mag(Wire, TLx493D_IIC_ADDR_A0_e);

/** Definition of a counter variable. */
uint8_t count = 0;

// ✅ IMPROVED: Fine-tuned drift detection algorithm
// Drift detection parameters - optimized for better stability
const float DRIFT_THRESHOLD = 0.3;  // Reduced from 0.5 for more sensitive detection
const int DRIFT_COUNT_THRESHOLD = 3;  // Reduced from 5 for faster response
const int DRIFT_RECOVERY_THRESHOLD = 2;  // Number of good readings to recover
const float DRIFT_SMOOTHING = 0.1;  // Smoothing factor for drift detection
bool driftDetected = false;
int driftCount = 0;
int goodReadingsCount = 0;
float lastGoodX = 0, lastGoodY = 0, lastGoodZ = 0;
float lastGoodRX = 0, lastGoodRY = 0, lastGoodRZ = 0;
float driftSmoothedX = 0, driftSmoothedY = 0, driftSmoothedZ = 0;

// Hangup recovery parameters - FIX FOR 30-SECOND FREEZING ISSUE
// Based on Burke111-DEV's solution: https://github.com/Burke111-DEV/TLV493D-3DMagnetic-Sensor-With-Hangup-Recovery
unsigned long lastMagUpdate = 0;
const unsigned long MAG_TIMEOUT = 2000; // 2 second timeout for sensor reads
unsigned long lastLoopTime = 0;

// Button callback functions
void button1Click() {
  Serial.println("Button 1 clicked");
  isOrbit = !isOrbit;
  Serial.println(isOrbit ? "Switched to orbit mode" : "Switched to movement mode");
}

void button2Click() {
  Serial.println("Button 2 clicked");
  // Reset drift detection
  driftDetected = false;
  driftCount = 0;
  lastGoodX = 0;
  lastGoodY = 0;
  lastGoodZ = 0;
  lastGoodRX = 0;
  lastGoodRY = 0;
  lastGoodRZ = 0;
  Serial.println("Drift detection reset");
}

// Double-click handlers for device reset
void button1DoubleClick() {
  Serial.println("Button 1 double-clicked - resetting device...");
  delay(100);
  rp2040.restart();
}

void button2DoubleClick() {
  Serial.println("Button 2 double-clicked - resetting device...");
  delay(100);
  rp2040.restart();
}

// Function to reset sensor when hangup is detected
void resetSensor() {
  Serial.println("Resetting sensor due to hangup...");
  
  mag.end();
  mag.begin();
  delay(100);
  
  Serial.println("Sensor reset complete");
}

void setup() 
{
  // Initialize USB HID
  Mouse.begin();
  Keyboard.begin();

  button1.attachClick(button1Click);
  button1.attachDoubleClick(button1DoubleClick);
  button1.attachLongPressStop(button1Click);

  button2.attachClick(button2Click);
  button2.attachDoubleClick(button2DoubleClick);
  button2.attachLongPressStop(button2Click);

  Serial.begin(115200);
  delay(1000);
  
  // Print version information
  Serial.println("=== SpaceMouse Startup ===");
  Serial.print("Version: "); Serial.println(VERSION);
  Serial.print("Build Date: "); Serial.println(BUILD_DATE);
  Serial.print("Build Time: "); Serial.println(BUILD_TIME);
  Serial.println("==========================");

  /** Initialize sensor with hangup recovery enabled */
  mag.begin();

// crude offset calibration on first boot
  for (int i = 1; i <= calSamples; i++)
  {
    double x, y, z;

  mag.getMagneticField(&x, &y, &z);
  //mag.printRegisters();

    xOffset += x;
    yOffset += y;
    zOffset += z;

    Serial.print(".");
  }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;

  Serial.println();
  Serial.println(xOffset);
  Serial.println(yOffset);
  Serial.println(zOffset);

  Serial.println("TLV493 setup done.");

  delay(6000);
  
  // Initialize hangup detection
  lastMagUpdate = millis();
}

void loop() 
{
  // Check for sensor hangup (more conservative approach)
  unsigned long currentTime = millis();
  if (currentTime - lastMagUpdate > MAG_TIMEOUT) {
    Serial.println("Sensor hangup detected - attempting reset...");
    Serial.print("Time since last sensor read: "); Serial.print(currentTime - lastMagUpdate); Serial.println("ms");
    resetSensor();
    lastMagUpdate = currentTime; // Reset the timer
  }
  
  // Add hardware diagnostics
  if (count > 0 && count % 500 == 0) {
    Serial.println("Watchdog check - device still alive");
    Serial.print("Free memory: "); Serial.print(rp2040.getFreeHeap()); Serial.println(" bytes");
    Serial.print("CPU temperature: "); Serial.print(rp2040.cpuid()); Serial.println(" (arbitrary units)");
  }
  
  lastLoopTime = currentTime;
  
  // Update button states
  button1.tick();
  button2.tick();

  double x, y, z;

  // Get magnetic field values with timeout protection
  unsigned long readStart = millis();
  
  // Add a simple timeout check before sensor read
  if (millis() - lastMagUpdate > 5000) {  // If sensor hasn't been read in 5 seconds
    Serial.println("Sensor timeout - resetting sensor");
    resetSensor();
    lastMagUpdate = millis();
  }
  
  mag.getMagneticField(&x, &y, &z);
  unsigned long readTime = millis() - readStart;
  
  if (readTime > 100) {
    Serial.print("Slow sensor read: "); Serial.print(readTime); Serial.println("ms");
  }
  
  lastMagUpdate = millis();

  // Store last values before updating
  xLast = xCurrent;
  yLast = yCurrent;
  zLast = zCurrent;
  xRaw = x;
  yRaw = y;
  zRaw = z;

  // Update translation filters
  yCurrent = xFilter.updateEstimate(x - xOffset);
  xCurrent = -yFilter.updateEstimate(y - yOffset);
  zCurrent = zFilter.updateEstimate(z - zOffset);

  // Calculate rotation values (derived from translation)
  rxCurrent = rxFilter.updateEstimate(xCurrent * rotationScale);
  ryCurrent = ryFilter.updateEstimate(yCurrent * rotationScale);
  rzCurrent = rzFilter.updateEstimate(zCurrent * rotationScale);

  // ✅ IMPROVED: Enhanced drift detection with smoothing and better recovery
  if (!driftDetected) {
    // Apply smoothing to current values
    driftSmoothedX = driftSmoothedX * (1 - DRIFT_SMOOTHING) + xCurrent * DRIFT_SMOOTHING;
    driftSmoothedY = driftSmoothedY * (1 - DRIFT_SMOOTHING) + yCurrent * DRIFT_SMOOTHING;
    driftSmoothedZ = driftSmoothedZ * (1 - DRIFT_SMOOTHING) + zCurrent * DRIFT_SMOOTHING;
    
    float deltaX = abs(driftSmoothedX - lastGoodX);
    float deltaY = abs(driftSmoothedY - lastGoodY);
    float deltaZ = abs(driftSmoothedZ - lastGoodZ);
    float deltaRX = abs(rxCurrent - lastGoodRX);
    float deltaRY = abs(ryCurrent - lastGoodRY);
    float deltaRZ = abs(rzCurrent - lastGoodRZ);

    // Check if any axis exceeds drift threshold
    if (deltaX > DRIFT_THRESHOLD || deltaY > DRIFT_THRESHOLD || deltaZ > DRIFT_THRESHOLD ||
        deltaRX > DRIFT_THRESHOLD || deltaRY > DRIFT_THRESHOLD || deltaRZ > DRIFT_THRESHOLD) {
      driftCount++;
      Serial.print("Drift count: "); Serial.println(driftCount);
      
      if (driftCount >= DRIFT_COUNT_THRESHOLD) {
        driftDetected = true;
        Serial.println("Drift detected - using last good values");
      }
    } else {
      // Good reading - update last good values
      driftCount = 0;
      lastGoodX = driftSmoothedX;
      lastGoodY = driftSmoothedY;
      lastGoodZ = driftSmoothedZ;
      lastGoodRX = rxCurrent;
      lastGoodRY = ryCurrent;
      lastGoodRZ = rzCurrent;
    }
  } else {
    // Try to recover from drift with improved logic
    float deltaX = abs(xCurrent - lastGoodX);
    float deltaY = abs(yCurrent - lastGoodY);
    float deltaZ = abs(zCurrent - lastGoodZ);
    float deltaRX = abs(rxCurrent - lastGoodRX);
    float deltaRY = abs(ryCurrent - lastGoodRY);
    float deltaRZ = abs(rzCurrent - lastGoodRZ);

    // Check if readings are back to normal
    if (deltaX < DRIFT_THRESHOLD && deltaY < DRIFT_THRESHOLD && deltaZ < DRIFT_THRESHOLD &&
        deltaRX < DRIFT_THRESHOLD && deltaRY < DRIFT_THRESHOLD && deltaRZ < DRIFT_THRESHOLD) {
      goodReadingsCount++;
      Serial.print("Good readings count: "); Serial.println(goodReadingsCount);
      
      if (goodReadingsCount >= DRIFT_RECOVERY_THRESHOLD) {
        driftDetected = false;
        driftCount = 0;
        goodReadingsCount = 0;
        Serial.println("Drift recovered - resuming normal operation");
      }
    } else {
      // Reset good readings count if still drifting
      goodReadingsCount = 0;
    }
  }

  // Use last good values if drift detected
  if (driftDetected) {
    xCurrent = lastGoodX;
    yCurrent = lastGoodY;
    zCurrent = lastGoodZ;
    rxCurrent = lastGoodRX;
    ryCurrent = lastGoodRY;
    rzCurrent = lastGoodRZ;
  }

  // Debug output
  Serial.print("Translation - X: ");
  Serial.print(xCurrent);
  Serial.print(", Y: ");
  Serial.print(yCurrent);
  Serial.print(", Z: ");
  Serial.print(zCurrent);
  Serial.print(" | Rotation - RX: ");
  Serial.print(rxCurrent);
  Serial.print(", RY: ");
  Serial.print(yCurrent);
  Serial.print(", RZ: ");
  Serial.println(rzCurrent);

  Serial.print("count : ");
  Serial.println(count);
  
  // Monitor for sensor hangup without using watchdog
  if (count % 100 == 0) {  // More frequent health checks
    Serial.println("=== Health Check ===");
    Serial.print("Count: "); Serial.println(count);
    Serial.print("Uptime: "); Serial.print(millis() / 1000); Serial.println(" seconds");
    Serial.print("Last sensor read: "); Serial.print(millis() - lastMagUpdate); Serial.println("ms ago");
    Serial.println("==================");
  }
  
  // Debug loop timing
  if (count % 100 == 0) {
    Serial.print("Loop time: "); Serial.print(millis() - lastLoopTime); Serial.println("ms");
  }
  
  ++count;

  // Add debug output for thresholds
  Serial.print("Thresholds - XY: ");
  Serial.print(xyThreshold);
  Serial.print(", Z: ");
  Serial.print(zThreshold);
  Serial.print(", Current XY: ");
  Serial.print(abs(xCurrent));
  Serial.print(", ");
  Serial.print(abs(yCurrent));
  Serial.println();

  // check the center threshold using filtered values and deadzone
  if (abs(xCurrent) > (xyThreshold + deadzone) || abs(yCurrent) > (xyThreshold + deadzone) || 
      abs(zCurrent) > (zThreshold + deadzone))
  {
    if (!wasMoving) {
      Serial.println("Threshold exceeded - starting movement");
      wasMoving = true;
    }

    // Map values to HID report range with increased sensitivity
    int xMove = map(xCurrent, -inRange, inRange, -outRange, outRange) * 2;  // Multiply by 2 for more movement
    int yMove = map(yCurrent, -inRange, inRange, -outRange, outRange) * 2;
    int zMove = map(zCurrent, -inRange, inRange, -outRange, outRange) * 2;
    int rxMove = map(rxCurrent, -inRange, inRange, -outRange, outRange);
    int ryMove = map(ryCurrent, -inRange, inRange, -outRange, outRange);
    int rzMove = map(rzCurrent, -inRange, inRange, -outRange, outRange);

    // ✅ IMPROVED: Optimized movement scaling with adaptive sensitivity
    // Send movement with improved sensitivity and adaptive scaling
    int scaleFactor = 2;  // Reduced from 4 for more responsive movement
    
    // Adaptive scaling based on movement magnitude
    float movementMagnitude = sqrt(xMove*xMove + yMove*yMove + zMove*zMove);
    if (movementMagnitude > 50) {
      scaleFactor = 3;  // Slower for large movements
    } else if (movementMagnitude < 10) {
      scaleFactor = 1;  // Faster for small movements
    }
    
    // Prepare HID report with proper 3D mouse structure
    mouseReport.x = xMove/scaleFactor;
    mouseReport.y = yMove/scaleFactor;
    mouseReport.z = zMove/scaleFactor;
    mouseReport.rx = rxMove/scaleFactor;
    mouseReport.ry = ryMove/scaleFactor;
    mouseReport.rz = rzMove/scaleFactor;
    
    if (isOrbit) {
      Keyboard.press(KEY_LEFT_SHIFT);
      Mouse.press(MOUSE_MIDDLE);
      Mouse.move(mouseReport.y, mouseReport.x, 0);
    } else {
      Mouse.press(MOUSE_MIDDLE);
      Mouse.move(mouseReport.x, mouseReport.y, mouseReport.z);
    }

    // Debug movement values
    Serial.print("Mapped moves - X: ");
    Serial.print(xMove);
    Serial.print(", Y: ");
    Serial.print(yMove);
    Serial.print(", Z: ");
    Serial.print(zMove);
    Serial.print(" | RX: ");
    Serial.print(rxMove);
    Serial.print(", RY: ");
    Serial.print(ryMove);
    Serial.print(", RZ: ");
    Serial.println(rzMove);
  } else {
    if (wasMoving) {
      Serial.println("Below threshold - stopping movement");
      wasMoving = false;
    }
    Mouse.release(MOUSE_MIDDLE);
    Keyboard.releaseAll();
  }

  delay(50); // Reduced delay for better responsiveness
}

// go to home view in Fusion 360 by pressing (CMD + SHIFT + H) shortcut assigned to the custom Add-in command
void goHome()
{
  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.write('h');
  delay(10);
  Keyboard.releaseAll();
  Serial.println("pressed home");
  delay(1000);
}

// fit to view by pressing the middle mouse button twice
void fitToScreen()
{
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);
  Serial.println("pressed fit");
  delay(1000);
}