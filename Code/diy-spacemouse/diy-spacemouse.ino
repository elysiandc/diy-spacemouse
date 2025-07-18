#include <TinyUSB_Mouse_and_Keyboard.h>
#include <OneButton.h>
#include <Tlv493d.h>
#include <SimpleKalmanFilter.h>

// USB HID Report Descriptor for 3D Mouse
const uint8_t desc_hid_report[] = {
  TUD_HID_REPORT_DESC_3D_MOUSE()
};

// USB Device Descriptor
const char* manufacturer = "3Dconnexion";
const char* product = "SpaceMouse";
const char* serial = "1234567890";

Tlv493d mag = Tlv493d();
SimpleKalmanFilter xFilter(1, 1, 0.2), yFilter(1, 1, 0.2), zFilter(1, 1, 0.2);
SimpleKalmanFilter rxFilter(1, 1, 0.2), ryFilter(1, 1, 0.2), rzFilter(1, 1, 0.2);

// Setup buttons
OneButton button1(27, true);
OneButton button2(24, true);

float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;
float rxCurrent = 0, ryCurrent = 0, rzCurrent = 0;

int calSamples = 300;
int sensitivity = 8;
int magRange = 3;
int outRange = 127;      // Max allowed in HID report
float xyThreshold = 0.4; // Center threshold

int inRange = magRange * sensitivity;
float zThreshold = xyThreshold * 1.5;

// 3D Mouse HID Report
struct {
  int8_t x, y, z;        // Translation
  int8_t rx, ry, rz;     // Rotation
  uint8_t buttons;       // Button states
} mouseReport;

void setup()
{
  // Initialize USB HID
  USBDevice.setManufacturerString(manufacturer);
  USBDevice.setProductString(product);
  USBDevice.setSerialString(serial);
  
  button1.attachClick(goHome);
  button1.attachLongPressStop(goHome);

  button2.attachClick(fitToScreen);
  button2.attachLongPressStop(fitToScreen);

  // mouse and keyboard init
  Mouse.begin();
  Keyboard.begin();

  Serial.begin(9600);
  Wire1.begin();

  // mag sensor init
  mag.begin(Wire1);
  mag.setAccessMode(mag.MASTERCONTROLLEDMODE);
  mag.disableTemp();

  // Calibration
  calibrateSensor();
}

void calibrateSensor()
{
  for (int i = 1; i <= calSamples; i++)
  {
    delay(mag.getMeasurementDelay());
    mag.updateData();

    xOffset += mag.getX();
    yOffset += mag.getY();
    zOffset += mag.getZ();

    Serial.print(".");
  }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;

  Serial.println("\nCalibration complete:");
  Serial.print("X offset: "); Serial.println(xOffset);
  Serial.print("Y offset: "); Serial.println(yOffset);
  Serial.print("Z offset: "); Serial.println(zOffset);
}

void loop()
{
  button1.tick();
  button2.tick();

  delay(mag.getMeasurementDelay());
  mag.updateData();

  // Update translation filters
  xCurrent = xFilter.updateEstimate(mag.getX() - xOffset);
  yCurrent = yFilter.updateEstimate(mag.getY() - yOffset);
  zCurrent = zFilter.updateEstimate(mag.getZ() - zOffset);

  // Calculate rotation (simplified - can be improved with additional sensors)
  rxCurrent = rxFilter.updateEstimate(xCurrent * 0.5);
  ryCurrent = ryFilter.updateEstimate(yCurrent * 0.5);
  rzCurrent = rzFilter.updateEstimate(zCurrent * 0.5);

  // Prepare HID report
  if (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold || abs(zCurrent) > zThreshold)
  {
    // Map values to HID report range
    mouseReport.x = map(xCurrent, -inRange, inRange, -outRange, outRange);
    mouseReport.y = map(yCurrent, -inRange, inRange, -outRange, outRange);
    mouseReport.z = map(zCurrent, -inRange, inRange, -outRange, outRange);
    
    mouseReport.rx = map(rxCurrent, -inRange, inRange, -outRange, outRange);
    mouseReport.ry = map(ryCurrent, -inRange, inRange, -outRange, outRange);
    mouseReport.rz = map(rzCurrent, -inRange, inRange, -outRange, outRange);
    
    // Send HID report
    Mouse.move(mouseReport.x, mouseReport.y, mouseReport.z);
    
    // For applications that support 3D mouse input directly
    if (abs(zCurrent) < zThreshold)
    {
      Keyboard.press(KEY_LEFT_SHIFT);
    }
  }
  else
  {
    Mouse.release(MOUSE_MIDDLE);
    Keyboard.releaseAll();
  }

  // Debug output
  Serial.print("X:"); Serial.print(xCurrent);
  Serial.print(" Y:"); Serial.print(yCurrent);
  Serial.print(" Z:"); Serial.print(zCurrent);
  Serial.print(" RX:"); Serial.print(rxCurrent);
  Serial.print(" RY:"); Serial.print(ryCurrent);
  Serial.print(" RZ:"); Serial.println(rzCurrent);
}

// go to home view in Fusion 360 by pressing  (CMD + SHIFT + H) shortcut assigned to the custom Add-in command
void goHome()
{
  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.write('h');

  delay(10);
  Keyboard.releaseAll();
  Serial.println("pressed home");
}

// fit to view by pressing the middle mouse button twice
void fitToScreen()
{
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);

  Serial.println("pressed fit");
}
