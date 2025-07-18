#include <TLx493D_inc.hpp>

// Version information
const char* VERSION = "SENSOR-TEST-1.0.0";

using namespace ifx::tlx493d;
TLx493D_A1B6 mag(Wire, TLx493D_IIC_ADDR_A0_e);

unsigned long count = 0;
unsigned long lastSensorRead = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Sensor Test Startup ===");
  Serial.print("Version: "); Serial.println(VERSION);
  Serial.println("==========================");
  
  // Initialize sensor
  Serial.println("Initializing sensor...");
  mag.begin();
  Serial.println("Sensor initialized");
  
  lastSensorRead = millis();
}

void loop() {
  count++;
  
  // Basic loop monitoring
  if (count % 100 == 0) {
    Serial.print("Count: "); Serial.println(count);
    Serial.print("Uptime: "); Serial.print(millis() / 1000); Serial.println(" seconds");
    Serial.print("Free memory: "); Serial.print(rp2040.getFreeHeap()); Serial.println(" bytes");
    Serial.println("---");
  }
  
  // Sensor read test
  if (count % 50 == 0) {  // Read sensor every 50 cycles
    Serial.print("Reading sensor at count: "); Serial.println(count);
    
    double x, y, z;
    unsigned long readStart = millis();
    
    mag.getMagneticField(&x, &y, &z);
    
    unsigned long readTime = millis() - readStart;
    Serial.print("Sensor read time: "); Serial.print(readTime); Serial.println("ms");
    Serial.print("X: "); Serial.print(x, 3);
    Serial.print(" Y: "); Serial.print(y, 3);
    Serial.print(" Z: "); Serial.println(z, 3);
    
    lastSensorRead = millis();
  }
  
  delay(100);  // 10Hz loop rate
} 