#include <TinyUSB_Mouse_and_Keyboard.h>

// Version information
const char* VERSION = "TEST-1.0.0";

unsigned long count = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Minimal Test Startup ===");
  Serial.print("Version: "); Serial.println(VERSION);
  Serial.println("==========================");
  
  Mouse.begin();
  Keyboard.begin();
  
  lastTime = millis();
}

void loop() {
  count++;
  
  // Basic functionality test
  if (count % 100 == 0) {
    Serial.print("Count: "); Serial.println(count);
    Serial.print("Uptime: "); Serial.print(millis() / 1000); Serial.println(" seconds");
    Serial.print("Free memory: "); Serial.print(rp2040.getFreeHeap()); Serial.println(" bytes");
    Serial.println("---");
  }
  
  // Simple mouse movement test
  if (count % 10000 == 0) {
    Mouse.move(1, 0, 0);  // Move 1 pixel right
    delay(10);
    Mouse.move(-1, 0, 0); // Move 1 pixel left
  }
  
  delay(50);  // 20Hz loop rate
} 