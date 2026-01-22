#include <Wire.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_NeoPixel.h>

/* 
 *  Bluefruit Sense - Gyro Visualization via NeoPixel
 *  
 *  - Reads Gyroscope (Rotation) data.
 *  - Changes the onboard NeoPixel color based on rotation direction.
 *  - X Axis: Red / Orange
 *  - Y Axis: Yellow / Green
 *  - Z Axis: Blue / Purple
 */

// --- Configuration ---
#define NEOPIXEL_PIN    8    // Pin 8 is standard for Bluefruit Sense NeoPixel
#define NUM_PIXELS      1
#define BRIGHTNESS      20   // 0-255 (Keep low to save eyes/power)
#define SENSITIVITY     0.5  // Threshold (rad/s) to trigger a color change. 
                             // Increase if it flickers too much, decrease for more sensitivity.

// --- Objects ---
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);

  // Initialize NeoPixel
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show(); // Initialize all pixels to 'off'

  // Initialize Sensor
  Serial.println("Initializing LSM6DS33...");
  if (!lsm6ds33.begin_I2C()) {
    Serial.println("Failed to find LSM6DS33 chip");
    // Flash Red if sensor fails
    while (1) { 
      strip.setPixelColor(0, strip.Color(255, 0, 0)); 
      strip.show(); delay(100);
      strip.setPixelColor(0, 0); 
      strip.show(); delay(100);
    }
  }
  Serial.println("LSM6DS33 Found!");
  
  // Set Gyro Range (500 degrees per second is good for hand gestures)
  lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
}

void loop() {
  sensors_event_t accel, gyro, temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

  float x = gyro.gyro.x;
  float y = gyro.gyro.y;
  float z = gyro.gyro.z;

  // Print values for debugging (Serial Plotter compatible)
  Serial.print("X:"); Serial.print(x); Serial.print("\t");
  Serial.print("Y:"); Serial.print(y); Serial.print("\t");
  Serial.print("Z:"); Serial.println(z);

  // --- Logic to determine Color ---
  // We want to find which axis has the STRONGEST rotation.
  // We use abs() to get the magnitude regardless of direction.
  
  float absX = abs(x);
  float absY = abs(y);
  float absZ = abs(z);

  uint32_t color = 0; // Default to black (off)

  // Check if movement is significant enough (above sensitivity threshold)
  if (absX > SENSITIVITY || absY > SENSITIVITY || absZ > SENSITIVITY) {
    
    // Determine which axis is dominant
    if (absX > absY && absX > absZ) {
      // X-Axis is dominant
      if (x > 0) {
        color = strip.Color(255, 0, 0);    // Red
      } else {
        color = strip.Color(255, 100, 0);  // Orange
      }
    } 
    else if (absY > absX && absY > absZ) {
      // Y-Axis is dominant
      if (y > 0) {
        color = strip.Color(255, 255, 0);  // Yellow
      } else {
        color = strip.Color(0, 255, 0);    // Green
      }
    } 
    else {
      // Z-Axis is dominant
      if (z > 0) {
        color = strip.Color(0, 0, 255);    // Blue
      } else {
        color = strip.Color(128, 0, 128);  // Purple
      }
    }
  } else {
    // If barely moving, turn off or show faint white
    color = strip.Color(0, 0, 0); 
  }

  strip.setPixelColor(0, color);
  strip.show();

  delay(50); // Small delay for stability
}
