#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SHT31.h>
#include <PDM.h> // For the PDM Microphone

/* 
 *  Bluefruit Sense Sensor Test
 *  
 *  This sketch initializes all onboard sensors and prints their 
 *  readings to the Serial Monitor. Bluetooth is disabled.
 */

// --------------------------------------------------------------------------
//  SENSOR OBJECTS
// --------------------------------------------------------------------------
Adafruit_LSM6DS33 lsm6ds33; // Accelerometer & Gyroscope
Adafruit_LIS3MDL lis3mdl;   // Magnetometer
Adafruit_APDS9960 apds9960; // Proximity, Light, RGB, Gesture
Adafruit_BMP280 bmp280;     // Pressure & Temperature
Adafruit_SHT31 sht30 = Adafruit_SHT31(); // Humidity & Temperature

// Microphone variables
short sampleBuffer[256];
volatile int samplesRead;

// --------------------------------------------------------------------------
//  SETUP FUNCTION
// --------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Wait for Serial Monitor to open
  while (!Serial) delay(10); 

  Serial.println("Bluefruit Sense Sensor Test");
  Serial.println("---------------------------");

  // 1. Initialize LSM6DS33 (Accel/Gyro)
  Serial.print("Initializing LSM6DS33... ");
  if (!lsm6ds33.begin_I2C()) {
    Serial.println("Failed!");
  } else {
    Serial.println("OK!");
  }

  // 2. Initialize LIS3MDL (Magnetometer)
  Serial.print("Initializing LIS3MDL... ");
  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed!");
  } else {
    Serial.println("OK!");
  }

  // 3. Initialize APDS9960 (Light/Color/Proximity)
  Serial.print("Initializing APDS9960... ");
  if (!apds9960.begin()) {
    Serial.println("Failed!");
  } else {
    Serial.println("OK!");
    apds9960.enableColor(true); // Enable color sensing
    apds9960.enableProximity(true); // Enable proximity sensing
  }

  // 4. Initialize BMP280 (Pressure)
  Serial.print("Initializing BMP280... ");
  if (!bmp280.begin()) {
    Serial.println("Failed!");
  } else {
    Serial.println("OK!");
  }

  // 5. Initialize SHT30 (Humidity)
  Serial.print("Initializing SHT30... ");
  if (!sht30.begin(0x44)) {
    Serial.println("Failed!");
  } else {
    Serial.println("OK!");
  }

  // 6. Initialize PDM Microphone
  Serial.print("Initializing PDM Mic... ");
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) { // 1 channel, 16kHz sample rate
    Serial.println("Failed!");
  } else {
    Serial.println("OK!");
  }

  Serial.println("---------------------------");
  Serial.println("All sensors initialized. Starting loop...");
  delay(1000);
}

// --------------------------------------------------------------------------
//  LOOP FUNCTION
// --------------------------------------------------------------------------
void loop() {
  Serial.println("\n--- Sensor Readings ---");

  // --- LSM6DS33 (Motion) ---
  sensors_event_t accel, gyro, temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

  Serial.print("Accel X: "); Serial.print(accel.acceleration.x);
  Serial.print(" \tY: "); Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: "); Serial.print(accel.acceleration.z); 
  Serial.println(" m/s^2");

  Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x);
  Serial.print(" \tY: "); Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: "); Serial.print(gyro.gyro.z); 
  Serial.println(" rad/s");

  // --- LIS3MDL (Magnetometer) ---
  sensors_event_t mag; 
  lis3mdl.getEvent(&mag);

  Serial.print("Mag   X: "); Serial.print(mag.magnetic.x);
  Serial.print(" \tY: "); Serial.print(mag.magnetic.y);
  Serial.print(" \tZ: "); Serial.print(mag.magnetic.z); 
  Serial.println(" uTesla");

  // --- APDS9960 (Light/Color) ---
  uint16_t r, g, b, c;
  apds9960.getColorData(&r, &g, &b, &c);
  uint8_t proximity = apds9960.readProximity();

  Serial.print("Color R: "); Serial.print(r);
  Serial.print(" G: "); Serial.print(g);
  Serial.print(" B: "); Serial.print(b);
  Serial.print(" Clear: "); Serial.println(c);
  Serial.print("Proximity: "); Serial.println(proximity);

  // --- BMP280 (Pressure/Altitude) ---
  float pressure = bmp280.readPressure();
  // Standard sea level pressure is ~1013.25 hPa
  float altitude = bmp280.readAltitude(1013.25); 

  Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" Pa");
  Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");

  // --- SHT30 (Temp/Humidity) ---
  float t = sht30.readTemperature();
  float h = sht30.readHumidity();

  if (!isnan(t)) {
    Serial.print("Temp: "); Serial.print(t); Serial.println(" *C");
  } else {
    Serial.println("Temp: Error reading SHT30");
  }
  
  if (!isnan(h)) {
    Serial.print("Humidity: "); Serial.print(h); Serial.println(" %");
  } else {
    Serial.println("Humidity: Error reading SHT30");
  }

  // --- PDM Microphone (Sound Level) ---
  // Calculate RMS (Root Mean Square) amplitude
  long sumSquares = 0;
  for (int i = 0; i < samplesRead; i++) {
    sumSquares += sampleBuffer[i] * sampleBuffer[i];
  }
  // Avoid division by zero if no samples read
  if (samplesRead > 0) {
    float rms = sqrt(sumSquares / samplesRead);
    Serial.print("Sound Level (RMS): "); Serial.println(rms);
  }
  
  // Clear samples read for next batch
  samplesRead = 0;

  delay(2000); // Wait 2 seconds before next reading
}

// --------------------------------------------------------------------------
//  HELPER FUNCTIONS
// --------------------------------------------------------------------------

// Callback function for PDM microphone
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
