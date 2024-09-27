#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BluetoothSerial.h> 
#include <Adafruit_NeoPixel.h>
// WiFi credentials
const char* ssid = "Xiaomi14Ultra";
const char* password = "zwyzs123";

// Gemini AI API endpoint and key
const char* apiEndpoint = "https://generativelanguage.googleleapis.com/v1beta/models/gemini-pro:generateContent?key=AIzaSyBXbK9bcQSj-pLpWyoNV9dNMa9PrhSKj38";
const char* apiKey = "AIzaSyBXbK9bcQSj-pLpWyoNV9dNMa9PrhSKj38";

#define LIGHT_SENSOR_PIN_ENV 34  // Light sensor for ambient light detection
#define LIGHT_SENSOR_PIN_HAT 35  // Light sensor for hat detection
#define SPEAKER_PIN 25           // Speaker pin (DAC1, GPIO 25)
#define LED_PIN 15               // LED strip data pin
#define NUM_LEDS 30              // Number of LEDs on the strip

Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

int distanceTravelled = 0;
int stepCounter = 0;
bool hatOn = false;
int litLeds = 0;  // Number of currently lit LEDs
bool isDark = false;  // To track whether the environment is dark

void setup() {
  Serial.begin(115200);
  pinMode(LIGHT_SENSOR_PIN_ENV, INPUT);
  pinMode(LIGHT_SENSOR_PIN_HAT, INPUT);
  strip.begin();
  strip.show();
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  SerialBT.begin("Hat_BT");
  dacWrite(SPEAKER_PIN, 0); 
}

void playStartRunningMessage() {
  for (int i = 0; i < 255; i++) {
    dacWrite(SPEAKER_PIN, i);
    delay(10);
  }
  delay(500);  // Assume the audio plays for 500ms
  dacWrite(SPEAKER_PIN, 0);
}

// Turn all LEDs white when the environment is dark
void turnOnAllWhite() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(255, 255, 255));  // White color
  }
  strip.show();
}

// Update the LED strip with colored lights based on the number of lit LEDs
void updateLedsWithColor() {
  for (int i = 0; i < litLeds; i++) {
    uint32_t color = strip.Color(0, 0, 255);  // Default to blue

    // Assign different colors to different LED positions (colored strip)
    if (i % 3 == 0) {
      color = strip.Color(255, 0, 0);  // Red
    } else if (i % 3 == 1) {
      color = strip.Color(0, 255, 0);  // Green
    }

    strip.setPixelColor(i, color);
  }
  strip.show();
}

void turnOffBrimLight() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));  // Turn off the lights
  }
  strip.show();
}

// Check if the user is jumping
bool isJumping(float ax, float ay, float az) {
  float totalAcceleration = sqrt(ax * ax + ay * ay + az * az);
  return totalAcceleration > 15;
}

// Calculate distance based on acceleration
int calculateDistance(float ax, float ay, float az) {
  float acceleration = sqrt(ax * ax + ay * ay + az * az);
  int distance = acceleration * 0.05;  // Calculate distance
  return distance;
}

void updateLeds() {
  if (litLeds < strip.numPixels()) {
    litLeds++;  // Increase the number of lit LEDs each time
  }
}

void loop() {
  int lightValueEnv = analogRead(LIGHT_SENSOR_PIN_ENV);  // Read ambient light sensor
  int lightValueHat = analogRead(LIGHT_SENSOR_PIN_HAT);  // Read hat detection light sensor

  // Determine if the hat is on
  if (lightValueHat < 500) {
    if (!hatOn) {
      hatOn = true;
      playStartRunningMessage();
    }
  } else {
    hatOn = false;
  }

  // Control lights based on ambient light
  if (lightValueEnv < 500) {  // If the environment is dark
    if (!isDark) {  // Only execute the first time the environment becomes dark
      isDark = true;
      turnOnAllWhite();  // Turn all LEDs white
    }
  } else {
    isDark = false;
    updateLedsWithColor();  // Show colored lights
  }

  // Get acceleration data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // Detect jump and light up new LEDs
  if (isJumping(ax, ay, az)) {
    stepCounter++;
    if (stepCounter >= 10) {  // Light up a new LED after 10 jumps
      updateLeds();
      playStartRunningMessage();
      stepCounter = 0;
    }
  }

  // Accumulate distance and light up new LEDs
  distanceTravelled += calculateDistance(ax, ay, az);
  if (distanceTravelled > 100) { // Light up a new LED every 100 meters
    updateLeds();
    playStartRunningMessage();
    distanceTravelled = 0;
  }

  // Check for Bluetooth signals
  if (SerialBT.available()) {
    String otherHatSignal = SerialBT.readString();
    if (otherHatSignal == "close") {
      updateLeds();  // Light up a new LED during Bluetooth interaction
      playStartRunningMessage();
    }
  }
  delay(100);
}