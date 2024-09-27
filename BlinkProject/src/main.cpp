#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BluetoothSerial.h> 
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "SD.h"                   
#include "DFRobotDFPlayerMini.h" 


// WiFi credentials
const char* ssid = "Xiaomi14Ultra";
const char* password = "zwyzs123";

// Gemini AI API endpoint and key
const char* apiEndpoint = "https://generativelanguage.googleleapis.com/v1beta/models/gemini-pro:generateContent?key=AIzaSyBXbK9bcQSj-pLpWyoNV9dNMa9PrhSKj38";
const char* apiKey = "AIzaSyBXbK9bcQSj-pLpWyoNV9dNMa9PrhSKj38";

#define LIGHT_SENSOR_PIN_ENV 34  // 环境光传感器引脚
#define LIGHT_SENSOR_PIN_HAT 35  // 帽子检测光传感器引脚
#define SD_CARD_CS 5              // SD卡的CS引脚
#define SPEAKER_PIN 25            // 扬声器引脚
#define LED_PIN 15                // LED灯带数据引脚
#define NUM_LEDS 30               // LED灯带上LED的数量
#define DFPLAYER_TX 26            // DFPlayer TX引脚
#define DFPLAYER_RX 27            // DFPlayer RX引脚

Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
SoftwareSerial mySerial(DFPLAYER_RX, DFPLAYER_TX); // 创建软件串口
DFRobotDFPlayerMini myDFPlayer;           // DFPlayer Mini对象

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

  // SD卡初始化
  if (!SD.begin(SD_CARD_CS)) {
    Serial.println("SD 卡初始化失败");
    return;
  }
  Serial.println("SD 卡初始化成功");

    // DFPlayer Mini初始化
  mySerial.begin(9600);                   // 初始化软件串口
  if (!myDFPlayer.begin(mySerial)) {      // 初始化DFPlayer Mini
    Serial.println("DFPlayer Mini 初始化失败");
    while (true);
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  SerialBT.begin("Hat_BT");
}
void playActionPrompt(String action) {
    if (action == "jump") {
        myDFPlayer.play(1);                 // 播放SD卡上的第1个音频文件（跳跃提示）
    } else if (action == "run") {
        myDFPlayer.play(2);                 // 播放SD卡上的第2个音频文件（跑步提示）
    }
}

void playHatOnPrompt() {
    myDFPlayer.play(3);                     // 播放SD卡上的第3个音频文件（戴上帽子的提示）
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

    // 检测帽子是否戴上
  if (lightValueHat < 500) {
      if (!hatOn) {
          hatOn = true;
          playHatOnPrompt();  // 提示戴上帽子后开始任务
          delay(3000);        // 等待提示音播放完成

          // 提示第一个任务
          playActionPrompt("jump");  // 提示开始跳跃任务
      }
  } else {
      hatOn = false;  // 如果帽子未戴上，重置状态
  }

  // Control lights based on ambient light
  if (lightValueEnv < 500) {  // If the environment is dark
    if (!isDark) {  // Only execute the first time the environment becomes dark
      isDark = true;
      turnOnAllWhite();  // Turn all LEDs white
    }
  } else {
    isDark = false;
    updateLeds();  // Show colored lights
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
  // 处理DFPlayer Mini
  myDFPlayer.available();  // 确保DFPlayer Mini可以处理音频播放
  delay(100);
}