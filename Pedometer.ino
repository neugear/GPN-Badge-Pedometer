#include <Arduino.h>
#include "rboot.h"
#include "rboot-api.h"
#include <FS.h>
#include <Wire.h>
#include <TFT_ILI9163C.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <limits>
#include "Vector3.hpp"

#undef USEWIFI

#define GPIO_BOOT   16
#define GPIO_WS2813 4
#define GPIO_LCD_CS 15
#define GPIO_LCD_DC 0

#define LCD_LED  5

#define MUX_BAT 1

#define BLACK   0x0000

#define UP      720
#define DOWN    570
#define RIGHT   480
#define LEFT    960
#define OFFSET  50

#define I2C_PCA 0x25

#define NUM_LEDS    4

TFT_ILI9163C tft = TFT_ILI9163C(GPIO_LCD_CS, GPIO_LCD_DC);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, GPIO_WS2813, NEO_GRB + NEO_KHZ800);
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ID, BNO055_ADDRESS_B);

byte portExpanderConfig = 0;


////////////////////////////////////////////

const int numReadings = 4;

Vector3f readings[numReadings];
int readIndex = 0;
Vector3f total = Vector3f();
Vector3f average = Vector3f();

Vector3f minValue;
Vector3f maxValue;

Vector3f activity;

float precision = 2.0f;

float threshold = 0.0f;
int mostActiveAxis = 0;

uint8_t sampleCounter = 0;

float newSample = 0.0f;
float oldSample = 0.0f;

int steps = 0;
unsigned long lastMillis = 0;

void setup()
{
  initBadge();

  startScreen();

  bno.begin();
  delay(300);

  rboot_config rboot_config = rboot_get_config();
  SPIFFS.begin();
  File f = SPIFFS.open("/rom" + String(rboot_config.current_rom), "w");
  f.println("Pedometer\n");
  f.close();

  f = SPIFFS.open("/steps.txt", "r");
  if(!f) {
    steps = 0;
    Serial.println("Error");
  }
  else {
    String line = f.readStringUntil('\n');
    steps = line.toInt();
  }
  f.close();

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = Vector3f(0);
  }

  resetMinValue();
  resetMaxValue();

}

void loop() {

  imu::Vector<3> reading = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  total = total - readings[readIndex];

  readings[readIndex] = Vector3f(reading.x(), reading.y(), reading.z());

  total = total + readings[readIndex];

  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  average = total / numReadings;

  for (int axis = 0; axis < 3; axis++) {
    minValue[axis] = std::min(minValue[axis], average[axis]);
    maxValue[axis] = std::max(maxValue[axis], average[axis]);
  }

  sampleCounter++;

  if (sampleCounter >= 50) {

    for (int axis = 0; axis < 3; axis++) {
      activity[axis] = abs(maxValue[axis]) + abs(minValue[axis]);
    }

    if (activity.x >= activity.y) {
      if (activity.x >= activity.z) {
        mostActiveAxis = 0;
      }
      else {
        mostActiveAxis = 2;
      }
    }
    else {
      if (activity.y >= activity.z) {
        mostActiveAxis = 1;
      }
      else {
        mostActiveAxis = 2;
      }
    }

    threshold = (maxValue[mostActiveAxis] + minValue[mostActiveAxis]) / 2;

    precision = activity[mostActiveAxis] * 0.3f + 0.3f;

    resetMinValue();
    resetMaxValue();

    sampleCounter = 0;
  }

  oldSample = newSample;

  if (abs(newSample - average[mostActiveAxis]) >= precision) {
    newSample = average[mostActiveAxis];
  }

  if (newSample < threshold && oldSample > threshold) {
    processStep();
  }

  if (getJoystick() == 5) {
    steps = 0;
    File f = SPIFFS.open("/steps.txt", "w");
    f.println(steps);
    f.close();
  }

//  Serial.print("A: ");
//  Serial.print(average[mostActiveAxis]);
//  Serial.print(" ,T: ");
//  Serial.println(threshold);

  handleScreen();

  delay(1);
}

void resetMinValue() {
  minValue = Vector3f(std::numeric_limits<float>::max());
}

void resetMaxValue() {
  maxValue = Vector3f(std::numeric_limits<float>::min());
}

void processStep() {

  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= 100 || currentMillis - lastMillis <= 1000) {
    steps++;
  }

  lastMillis = currentMillis;

  if (steps % 50 == 0) {
    File f = SPIFFS.open("/steps.txt", "w");
    f.println(steps);
    f.close();
  }
}

void startScreen() {
  tft.fillScreen(0x0000);

  tft.fillRect(0, 55, 127, 18, 0xffff);

  tft.writeFramebuffer();
}

void handleScreen() {
  tft.setTextColor(0x0000, 0xffff);
  tft.setTextSize(2);

  char buffer[10];
  sprintf(buffer, "%10d", steps);

  tft.setCursor(4, 57);
  tft.print(buffer);
  tft.writeFramebuffer();
}

///////////////////////////////////////////////

int getJoystick() {
  uint16_t adc = analogRead(A0);

  if (adc < UP + OFFSET && adc > UP - OFFSET)             return 1;
  else if (adc < DOWN + OFFSET && adc > DOWN - OFFSET)    return 2;
  else if (adc < RIGHT + OFFSET && adc > RIGHT - OFFSET)  return 3;
  else if (adc < LEFT + OFFSET && adc > LEFT - OFFSET)    return 4;
  if (digitalRead(GPIO_BOOT) == 1) return 5;
}

void setGPIO(byte channel, boolean level) {
  bitWrite(portExpanderConfig, channel, level);
  Wire.beginTransmission(I2C_PCA);
  Wire.write(portExpanderConfig);
  Wire.endTransmission();
}

void setAnalogMUX(byte channel) {
  portExpanderConfig = portExpanderConfig & 0b11111000;
  portExpanderConfig = portExpanderConfig | channel;
  Wire.beginTransmission(I2C_PCA);
  Wire.write(portExpanderConfig);
  Wire.endTransmission();
}

uint16_t getBatLvl() {
  if (portExpanderConfig != 33) {
    setAnalogMUX(MUX_BAT);
    delay(20);
  }
  uint16_t avg = 0;
  for (byte i = 0; i < 16; i++) {
    avg += analogRead(A0);
  }
  return (avg / 16);
}

uint16_t getBatVoltage() { //battery voltage in mV
  return (getBatLvl() * 4.8);
}

void initBadge() { //initialize the badge

#ifdef USEIR
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  setGPIO(IR_EN, HIGH);
  irrecv.enableIRIn(); // Start the receiver
  irsend.begin();
#else
  Serial.begin(115200);
#endif



#ifdef USEWIFI
  // Next 2 line seem to be needed to connect to wifi after Wake up
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  delay(20);
#endif

  pinMode(GPIO_BOOT, INPUT_PULLDOWN_16);  // settings for the leds
  pinMode(GPIO_WS2813, OUTPUT);

  pixels.begin(); //initialize the WS2813
  pixels.clear();
  pixels.show();

  Wire.begin(9, 10); // Initalize i2c bus
  Wire.beginTransmission(I2C_PCA);
  Wire.write(0b00000000); //...clear the I2C extender to switch off vibrator and backlight
  Wire.endTransmission();

  delay(100);

  tft.begin(); //initialize the tft. This also sets up SPI to 80MHz Mode 0
  tft.setRotation(2); //turn screen
  tft.scroll(32); //move down by 32 pixels (needed)
  tft.fillScreen(BLACK);  //make screen black

  tft.setTextSize(2);
  tft.setCursor(25, 48);
  tft.print("GPN17");
  tft.setTextSize(1);

  tft.writeFramebuffer();
  setGPIO(LCD_LED, HIGH);

  pixels.clear(); //clear the WS2813 another time, in case they catched up some noise
  pixels.show();
}
