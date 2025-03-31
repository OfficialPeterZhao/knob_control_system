/********************************************************************
   Matsusada.ino - PSU Monitoring & RS-485 Communication
   --------------------------------------------------

   This Arduino sketch monitors and displays voltage, current, 
   and on/off status of a Matsusada high-voltage power supply (PSU).
   It also sends the data over RS-485 to a PC for logging or remote
   monitoring.

   FEATURES:
   1) **ADS1115** for analog readings:
      - On/Off Status (CH_ON_OFF) *
      - Voltage Monitor (CH_VOLTAGE)
      - Current Monitor (CH_CURRENT)
   2) **RS-485 Communication** using SoftwareSerial on pins 8/9.
   3) **I2C LCD** (16×2) to display voltage, current, and status.
   4) **Backlight Control**:
      - PSU **ON** → Full brightness
      - PSU **OFF** → Dimmed backlight

   (*) If PSU status is a digital logic signal, consider using a
   digital pin instead of an ADC channel.

   NOTES & REQUIREMENTS:
   - Libraries:
       * Adafruit ADS1X15 (for ADS1115)
       * LiquidCrystal_PCF8574 (for I2C LCD)
   - Hardware Connections:
       * ADS1115 on I2C (0x48)
       * LCD on I2C (0x27)
       * RS-485 transceiver (MAX485):
         - DE/RE → Pin 2
         - RX → Pin 8
         - TX → Pin 9
       * LCD backlight dimming (if supported) → Pin 3 (PWM)

   WRITTEN BY: FAB LAB TEAM (Hardware & Software)
   DATE: March 2025
********************************************************************/

/* --------------- LIBRARY INCLUDES --------------- */
#include <Wire.h>                     // I2C communication
#include <Adafruit_ADS1X15.h>         // ADS1115 ADC library
#include <LiquidCrystal_PCF8574.h>    // I2C LCD (PCF8574-based)
#include <SoftwareSerial.h>           // RS-485 communication

/* --------------- CONSTANT DEFINITIONS --------------- */

// RS-485 transceiver control pin (DE/RE)
const int RS485_DE_RE_PIN = 2;

// LCD backlight control (PWM pin)
const int LCD_BACKLIGHT_PIN = 3;

// SoftwareSerial for RS-485 (pin 8 = RX, pin 9 = TX)
SoftwareSerial rs485Serial(8, 9);

// ADS1115 instance (I2C address 0x48)
Adafruit_ADS1115 ads(0x48);

// I2C LCD instance (typically 0x27 or 0x3F)
#define LCD_ADDR 0x27
LiquidCrystal_PCF8574 lcd(LCD_ADDR);

/* --------------- ADC CHANNEL DEFINITIONS --------------- */
enum {
  CH_ON_OFF   = 0,  // Reads PSU On/Off Status
  CH_VOLTAGE  = 1,  // Reads High-Voltage Output
  CH_CURRENT  = 2   // Reads Current Output
};

/* --------------- GLOBAL VARIABLES --------------- */
bool lastIsOn = false;        // Stores last PSU state to prevent redundant LCD updates
float lastHvVoltage = -1.0;   // Stores last voltage reading to prevent flickering
float lastHvCurrent = -1.0;   // Stores last current reading to prevent flickering

/* --------------- SETUP FUNCTION --------------- */
void setup() {
  // 1) Initialize Serial communication for debugging
  Serial.begin(9600);

  // 2) Initialize RS-485 communication
  rs485Serial.begin(9600);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Set to receive mode initially

  // 3) Initialize I2C communication
  Wire.begin();

  // 4) Initialize ADS1115 (ADC)
  if (!ads.begin()) {
    Serial.println("Error: ADS1115 not found!");
    while (1);  // Halt execution if ADC fails to initialize
  }
  ads.setGain(GAIN_ONE);  // ±4.096V input range

  // 5) Initialize LCD Display
  lcd.begin(16, 2);
  lcd.setBacklight(255);  // Full brightness
  lcd.clear();
  lcd.print("Matsusada PSU");
  delay(1000);
  lcd.clear();

  // 6) Configure LCD backlight control (if supported)
  pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
  analogWrite(LCD_BACKLIGHT_PIN, 255); // Full brightness
}

/* --------------- MAIN LOOP --------------- */
void loop() {
  // 1) READ analog inputs from ADS1115
  float rawOnOff   = readADC(CH_ON_OFF);
  float rawVoltage = readADC(CH_VOLTAGE);
  float rawCurrent = readADC(CH_CURRENT);

  // 2) CONVERT raw ADC readings to real-world values
  bool isOn = (rawOnOff > 2048);  // Threshold: 50% of ADC range
  float hvVoltage = (rawVoltage * 4.096) / 32768.0;
  float hvCurrent = (rawCurrent * 4.096) / 32768.0;

  // 3) DEBUG OUTPUT (Serial Monitor)
  Serial.print("PSU: ");
  Serial.print(isOn ? "ON" : "OFF");
  Serial.print(" | HV(V): ");
  Serial.print(hvVoltage, 2);
  Serial.print(" | I(A): ");
  Serial.println(hvCurrent, 3);

  // 4) SEND DATA OVER RS-485
  sendRS485Data(hvVoltage, hvCurrent, isOn);

  // 5) UPDATE LCD DISPLAY (only if values changed)
  if (hvVoltage != lastHvVoltage || hvCurrent != lastHvCurrent || isOn != lastIsOn) {
    lcd.setCursor(0, 0);
    lcd.print("HV: ");
    lcd.print(hvVoltage, 2);
    lcd.print(" V   ");

    lcd.setCursor(0, 1);
    lcd.print("PSU: ");
    lcd.print(isOn ? "ON " : "OFF");
    lcd.print("   ");

    lastHvVoltage = hvVoltage;
    lastHvCurrent = hvCurrent;
    lastIsOn = isOn;
  }

  // 6) CONTROL LCD BACKLIGHT (only update if PSU state changes)
  static bool lastBacklightState = true;
  if (isOn != lastBacklightState) {
    analogWrite(LCD_BACKLIGHT_PIN, isOn ? 255 : 80);
    lastBacklightState = isOn;
  }

  delay(500); // Slow update rate for stability
}

/* --------------- FUNCTION: readADC() --------------- */
float readADC(uint8_t channel) {
  int16_t result = ads.readADC_SingleEnded(channel);
  return (result < 0) ? 0 : (float)result;
}

/* --------------- FUNCTION: sendRS485Data() --------------- */
void sendRS485Data(float voltage, float current, bool isOn) {
  digitalWrite(RS485_DE_RE_PIN, HIGH); // Enable TX mode

  rs485Serial.print("MATSUSADA,ON,");
  rs485Serial.print(isOn ? 1 : 0);
  rs485Serial.print(",VOLT,");
  rs485Serial.print(voltage, 2);
  rs485Serial.print(",CURR,");
  rs485Serial.print(current, 3);
  rs485Serial.println();

  delay(5);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Switch back to RX mode
}
