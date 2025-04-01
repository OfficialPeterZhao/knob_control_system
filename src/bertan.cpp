/********************************************************************
   KnobBox_Bertan.ino - DRAFT
   --------------------------------------------------

   This sketch is intended for an Arduino controlling/monitoring
   a Bertan high-voltage power supply (PSU). It reads polarity,
   voltage, and current signals (via an ADS1115 ADC), displays them
   on an I2C LCD, and sends data via RS-485.

   FEATURES:
   1) **ADS1115** for analog inputs:
       - Polarity (CH_POLARITY) *
       - Voltage Monitor (CH_VOLTAGE)
       - Current Monitor (CH_CURRENT)
   2) **RS-485** communication using SoftwareSerial on pins 8/9.
   3) **I2C LCD** (16×2) via LiquidCrystal_PCF8574 library.

   (*) If your Bertan’s polarity line is an open-collector digital
   output, consider using a digital pin with pull-up instead of the ADC.

   NOTES & REQUIREMENTS:
   - Libraries:
       * Adafruit ADS1X15 (for ADS1115)
       * LiquidCrystal_PCF8574 (or another I2C LCD library)
   - Wiring (example):
       * ADS1115 on I2C (0x48)
       * LCD on I2C (0x27)
       * RS-485 transceiver (e.g. MAX485) with DE/RE on pin 2,
         RX on pin 8, TX on pin 9
   - Adjust pin assignments, addresses, thresholds, and scaling as needed.

   WRITTEN BY: FAB LAB TEAM (Hardware & Software)
   DATE: April 2025
********************************************************************/

/* --------------- LIBRARY INCLUDES --------------- */
#include <Wire.h>                     // I2C library
#include <Adafruit_ADS1X15.h>         // ADS1015/ADS1115 library
#include <LiquidCrystal_PCF8574.h>    // I2C LCD (PCF8574-based)
#include <SoftwareSerial.h>           // For RS-485 

/* --------------- CONSTANT DEFINITIONS --------------- */

// Pin controlling RS-485 transceiver (tied to DE/RE).
// HIGH = Transmit mode, LOW = Receive mode
const int RS485_DE_RE_PIN = 2;

// SoftwareSerial pins for RS-485
//  - Pin 8: RX input to Arduino from RS-485 transceiver
//  - Pin 9: TX output from Arduino to RS-485 transceiver
SoftwareSerial rs485Serial(8, 9);

// ADS1115 at I2C address 0x48
Adafruit_ADS1115 ads;  // Use default constructor

// I2C address of the LCD (often 0x27 or 0x3F).
#define LCD_ADDR 0x27
LiquidCrystal_PCF8574 lcd(LCD_ADDR);

/* --------------- ADC CHANNEL DEFINITIONS --------------- */
/*
   ADS1115 has 4 single-ended channels (0..3). Example usage:
     - CH_POLARITY = 0
     - CH_VOLTAGE  = 1
     - CH_CURRENT  = 2
     - CH_SPARE    = 3 
*/
enum {
  CH_POLARITY = 0,  // Polarity or spare channel
  CH_VOLTAGE  = 1,  // High-voltage monitor
  CH_CURRENT  = 2   // Current monitor
};


/* --------------- SETUP FUNCTION --------------- */
void setup() {


  // 1) Debugging over USB
  Serial.begin(9600);

  // 2) RS-485 config (via SoftwareSerial)
  rs485Serial.begin(9600);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Start in receive mode

  // 3) I2C initialization
  Wire.begin();

  // 4) ADS1115 initialization
  ads.begin(0x48);
  ads.setGain(GAIN_ONE); // ±4.096 V range

  // 5) LCD initialization
  lcd.begin(16, 2);      // 16 columns, 2 rows
  lcd.setBacklight(255); // Turn backlight on
  lcd.clear();
  lcd.print("Bertan KnobBox");
  delay(1000);
  lcd.clear();
}


/* --------------- MAIN LOOP --------------- */
void loop() {
  // 1) READ from ADS1115
  float rawPolarity = readADC(CH_POLARITY);
  float rawVoltage  = readADC(CH_VOLTAGE);
  float rawCurrent  = readADC(CH_CURRENT);

  // 2) CONVERT raw readings to engineering units
  //    Example threshold for polarity:
  bool polarityPositive = (rawPolarity > 2048);
  // Convert 0..32767 range to approximate 0..5 V.
  // A more precise approach = rawValue * (4.096 / 32767.0).
  float hvVoltage = (rawVoltage * 5.0f) / 32767.0f;
  float hvCurrent = (rawCurrent * 5.0f) / 32767.0f;

  // 3) DEBUG: print to USB serial
  Serial.print("Polarity: ");
  Serial.print(polarityPositive ? "POS" : "NEG");
  Serial.print(" | HV(V): ");
  Serial.print(hvVoltage, 2);
  Serial.print(" | I(A): ");
  Serial.println(hvCurrent, 3);

  // 4) SEND data via RS-485
  sendRS485Data(hvVoltage, hvCurrent, polarityPositive);

  // 5) UPDATE the LCD
  lcd.setCursor(0, 0);
  lcd.print("HV: ");
  lcd.print(hvVoltage, 2);
  lcd.print(" V   ");

  lcd.setCursor(0, 1);
  lcd.print("Pol: ");
  lcd.print(polarityPositive ? "POS" : "NEG");
  lcd.print("    ");

  delay(500); // Slow the loop slightly
}


/* --------------- readADC() --------------- */
/*
   Reads a single-ended channel from ADS1115:
   - Returns float in 0..32767 (GAIN_ONE).
   - Negative values are clamped to 0 (just in case).
*/
float readADC(uint8_t channel) {
  int16_t result = ads.readADC_SingleEnded(channel);
  if (result < 0) result = 0;
  return (float)result;
}


/* --------------- sendRS485Data() --------------- */
/*
   Sends a CSV-like string: "BERTAN,VOLT,<v>,CURR,<c>,POL,<p>\n"
   - voltage: HV reading in user-defined units (V or kV).
   - current: current reading (A or mA).
   - posPolarity: boolean (true = positive, false = negative).

   Steps:
   1) Enable TX mode
   2) Print message
   3) Small delay
   4) Disable TX mode
*/
void sendRS485Data(float voltage, float current, bool posPolarity) {
  digitalWrite(RS485_DE_RE_PIN, HIGH); // TX mode

  rs485Serial.print("BERTAN,VOLT,");
  rs485Serial.print(voltage, 2);
  rs485Serial.print(",CURR,");
  rs485Serial.print(current, 3);
  rs485Serial.print(",POL,");
  rs485Serial.print(posPolarity ? 1 : 0);
  rs485Serial.println();

  delay(5);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // RX mode
}
