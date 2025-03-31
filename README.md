# Arduino-based Knob Box for PSU Monitoring and Control

This project involves creating a **Knob Box** using an **Arduino Uno** to interface with a **Power Supply Unit (PSU)**. The box allows users to interact with the PSU through knobs (potentiometers) for controlling voltage and current, monitor PSU status, and display readings on an I2C screen. It also communicates with a PC using **RS-485** to send real-time data from the PSU.

The project supports monitoring and controlling two types of PSUs:
- **Bertan PSU**
  - Polarity indicator
  - Voltage and current monitor
- **Matsusada PSU**
  - On/off status
  - Voltage and current monitor

---

## Features

1. **Knob Controls (Potentiometers)**:
   - **Adjust PSU parameters** like voltage and current using rotary potentiometers (knobs).
   - Knobs control digital potentiometers or communicate with the PSU to change settings.

2. **ADC Pin Readings**:
   - **Monitor PSU voltage and current** using the Arduino's ADC pins.
   - **External ADC** (if necessary) can be used for better resolution and accuracy.

3. **PSU Monitoring**:
   - **Bertan PSU**:
     - **Polarity Indicator**: Detects the output polarity (positive/negative).
     - **Voltage Monitor**: Measures and displays PSU output voltage.
     - **Current Monitor**: Measures and displays PSU output current.
   - **Matsusada PSU**:
     - **On/Off Status**: Detects PSU on/off status.
     - **Voltage Monitor**: Measures and displays PSU output voltage.
     - **Current Monitor**: Measures and displays PSU output current.

4. **RS-485 Communication**:
   - **Transmit real-time data** over **RS-485** to a PC for further analysis or logging.
   - Regular updates with **voltage**, **current**, and **status** readings.

5. **I2C Display**:
   - **Display PSU parameters** such as voltage, current, and status on an I2C screen (LCD/OLED).
   - **Control the display’s backlight**: Dim or turn off the backlight when the PSU is off to conserve energy.

6. **Backlight Control**:
   - The I2C display’s backlight adjusts based on the **PSU status**:
     - When the PSU is **off**, the backlight is dimmed or turned off.
     - When the PSU is **on**, the backlight is bright.

---

## Hardware Components

- **Arduino Uno** – Main controller for reading knobs, sending data to the PC, and controlling the display.
- **Rotary Potentiometers** – User input for controlling PSU voltage and current.
- **External ADC** – For more precise readings of PSU parameters (if needed).
- **I2C Display** – For displaying real-time PSU data (e.g., voltage, current, status).
- **RS-485 Transceiver** – For communication with a PC over RS-485.
- **Power Supply** – Powers the Arduino and associated components.


---

## Libraries Required

- **Wire.h** – For I2C communication.
- **RS485.h** – For RS-485 communication with the PC.
- **Adafruit_Sensor.h** and **Adafruit_GFX.h** – For handling the display.
- **SPI.h** – For communication with the external ADC (if used).
- **Wire.h** – For I2C communication with the display.

---

