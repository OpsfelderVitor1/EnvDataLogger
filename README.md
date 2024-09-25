# LoRa-Based Temperature Data Logger Project

## Overview

This project is part of the **EA076 - Embedded Systems Course** and was developed by **Lucas A. P.** and **Vitor O. E.**. The goal of the project is to create a data logger system that collects temperature data from a sensor, displays it on a 7-segment display, and transmits the collected data wirelessly using LoRa communication.

### Key Features:
- **Temperature Sensor**: Continuously collects temperature data using an analog sensor.
- **LoRa Communication**: Sends the collected temperature data to a remote station via LoRa.
- **LCD Display**: Displays system status and prompts.
- **Memory Storage**: Logs temperature data in EEPROM for future retrieval.
- **Keyboard Input**: User can interact with the system to trigger data collection, reset, or transfer data.

## Project Structure

The project is structured as follows:

├── src
│   ├── main.ino               # Main Arduino sketch
│   ├── LoRaCommunication.h     # Header file for LoRa functions
│   ├── LoRaCommunication.cpp   # Implementation of LoRa functions
├── README.md                   # This documentation file
├── LICENSE                     # Project license



## Hardware Requirements

- Arduino Uno or compatible board
- LoRa Module (e.g., SX1278)
- Analog Temperature Sensor (e.g., LM35)
- 7-segment Display (via I2C)
- 16x2 LCD Display (optional for status display)
- EEPROM for data storage (24C16 or similar)
- Keyboard matrix (for user interaction)
- Jumper wires and breadboard

## Software Requirements

- **Arduino IDE**: Version 1.8.10 or higher.
- **LoRa Library**: You can install this library in the Arduino IDE by searching for `LoRa` in the library manager.
- **Wire Library**: For I2C communication (included with Arduino IDE).

## Setup Instructions

1. **Wiring**: 
   - Connect the LoRa module to the Arduino board using the SPI pins.
   - Connect the temperature sensor to an analog pin (e.g., A1).
   - Connect the 7-segment display using I2C.
   - Connect the 16x2 LCD display if you want to show the system status.

2. **Installing Dependencies**:
   - In the Arduino IDE, go to `Sketch > Include Library > Manage Libraries...`
   - Search for "LoRa" and install it.
   - Make sure `Wire.h` is included for I2C communication (already included by default in the Arduino IDE).

3. **Code Upload**:
   - Open `main.ino` in the Arduino IDE.
   - Select the correct port and board from the `Tools` menu.
   - Click `Upload` to flash the code to your Arduino.

## Usage

### System Features

- **Temperature Collection**:
  The system continuously collects temperature data from the sensor. Once a set number of samples have been collected, the data is transmitted wirelessly via LoRa.

- **LoRa Transmission**:
  The temperature data is sent to a remote receiver via LoRa. The receiver can be another Arduino or a compatible device equipped with a LoRa module. Each temperature reading is sent as a string in Celsius.

- **LCD and 7-Segment Display**:
  The LCD shows the system prompts, and the 7-segment display shows the temperature in real-time.

- **EEPROM Data Logging**:
  The collected temperature data is also stored in the EEPROM memory of the microcontroller, allowing for future retrieval and analysis.

### User Interaction

- **Keyboard Commands**:
  - Press `1` to reset the memory.
  - Press `2` to check the system status.
  - Press `3` to start collecting temperature data.
  - Press `4` to stop data collection.
  - Press `5` to transfer logged data via serial.

### Receiving Data via LoRa

If you have a second Arduino set up as a LoRa receiver, it will receive the transmitted temperature data and print it to the serial monitor.

```cpp
void receiveLoRaData() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }
    Serial.println("Received Data: " + receivedData);
  }
}
```

### Serial Data Transfer
The system can transfer the collected data via Serial using the keyboard command. It will send a user-defined number of data points stored in the EEPROM memory.

### Future Improvements
 - Add more robust error handling for LoRa communication.
 - Integrate with a cloud platform for remote monitoring.
 - Add security features for LoRa data transmission (e.g., encryption).

### License
This project is licensed under the MIT License - see the LICENSE file for details.

### Authors
Lucas Antonio Pelike
Vitor Opsfelder Estanislau

### Acknowledgments


Thanks to the EA076 course staff for guidance and support in completing this project.



### Customization
- Modify the **Hardware Requirements** section based on your specific setup.
- If you are using specific LoRa settings (e.g., frequency, spread factor), document them in the **Setup Instructions**.
- The **Future Improvements** section can be adjusted based on your project goals. 

