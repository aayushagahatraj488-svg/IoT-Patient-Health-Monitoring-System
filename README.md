![Arduino](https://img.shields.io/badge/Arduino-IDE-00979D?logo=arduino&logoColor=white)
![ESP8266](https://img.shields.io/badge/ESP8266-NodeMCU-blue)
![MAX30100](https://img.shields.io/badge/Sensor-MAX30100-red)
![OLED](https://img.shields.io/badge/Display-SSD1306-green)
![License](https://img.shields.io/badge/License-MIT-success)
# ❤️ IoT-Based Patient Health Monitoring System

An IoT-based patient health monitoring system developed using **NodeMCU (ESP8266)**, **MAX30100 SpO₂/Heart Rate Sensor**, **SSD1306 OLED Display**, and a **Piezo Buzzer**.

The system measures **Heart Rate (BPM)** and **Blood Oxygen Saturation (SpO₂)** in real time. The readings are displayed on an OLED screen and streamed to a browser dashboard through USB Serial communication.
## ✨ Features

- ❤️ Real-time Heart Rate (BPM) Monitoring
- 🩸 Real-time SpO₂ Monitoring
- 📺 SSD1306 OLED Display
- 🔔 Piezo Buzzer Alert
- 💻 Live Browser Dashboard via USB Serial
- 📈 Live Sensor Data Visualization
- ⚡ NodeMCU (ESP8266) Based
- 🔌 I²C Communication (MAX30100 & OLED)
- ## 🛠 Components Used

| Component | Quantity |
|-----------|---------:|
| NodeMCU ESP8266 | 1 |
| MAX30100 SpO₂ & Heart Rate Sensor | 1 |
| SSD1306 OLED Display | 1 |
| Piezo Buzzer | 1 |
| Breadboard | 1 |
| Jumper Wires | As Required |
| USB Cable | 1 |
## 🔌 Pin Connections

| Component | NodeMCU Pin |
|-----------|-------------|
| MAX30100 SDA | D2 (GPIO4) |
| MAX30100 SCL | D1 (GPIO5) |
| OLED SDA | D2 (GPIO4) |
| OLED SCL | D1 (GPIO5) |
| Piezo Buzzer | D5 (GPIO14) |
| VCC | 3.3V |
| GND | GND |
## ⚙️ How It Works

1. Connect the NodeMCU to your computer using a USB cable.
2. Place your finger on the MAX30100 sensor.
3. The MAX30100 measures:
   - ❤️ Heart Rate (BPM)
   - 🩸 Blood Oxygen Level (SpO₂)
4. The sensor sends the data to the NodeMCU through the **I²C protocol**.
5. The NodeMCU processes the received data.
6. The readings are displayed on the SSD1306 OLED display.
7. The NodeMCU sends the processed data through **USB Serial**.
8. The browser dashboard reads the serial data and displays live readings.
9. The piezo buzzer provides an alert whenever required.
10. ## 📁 Project Structure

```
IoT-Patient-Health-Monitoring-System
│
├── AN_Health_Pulse_Node.ino
├── README.md
└── LICENSE
```
## 🚀 Installation

### Prerequisites

- Arduino IDE
- ESP8266 Board Package
- MAX30100 Pulse Oximeter Library
- U8g2 Graphics Library

### Steps

1. Clone this repository:

```bash
git clone https://github.com/aayushagahatraj488-svg/IoT-Patient-Health-Monitoring-System.git
```

2. Open `AN_Health_Pulse_Node.ino` in Arduino IDE.

3. Select the board:

```
NodeMCU 1.0 (ESP-12E Module)
```

4. Select the correct COM port.

5. Click **Upload**.

6. Connect the MAX30100, OLED Display, and Piezo Buzzer according to the pin connections.

7. Open the Serial Monitor (115200 baud) or use the compatible browser dashboard to view the live data.
