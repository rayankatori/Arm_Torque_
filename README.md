# Arm_Torque Project 💪⚙️

A microcontroller-based project designed to **measure and/or control torque** on a robotic arm or mechanical joint using sensors and actuators. Ideal for robotics, automation, and feedback-controlled motion systems.

---

## 🎯 Project Goals

- Measure torque applied to a joint or arm using a torque sensor
- Control a motor or servo to apply or counter torque
- Implement feedback control (e.g., PID) to stabilize torque or position
- Visualize torque and system response over time

---

## 🧰 Hardware Requirements

- Arduino Uno / Nano / ESP32 / Raspberry Pi
- Torque sensor (e.g., strain gauge with amplifier like HX711)
- Servo motor or DC motor (with H-Bridge like L298N)
- Jumper wires and breadboard
- External power supply (depending on motor specs)
- (Optional) OLED / LCD screen for real-time display

---

## 💽 Software Requirements

- Arduino IDE or Python (if using Raspberry Pi)
- Libraries:
  - `Servo.h` for controlling servo motors
  - `PID_v1.h` for PID control (Arduino)
  - `HX711.h` for torque sensor (if using strain gauge)
  - `matplotlib`, `numpy`, `RPi.GPIO` (for Raspberry Pi implementation)

---

## 🔌 Wiring Overview

| Component        | Arduino Pin Example |
|------------------|---------------------|
| Torque Sensor    | A0 (Analog Input)   |
| Servo Motor      | D9 (PWM Output)     |
| HX711 (if used)  | D2, D3 (for Data, Clock) |
| Power/GND        | 5V / GND            |
| LCD/OLED Display | I2C (A4/A5) or SPI  |

---

## 💻 Example Arduino Code

```cpp
#include <Servo.h>
#include <PID_v1.h>

Servo motor;
double setpoint = 50.0;  // Desired torque level
double input, output;
PID myPID(&input, &output, &setpoint, 2, 5, 1, DIRECT);

void setup() {
  Serial.begin(9600);
  motor.attach(9);  // PWM pin
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  int sensorValue = analogRead(A0);  // Read torque sensor
  input = map(sensorValue, 0, 1023, 0, 100);  // Convert to torque units
  myPID.Compute();
  motor.write(constrain(output, 0, 180));  // Control servo position
  Serial.print("Torque: "); Serial.println(input);
  delay(200);
}
🚀 How to Use
Wire the components according to the table above

Upload the Arduino code via the Arduino IDE

Monitor real-time torque values using the Serial Monitor or external display

Adjust the PID parameters for better control response

📁 Project Structure
bash
Copy
Edit
Arm_Torque/
├── Arm_Torque.ino        # Main Arduino sketch
├── README.md             # Project documentation
├── docs/
│   └── wiring_diagram.png (optional)
└── lib/                  # Custom or external libraries
🔄 Future Enhancements
Real-time plotting via Python or Processing

Wireless control via Bluetooth or Wi-Fi

Full 2-axis or 3-axis torque feedback for robotic joints

GUI interface for adjusting setpoint and PID tuning

# Arm_Torque_
