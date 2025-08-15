Tamam Mustafa, verdiğin metni yeni eklenen özelliklere göre **güncelledim** ve önceki formatına sadık kaldım:

---

**STM32F103 Universal Multi-Feature Firmware**
A feature-rich firmware for the STM32F103 MCU.
Supports UART command interface for:

* **LED control** (ON/OFF/STATUS)
* **PWM signal generation** (set frequency & duty)
  and **PWM measurement** via input capture
* **Servo motor angle setting** (`SERVO;SET;<-90..90>`)
  and **automatic Y-axis tracking** with ADXL345
* **ADC reading** (voltage on specific pins, all channels, resistance, capacitance)
* **I2C scanning** and **ADXL345 accelerometer reading** (gX, gY, gZ, aX, aY, aZ)
* **Flash memory read/write operations** on last page
* **HC-SR04 ultrasonic distance measurement** (single, continuous, OLED display)
* **Radar scan mode** (servo + ultrasonic + live OLED sweep display)
* **64x64 bitmap image display** on SSD1306 OLED
* **WAIT** command for millisecond delays
* **MCU restart** command
* **HELP** command to list all available commands

**Use cases:** Fast prototyping, education, custom boards, lab projects,
embedded development, and hardware feature demonstrations.

**How to use:**
Send commands via UART to interact with hardware peripherals in real-time.
Example:

```
PWM;SET;FREQ;1000;DUTY;30
MEASURE;DISTANCE;ON
RADAR;ON
```
