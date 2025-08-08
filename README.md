# STM32F103 Universal Multi-Feature Firmware

A feature-rich firmware for the STM32F103 MCU. Supports UART command interface for:

- LED control
- PWM signal generation and measurement
- Servo motor angle setting
- ADC reading (voltage, resistance, capacitance)
- I2C scanning and ADXL345 accelerometer reading
- Flash memory read/write operations

**Use case:** Fast prototyping, education, custom boards, lab projects, embedded development.

**How to use:**  
Send commands via UART to interact with hardware peripherals in real-time.  
Example: `PWM;SET;FREQ;1000;DUTY;30`
