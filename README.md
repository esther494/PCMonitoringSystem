# Real-Time PC Monitoring System

This project involves creating a real-time PC monitoring system that alerts the user if the PC overheats or if the CPU and RAM usage are too high. In case the temperature exceeds the predefined value, a small fan activates to cool the system down. The system is built using an STM32F401RE microcontroller, which is wired with a temperature sensor and a buzzer. A Python script runs on the PC to periodically send CPU and RAM usage data serially to the STM32 hardware. The gathered information, including temperature and usage data, is displayed on an LCD. The buzzer provides an alert if any monitored parameter exceeds the predefined threshold.

## Materials

1. 2004 I2C LCD
2. Adafruit AM2320 Sensor
3. PS1240P02BT
4. SER0006

## Connections
![image](https://github.com/user-attachments/assets/ec0669d7-56a6-4f5a-83db-8f819206d911)

- PB8 & PB9 are for the I2C of the LCD
- PA2 & PA3 are for the UART transmission from the Python script
- PA8 & PC9 are for the I2C of the temperature & humidity sensor

