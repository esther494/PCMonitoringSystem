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

## Temperature & Humidity Sensor
I created a library to receive temperature and humidity values from the sensor. You can find this [here](PCMonitoringSystem/Core/Src/am2320.c).
The command to send to read the regsiters and the register addresses for temperature and humidity were all found in the [datasheet of the sensor](https://cdn-shop.adafruit.com/product-files/3721/AM2320.pdf).

## CPU & RAM Usage
I wrote a script in Python to continuously send data serially over the port to the Nucleo board. The Serial Port of the UART configured in STM32MX was COM3 and the baud rate of both the receiver and the transmitter was 9600. 

Since the size of the data buffer was not consistent (percentage can be one or two digits), I added a feature to add a space if the CPU or RAM usage decreased under 10% to keep the data size the same. The data was sent every 2 seconds. This script can be found [here](send_pc_infor.py).
