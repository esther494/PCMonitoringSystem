# Real-Time PC Performance and Health Dashboard
**RTOS, UART DMA, Python**

![](Preview.gif)

This is a simple embedded project that displays the temperature and humidity inside a PC case, as well as the CPU and RAM usage.
- Python program that automatically executes as a startup program will transmit PC data over UART.
- A I2C LCD that displays all collected data.
- A notification system that alerts the user when the temperature exceeds a predefined value.

## Hardware Integration
- I2C LCD [(2004 LCD)](https://www.digikey.ca/en/products/detail/sunfounder/CN0296D/18668625?utm_adgroup=&utm_term=&utm_content=&gad_source=1)
- Temperature and Humidity Sensor [(AM2320)](https://www.adafruit.com/product/3721)
- A USB cable connected to PC for UART transmission

## Project Setup
- Display: PB9 is set to I2C1_SDA and PB8 is set to I2C_SCL.
- Sensor: PC9 is set to I2C3_SDA and PA8 is set to I2C3_SCL.

![Pinout_PC_Performance](https://github.com/user-attachments/assets/455da082-8ecd-4fa5-80f7-44691f3c5fc6)

## RTOS
