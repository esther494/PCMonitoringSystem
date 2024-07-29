# Real-Time PC Monitoring System

This project involves creating a real-time PC monitoring system that alerts the user if the PC overheats or if the CPU and RAM usage are too high. In case the temperature exceeds the predefined value, a small fan activates to cool the system down. The system is built using an STM32F401RE microcontroller, which is wired with a temperature sensor and a buzzer. A Python script runs on the PC to periodically send CPU and RAM usage data serially to the STM32 hardware. The gathered information, including temperature and usage data, is displayed on an LCD. The buzzer provides an alert if any monitored parameter exceeds the predefined threshold.

## Materials

1. 2004 I2C LCD
2. Adafruit AM2320 Sensor
3. PS1240P02BT
4. SER0006

## Connections
![image](https://github.com/user-attachments/assets/858797c7-e27e-442a-8b70-227abc66500c)

- PB8 & PB9 are for the I2C of the LCD
- PA2 & PA3 are for the UART transmission from the Python script
- PA8 & PC9 are for the I2C of the temperature & humidity sensor
- PB4 is for the TIM3_CH1 of the passive buzzer

## Temperature & Humidity Sensor
I created a library to receive temperature and humidity values from the sensor. You can find this [here](PCMonitoringSystem/Core/Src/am2320.c).
The command to send to read the regsiters and the register addresses for temperature and humidity were all found in the [datasheet of the sensor](https://cdn-shop.adafruit.com/product-files/3721/AM2320.pdf).

## CPU & RAM Usage
I wrote a script in Python to continuously send data serially over the port to the Nucleo board. The Serial Port of the UART configured in STM32MX was COM3 and the baud rate of both the receiver and the transmitter was 9600. 

Since the size of the data buffer was not consistent (percentage can be one or two digits), I added a feature to add a space if the CPU or RAM usage decreased under 10% to keep the data size the same. The data was sent every 2 seconds. This script can be found [here](send_pc_infor.py).

## Passive Buzzer
To activate the passive buzzer, we must generate PWM signals using the configured timer. I have referred to [this website](https://controllerstech.com/interface-passive-buzzer-with-stm32/) to use the buzzer
![image](https://github.com/user-attachments/assets/c73cd438-19e2-49d8-86ef-7e621a95c893)

Clock source is set from the internal clock and the channel1 generates the PWM signal.
Prescaler is set as 0, so the frequency we are using is 60MHz. Counter Period is 1000 which means that at every 60MHz, the counter increments up and when it reaches 1000, it auto reloads. 

To set the desired frequency, I just changed the prescaler of the timer and kept all the other values the same. The function presForFrequency takes frequency as the input and outputs the prescaler needed to set the frequency. 
