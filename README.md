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
I developed a library to receive temperature and humidity values from the sensor. You can find it [here](PCMonitoringSystem/Core/Src/am2320.c).
The commands for reading the registers and the register addresses for temperature and humidity were all obtained from the [sensor's datasheet](https://cdn-shop.adafruit.com/product-files/3721/AM2320.pdf).

## CPU & RAM Usage
I wrote a Python script to continuously send data serially to the Nucleo board. The serial port configured in STM32MX was COM3, and both the receiver and transmitter were set to a baud rate of 9600.

To ensure consistent data buffer size (since the percentage values can be one or two digits), I added a feature to insert a space if the CPU or RAM usage dropped below 10%. This keeps the data size uniform. The data is sent every 2 seconds. You can find the script [here](send_pc_infor.py).

## Passive Buzzer
To activate the passive buzzer, we need to generate PWM signals using the configured timer. I referred to this [this website](https://controllerstech.com/interface-passive-buzzer-with-stm32/) for guidance on using the buzzer.
![image](https://github.com/user-attachments/assets/c73cd438-19e2-49d8-86ef-7e621a95c893)

The clock source is set to the internal clock, and channel 1 generates the PWM signal. The prescaler is set to 0, so the frequency we are using is 60 MHz. The counter period is 1000, meaning that at every 60 MHz, the counter increments up, and when it reaches 1000, it auto-reloads.

To set the desired frequency, I simply changed the prescaler of the timer while keeping all other values the same. The function presForFrequency takes the frequency as input and outputs the necessary prescaler to set that frequency.

## Putting It All Together
Since this project addresses many subproblems, I used FreeRTOS CMSIS_V1. For each thread, I set the stack size to 256 and assigned normal priority. For the System Information task, I use a global flag called UARTFlag, which is raised when UART finishes its transmission. This allows the task to process the received data and output it on the display.
