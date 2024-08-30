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
Although it is a straightforward project, it involved several hardwares and subtasks and I have decided that implementing FreeRTOS would be highly beneficial for this project. Setting up the FreeRTOS was done in STM32CubeMX.

![image](https://github.com/user-attachments/assets/6b89c74d-3ddf-445f-9abd-fa6f7df60227)

**Task Separation:** For each hardware that interacts with the program (e.g., display, sensor, DMA, and LCD), FreeRTOS allowed me to develop each of these part separately, ensuring that I can debug without interference.

**Concurrent Execution:** The project required multiple tasks to run simultaneously (e.g., reading sensor data, receiving PC system info, and displaying on LCD) and not using FreeRTOS would have significantly delayed the tasks.

**Prioritization and Scheduling:** In my project, receiving the PC system data has the highest priority and since there are some delays involved here and there, it is very beneficial to allow the task to get the most CPU attention during those delays.

## Sensor
The sensor uses I2C to send its data. I created a [library](PCMonitoringSystem/Core/Src/am2320.c) that helps read temperature and humidity separately. The sensor has a sleep mode, requiring the programmer to wake it up before requesting data. The temperature and humidity each have different commands, as detailed in the datasheet. It is important to note that after each execution, the sensor needs a short delay before requesting new data. The data read is then used in a different task to be displayed on the LCD. To ensure the data doesn’t change during this process, the raw data is first saved in a buffer. The code below shows one of the functions:

```c
HAL_StatusTypeDef AM2320_ReadTemperature(I2C_HandleTypeDef *hi2c, float *temperature) {
  // cmd + start address + num of regs
  uint8_t msg[3] = {0x03, 0x02, 0x02};
  uint8_t temp_data[8];

  HAL_I2C_Master_Transmit(hi2c, SENSOR_ADDRESS << 1, 0x02, 0, HAL_MAX_DELAY);
  HAL_Delay(1);

  // Send command to read temperature
  if (HAL_I2C_Master_Transmit(hi2c, SENSOR_ADDRESS << 1, msg, 3, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }
  HAL_Delay(2);

  // Receive temperature data
  if (HAL_I2C_Master_Receive(hi2c, SENSOR_ADDRESS << 1, temp_data, 8, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }
  HAL_Delay(2);

  // Convert received data to temperature
  uint16_t raw_temp = (temp_data[2] << 8) | temp_data[3];
  *temperature = (float) raw_temp / 10;

  return HAL_OK;
}
```

## System Data
Using a Python program, I sent the CPU and RAM usage over UART DMA to the Nucleo board. The data was sent as a single string with a delimiter. After receiving the string, the delimiter was used to separate the data, which was then sent to the LCD for real-time display. This program was later converted into an executable file that can be run during execution.

## Comments
Future Considerations:
1. Error handling for I2C communication and UART DMA
2. Enhance the Python program by adding data logging or additional system metrics
