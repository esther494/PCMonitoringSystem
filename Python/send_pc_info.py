import serial
import psutil
import time

# Configure the serial port
SERIAL_PORT = 'COM3'
BAUD_RATE = 9600

# Open serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def get_system_info():
    cpu_usage = psutil.cpu_percent(interval=1)
    ram_usage = psutil.virtual_memory().percent
    return cpu_usage, ram_usage

def send_data(cpu_usage, ram_usage):
    if cpu_usage >= 10 and ram_usage >= 10:
        data = f"CPU: {cpu_usage:.2f}%,RAM: {ram_usage:.2f}%\n"
    elif cpu_usage >= 10 and ram_usage < 10:
        data = f"CPU: {cpu_usage:0.2f}%,RAM:  {ram_usage:0.2f}%\n"
    elif cpu_usage < 10 and ram_usage >= 10:
        data = f"CPU:  {cpu_usage:0.2f}%,RAM: {ram_usage:0.2f}%\n"
    ser.write(data.encode('utf-8')) # Convert the data into bytes and send

try:
    while True:
        cpu_usage, ram_usage = get_system_info()
        send_data(cpu_usage, ram_usage)
        time.sleep(2)
except KeyboardInterrupt:
    ser.close()