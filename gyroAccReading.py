import serial
import re
ser = serial.Serial('COM3', 9600)
while True:
    data_sensor = str(ser.readline())
    print(data_sensor)
    data_sensor_modify = re.findall("[-0-9.]+", data_sensor)
    print(data_sensor_modify)
    print(data_sensor_modify[1])