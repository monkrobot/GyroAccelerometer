import serial
import time
import re
import numpy as np
import math

ser = serial.Serial('COM3', 9600)

while True:
    start = time.time()
    #print('thie is start:' + str(start))
    data_sensor = str(ser.readline())

    print(data_sensor)
    data_sensor_modify = re.findall("[-0-9.]+", data_sensor)
    #print('data_sensor_modify' + str(data_sensor_modify))
    if data_sensor_modify == []:
        continue
    else:
        data_gyro = data_sensor_modify[3:6]
        data_gyro_massive = np.array([float(data_gyro[0]), float(data_gyro[1]), float(data_gyro[2])])
        #print("Data_gyro_massive" + str(data_gyro_massive))
        data_accel = data_sensor_modify[0:3]
        data_accel_massive = np.array([float(data_accel[0]), float(data_accel[1]), float(data_accel[2])])
