#透過pytohn讀取arduino超音波資料
import serial
import time
ser=serial.Serial('COM30',57600)#設定串口，鮑率
while True:
    x=ser.readline()#讀取整排資料
    x=bytes.decode(x)#byte2str
    x=eval(x)#str2dictionary
    print(x)
    time.sleep(0.01)