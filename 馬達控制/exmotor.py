import serial

import  motor

SerPort = '/dev/ttyACM0'  #舉例

#方法一
motorSer = motor.begin(SerPort)

#方法二
#motorSer = serial.Serial(SerPort,115200,timeout=0.1)
#motor.getSer(motorSer)

#馬達動作
motor.go()  #停車
motor.go(0,0)  #停車

motor.go(50,50)  #前進 左PWM 50  右PWM 50  預設持續0.5秒
motor.go(-50,-50)  #後退  同上

motor.go(50,50,2)  #前進 左PWM 50  右PWM 50  預設持續2秒 (最久2.25秒)

#持續時間內有新指令會以新指令為主