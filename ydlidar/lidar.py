import init
import time
port='COM19'
Obj=init.YdLidarX4(port)
if(Obj.Connect()):
    gen = Obj.StartScanning()
    t = time.time() # start time 
  
    data=next(gen)
    print(data)
    
        
    Obj.StopScanning()
    Obj.Disconnect()
else:
    print("Error connecting to device")