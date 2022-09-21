import time
import serial
          

ser = serial.Serial(
                    port='/dev/ttyUSB0',
                    baudrate = 9600,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                    )
                    #timeout=1 # must use when using data.readline()
                    #)
print (" ")


u1rfid="4B005B344A6E"
u2rfid="4B005AA302B0"
u3rfid="4B005AAFFA44"

vrfid1=0
vrfid2=0
vrfid3=0

while(True):
    
      if (ser.inWaiting()>0):
             #data_str = ser.read(ser.inWaiting()).decode('ascii') 
        data_str = ser.read(12).decode('ascii')
        print(data_str, end='')
        if data_str==u1rfid:
            vrfid1=1
        elif data_str==u2rfid:
            vrfid2=1
        elif data_str==u3rfid:
            vrfid3=1
        data_str=""
      time.sleep(0.01)
    
