import cv2
import os
import time
import RPi.GPIO as GPIO
import os.path
import RPi.GPIO as gp  
from time import sleep
import time
import serial


f1 = open("/home/pi/vehiclenumberdatabase/1.txt", "r")
authvech1=f1.read();
f1.close()
f2 = open("/home/pi/vehiclenumberdatabase/2.txt", "r")
authvech2=f2.read();
f2.close()
f3= open("/home/pi/vehiclenumberdatabase/3.txt", "r")
authvech3=f3.read();
f3.close()


print("++++++++++++ AURTHORIZED VEHICLE NUMBERS ++++++++++++++++")
print(authvech1)
print(authvech2)
print(authvech3)
print("+++++++++++++++++++++++++++++++")   


ser = serial.Serial(
                    port='/dev/ttyAMA0',
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

servoPIN=12
manualbutton=13
buzzerpin=40
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(manualbutton, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(servoPIN,GPIO.OUT)
GPIO.setup(buzzerpin,GPIO.OUT)

GPIO.output(buzzerpin , 0)


def buzzering():
    GPIO.output(buzzerpin , 1)
    time.sleep(2)
    GPIO.output(buzzerpin , 0)

def SetAngle(angle):
    pwm=GPIO.PWM(servoPIN,50)  
    pwm.start(0)
    duty = angle / 18 + 2
    GPIO.output(servoPIN , True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servoPIN, False)
    pwm.ChangeDutyCycle(0)
    pwm.stop()


def closgate():
    SetAngle(180)
    
    
def operategate():
    SetAngle(90)
    time.sleep(5)
    SetAngle(180)
    
    
    
mainruncmmd ="sudo python3 /home/pi/automatictollgate/plate_recognition.py --api-key ca80ed924a7e3a66431b56db6b7bcdd57ba8dd25 "

timestr=time.strftime("%Y%m%d-%H%M%S")
dir_path = '/home/pi/Images/'
file_name= timestr +'.jpg'

path = dir_path +file_name

withfilepath = mainruncmmd + path 

cap = cv2.VideoCapture(0) 
cap.set(480, 640) 

cap.read()
closgate()

while(True): 
    ret, frame = cap.read()
   
    if (ser.inWaiting()>0):
        data_str = ser.read(12).decode('ascii')
        print(data_str, end='')
        if data_str==u1rfid:
            vrfid1=1
        elif data_str==u2rfid:
            vrfid2=1
        elif data_str==u3rfid:
            vrfid3=1
            
    if not GPIO.input(11):
        print('Input was LOW')
        cv2.imwrite(path, frame)
        os.system(withfilepath)
        f = open("/home/pi/automatictollgate/numberdata.txt", "r")
        resultnumber=f.read()
        print("Recognized LP Number:")
        print(resultnumber)
        
        if resultnumber==authvech1 and vrfid1==1:
            print("---VALID NUMBER 1  PLATE AVAILABLE IN DB -----")
            operategate()
        elif resultnumber==authvech2 and vrfid2==1:
            print("---VALID NUMBER 2 PLATE AVAILABLE IN DB -----")
            operategate()
        elif resultnumber==authvech3 and vrfid3==1:
            print("---VALID NUMBER 3 PLATE AVAILABLE IN DB -----")
            operategate()
            
        else:
            print("******* NUMBER PLATE NOT AVAILABLE IN DB PAY MANUALLY ******")
            buzzering()
            
            
    if not GPIO.input(manualbutton):
        operategate()
        
        
    
            
        
            
        
        
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
