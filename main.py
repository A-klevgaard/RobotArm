#Imports
import serial, math

#Globals

ser = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
    )



#Functions block

#checks to see if a string entered is a float
def isFloat(string):
    try:
        float(string)
        return True
    except:
        return False


def DegreesToServoAngle(deg):
    return 2.5 * deg + 150
    
def CalculateTriangle(dist):
    arm1 = 130 # Servo2 to grip in mm
    arm2 = 100 # Servo 1 - 2 in mm
    
    #math based on https://www.calculator.net/triangle-calculator.html?vc=&vx=130&vy=100&va=&vz=100&vb=&angleunits=d&x=69&y=18
    angle1 = math.acos((math.pow(arm1,2) + math.pow(dist,2) - math.pow(arm2,2)) / (2 * arm1 * dist) ) 
    angle2 = math.acos((math.pow(arm1,2) + math.pow(arm2,2) - math.pow(dist,2)) / (2 * arm1 * arm2) ) 
    
    angleTuple = ( DegreesToServoAngle( angle1 ) , DegreesToServoAngle( angle2 ))
    
    return angleTuple

    

def LookForDie():
    #keep track of the last 2 data points and the current one, if they are
    #different by at least 0.3 then stop
    while (isFloat(ser.readline()) == False):
        pass
        
    oldest = float(ser.readline())
    newest = float(ser.readline())

    while (abs(oldest - newest) < 0.1):
        print('searching')
        oldest = float(ser.readline())
        newest = float(ser.readline())
        
    print('Found it! : Oldest :' + str(oldest) + ' Newest : ' + str(newest))

    #stop the servo
    

    #take the average value of newest
    count = 0
    readSum = 0.0
    while (count < 100):
        readSum += float(ser.readline())
        count +=1

    avg = readSum / count
    print(avg)

    num = math.log(avg / 3.839) / -0.101
    print(num)
    
    



#The Main Program
LookForDie()


