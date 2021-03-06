#Imports
import serial, math, time
# Import the PCA9685 module.
import Adafruit_PCA9685
import enum
#Globals

ser = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    #timeout = 1
    timeout = None
    )

class armStateMachine(enum.Enum):
    sweeping = 1
    reaching = 2
    grabbing = 3
    throwing = 4
    returning = 5
    
directionServo = 300
moveInc = 5
oldest = 0;

#Initializations

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()
armState = 1

#flush serial input
ser.flush()


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
    global directionServo, moveInc, pwm, armState, oldest #pull in the global variable,
    firstEdge = 0
    secondEdge = 0

    #keep track of the last 2 data points and the current one, if they are
    #different by at least 0.3 then stop
    while (isFloat(ser.readline()) == False):
        pass

    #clear input buffers (one of these is redundant and can be deleted later)
    ser.flush()
    ser.flushInput()

    time.sleep(0.1)
    newest = float(ser.readline())
    oldest = newest
    while (abs(oldest - newest) < 0.4):
        print('searching')
        #move the arm to the next test point
        if directionServo < 150 or directionServo > 600:
            moveInc *= -1;
        directionServo += moveInc
        print("servo angle: ", directionServo)
        pwm.set_pwm(0,0,directionServo)

        #oldest and newest       
        ser.flush()
        oldest = newest
        newest = float(ser.readline())
                  
        
    print('Found it! : Oldest :' + str(oldest) + ' Newest : ' + str(newest))
    armState = 2
    firstEdge = directionServo
    print("Arm state is: " + str(armState))
    print("first edge: ",firstEdge)

    oldest = newest
    ser.flush()
    ser.flushInput()
    if armState == 2:
        while (abs(oldest - newest) < 0.4):
            print('searching')
            #move the arm to the next test point
            if directionServo < 150 or directionServo > 600:
                moveInc *= -1;
            directionServo += moveInc
            print("servo angle: ", directionServo)
            pwm.set_pwm(0,0,directionServo)

            #oldest and newest
            ser.flush()
            oldest = newest
            newest = float(ser.readline())
            print("oldest value: ", oldest)
            print("newest value: ", newest)
            print("difference: ",abs(oldest - newest)) 

    #break out of the second loop, armstate is 3, both edges detected
    armState = 3
    secondEdge = directionServo
    center = (firstEdge + secondEdge) / 2
    pwm.set_pwm(0,0,center)
    print("armState is: ", armState)
    print("second edge ", secondEdge)
    
    
    #take the average value of newest
    count = 0
    readSum = 0.0
    while (count < 100):
        readSum += float(ser.readline())
        count +=1

    avg = readSum / count
    print("average mv: ",avg)



    num = math.log(avg / 3.839) / -0.101
    print(num)


#The Main Program
pwm.set_pwm(1,0,300)
pwm.set_pwm(2,0,300)
pwm.set_pwm(0,0,directionServo)
key = raw_input("Press space key to start")
print("First reading: ", ser.readline())
while key == " " :
    while armState == 1:
        LookForDie()
    


