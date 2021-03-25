#Imports
import serial, math, time
import Adafruit_PCA9685
import enum

#Globals

ser = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
    )

l1 = 95 #Length of arm1
l2 = 135 #length of arm2


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
    
def ServoAngleToDegrees(SA):
    return (SA - 150) / 2.5 
    
def RadiansToDegrees(rad):
    return rad * 180 / math.pi


def sq(num):
    return math.pow(num,2)



#IK for just the 2 links
def invkin(x, y):
    """Returns the angles of the first two links
    in the robotic arm as a list.
    returns -> (th1, th2)
    input:
    x - The x coordinate of the effector
    y - The y coordinate of the effector
    angleMode - tells the function to give the angle in
                degrees/radians. Default is degrees
    output:
    th1 - angle of the first link w.r.t ground
    th2 - angle of the second link w.r.t the first"""
    x += 50
    

    #stuff for calculating th2
    r_2 = x**2 + y**2
    l_sq = l1**2 + l2**2
    term2 = (r_2 - l_sq)/(2*l1*l2)
    term1 = ((1 - term2**2)**0.5)*-1
    #calculate th2
    th2 = math.atan2(term1, term2)
    #optional line. Comment this one out if you 
    #notice any problems
    #th2 = -1*th2

    #Stuff for calculating th2
    k1 = l1 + l2*math.cos(th2)
    k2 = l2*math.sin(th2)
    r  = (k1**2 + k2**2)**0.5
    gamma = math.atan2(k2,k1)
    #calculate th1
    th1 = math.atan2(y,x) - gamma


    return math.degrees(th1), 180 + math.degrees(th2)
    

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
    
    return num



#The Main Program
pwm.set_pwm(1,0,300)
pwm.set_pwm(2,0,300)

distance = LookForDie()
    
angles = invkin(distance, -50)

servoangle2 = 395 + 2.5 * (90 - angles[1])
servoangle2 = int(round(servoangle2))
servoangle1 = 560 - 2.5 * angles[0]
servoangle1 = int(round(servoangle1))

print("Between arm1 and arm2 = " + str(angles[1]) + " | Amount down from 90 degrees = " + str(90 - angles[1]) + " | Actual Small Servo Angle = " + str(395 + 2.5 * (90 - angles[1])))

print("Angle away from flat = " + str(angles[0]) + " | Actual Big Servo Angle = " + str(560 - 2.5 * angles[0]))

pwm.set_pwm(1,0,servoangle1)
pwm.set_pwm(2,0,servoangle2)

#straight up big = 340
#straight up small = 160

#servo 2 = 395 when 45 degrees

#angle at 85 degrees = 398
#angle at 72 degrees = 440


# Max = 560, Min = 315
# Servo 1, Angle = 90, Servo Angle = 315
# Servo 1, Angle = 45, Servo Angle = 445
# Servo 1, Angle = 0 , Servo Angle = 560
# offset = 

# Max = 500, Min = 90
# Servo 2, Angle = 77, Servo Angle = 90
# Servo 2, Angle = 45, Servo Angle = 150
# Servo 2, Angle = 0 , Servo Angle = 275
# Servo 2, Hits Block , Servo Angle = 500
# Range of 104 degrees
# offset = 



