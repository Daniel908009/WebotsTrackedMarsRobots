from controller import Robot, Keyboard
import math

robot = Robot()
keyboard = Keyboard()

timestep=int(robot.getBasicTimeStep())
keyboard.enable(timestep)

left_motor = robot.getDevice("left motor")
right_motor = robot.getDevice("right motor")
gps = robot.getDevice("gps")
compass = robot.getDevice("compass")
status_led = robot.getDevice("statusLed")
receiver = robot.getDevice("receiver")
emitter = robot.getDevice("emitter")
gps.enable(timestep)
compass.enable(timestep)
receiver.enable(timestep)

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

speed = 0.5
wait_time = 3.0
targets = []

manualDriving = False

leftSpeed = 0.0
rightSpeed = 0.0

emitter.send(f"register,{robot.getName()}".encode('utf-8'))

def forward(speed):
    global leftSpeed, rightSpeed
    leftSpeed = speed
    rightSpeed = speed
def backward(speed):
    global leftSpeed, rightSpeed
    leftSpeed = -speed
    rightSpeed = -speed
def turnLeft(speed):
    global leftSpeed, rightSpeed
    leftSpeed = -speed
    rightSpeed = speed
def turnRight(speed):
    global leftSpeed, rightSpeed
    leftSpeed = speed
    rightSpeed = -speed
def stop():
    global leftSpeed, rightSpeed
    leftSpeed = 0.0
    rightSpeed = 0.0

def setMotorSpeeds():
    global leftSpeed, rightSpeed
    left_motor.setVelocity(leftSpeed)
    right_motor.setVelocity(rightSpeed)

while robot.step(timestep) != -1:
    key = keyboard.get_key()
    status_led.set(0x00FF00)
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        name, x, y = tuple(map(str, message.split(',')))
        if name == robot.getName():
            target = (float(x), float(y))
            targets.append(target)
            print("Targets received: " + str(targets))
        receiver.nextPacket()
    #print(key)
    #print("Targets: " + str(targets))
    #print("position: "+ str(gps.getValues()) + "direction: " + str((compass.getValues())))
    if key == "M":
        manualDriving = True
    elif key == "T":
        manualDriving = False
    if manualDriving:
        status_led.set(0x0000FF)
        stop()
        if key == "W":
            forward(speed)
            #print("here")
        elif key == "S":
            backward(speed)
        elif key == "A":
            turnLeft(speed)
            #print("here")
        elif key == "D":
            turnRight(speed)
    elif len(targets) > 0:
        status_led.set(0xFF0000)
        position = gps.getValues()
        dx = targets[0][0] - position[0]
        dy = targets[0][1] - position[1]
        targetAngle = math.atan2(dy, dx)
        #print("target Angle: " + str(math.degrees(targetAngle)))
        currentAngle = math.atan2(compass.getValues()[0], compass.getValues()[1])
        #print("current Angle: " + str(math.degrees(currentAngle)))
        angleDiff = targetAngle - currentAngle
        #print("angle Diff: " + str(math.degrees(angleDiff)))
        if angleDiff > 0.1:
            turnRight(speed)
            #print("turning right")
        elif angleDiff < -0.1:
            turnLeft(speed)
        else:
            forward(speed)
        distance = math.sqrt(dx**2 + dy**2)
        #print("distance to target: " + str(distance))
        if distance < 0.5:
            targets.pop(0)
            print("Target reached! Remaining targets: " + str(len(targets)))
            emitter.send(f"complete,{robot.getName()}".encode('utf-8'))
            stop()
            if len(targets) == 0:
                print("All targets reached!")
            setMotorSpeeds()
            for _ in range(int(wait_time * 1000 / timestep)):
                #print("Waiting for " + str(wait_time) + " seconds...")
                status_led.set(0x00FFFF)
                robot.step(timestep)
    
    setMotorSpeeds()