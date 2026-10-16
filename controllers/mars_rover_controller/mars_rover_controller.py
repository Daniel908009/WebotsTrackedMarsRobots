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


# collision stuff
distanceSensorLeft = robot.getDevice("distanceSensorLeft")
distanceSensorRight = robot.getDevice("distanceSensorRight")
distanceSensorFront = robot.getDevice("distanceSensorFront")
if distanceSensorLeft is None or distanceSensorRight is None or distanceSensorFront is None:
    pass
else:
    distanceSensorLeft.enable(timestep)
    distanceSensorRight.enable(timestep)
    distanceSensorFront.enable(timestep)


gps.enable(timestep)
compass.enable(timestep)
receiver.enable(timestep)

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

speed = 0.5
DistanceThreshold = 0.8
targets = []

manualDriving = False
collisionStop = False
avoidingObstacle = False
avoidingObstacleTime = 3
currentAvoidingTime = 0

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
    leftSpeed = speed
    rightSpeed = -speed
def turnRight(speed):
    global leftSpeed, rightSpeed
    leftSpeed = -speed
    rightSpeed = speed
def stop():
    global leftSpeed, rightSpeed
    leftSpeed = 0.0
    rightSpeed = 0.0

def setMotorSpeeds():
    global leftSpeed, rightSpeed
    left_motor.setVelocity(leftSpeed)
    right_motor.setVelocity(rightSpeed)

def HandleCollisionSafety():
    global collisionStop, avoidingObstacle, currentAvoidingTime
    if distanceSensorLeft is None or distanceSensorRight is None or distanceSensorFront is None: return
    left_distance = distanceSensorLeft.getValue()
    right_distance = distanceSensorRight.getValue()
    front_distance = distanceSensorFront.getValue()
    #print(f"Left distance: {left_distance}, Right distance: {right_distance}, Front distance: {front_distance}")
    if left_distance < 5 or right_distance < 5 or front_distance < 5:
        #print("Collision detected! Stopping the rover.")
        collisionStop = True
        stop()
        ReturnTargets()
    elif left_distance < right_distance or front_distance < 80:
        #print("Obstacle detected on the left")
        turnRight(speed/2)
        avoidingObstacle = True
        currentAvoidingTime = 0
    elif right_distance < left_distance:
        #print("Obstacle detected on the right")
        turnLeft(speed/2)
        avoidingObstacle = True
        currentAvoidingTime = 0

def ReturnTargets():
    global targets
    emitter.send(f"unavailable,{robot.getName()}".encode('utf-8'))
    for _ in range(targets.__len__()):
        emitter.send(f"targetReturn,{robot.getName()},{targets[0][0]},{targets[0][1]}".encode('utf-8'))
        targets.pop(0)

while robot.step(timestep) != -1:
    key = keyboard.get_key()
    if not collisionStop:
        status_led.set(0x00FF00)
    else:
        status_led.set(0xFFA500)
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        task, name = tuple(map(str, message.split(',')[0:2]))
        if name == robot.getName():
            if task == "target":
                x, y = tuple(map(float, message.split(',')[2:]))
                target = (float(x), float(y))
                targets.append(target)
                print("Rover " + robot.getName() + " Target received: " + str(targets))
        receiver.nextPacket()
    #print(key)
    #print("Targets: " + str(targets))
    #print("position: "+ str(gps.getValues()) + "direction: " + str((compass.getValues())))
    emitter.send(f"positionUpdate,{robot.getName()},{gps.getValues()[0]},{gps.getValues()[1]}".encode('utf-8'))
    if key == "M":
        manualDriving = True
        collisionStop = False
        ReturnTargets()
    elif key == "T":
        manualDriving = False
        emitter.send(f"available,{robot.getName()}".encode('utf-8'))
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
    elif len(targets) > 0 and not collisionStop and not avoidingObstacle:
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
            turnLeft(speed)
            #print("turning right")
        elif angleDiff < -0.1:
            turnRight(speed)
        else:
            forward(speed)
        distance = math.sqrt(dx**2 + dy**2)
        #print("distance to target: " + str(distance))
        if distance < DistanceThreshold:
            target = targets[0]
            stop()
            setMotorSpeeds()

            wait_time = 3.0
            while robot.step(timestep) != -1:
                #print("Waiting for " + str(wait_time) + " seconds...")
                wait_time -= timestep / 1000.0
                if wait_time <= 0:
                    break
                status_led.set(0x00FFFF)
                robot.step(timestep)
            emitter.send(f"complete,{robot.getName()},{target[0]},{target[1]}".encode('utf-8'))
            targets.pop(0)
            print("Rover " + robot.getName() + " Target reached! Remaining targets: " + str(len(targets)))
            #if len(targets) == 0:
                #print("Rover " + robot.getName() + " Tasks completed!")
        HandleCollisionSafety()
    elif avoidingObstacle:
        #print("Avoiding obstacle...")
        status_led.set(0xFFFF00)
        forward(speed/2)
        HandleCollisionSafety()
        currentAvoidingTime += timestep / 1000.0
        if currentAvoidingTime >= avoidingObstacleTime:
            avoidingObstacle = False
            currentAvoidingTime = 0
    
    setMotorSpeeds()