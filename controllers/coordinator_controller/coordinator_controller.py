from controller import Supervisor
import random

coordinator = Supervisor()

timestep = int(coordinator.getBasicTimeStep())
next_target_time = 0
target_interval = 15.0
unassigned_targets = []
registered_rovers = {}
emitter = coordinator.getDevice("emitter")
receiver = coordinator.getDevice("receiver")
receiver.enable(timestep)

def canSendTarget():
    return True


def ClosestRover(rovers, target):
    closest_rover = None
    closest_distance = float('inf')
    #print("rovers: " + str(rovers))
    for rover in rovers:
        rover_position = registered_rovers[rover]["position"]
        #print(f"Rover {rover} position: {rover_position}, target: {target}")
        distance = ((rover_position[0] - target[0]) ** 2 + (rover_position[1] - target[1]) ** 2) ** 0.5
        if distance < closest_distance:
            closest_distance = distance
            closest_rover = rover
    return closest_rover


while coordinator.step(timestep) != -1:
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        task, roverName = tuple(map(str, message.split(',')[0:2]))
        if task == "register":
            registered_rovers[roverName] = {"status": "available", "position": (0, 0)}
        elif task == "complete":
            #print(f"Rover {roverName} has completed its task.")
            registered_rovers[roverName]["status"] = "available"
        elif task == "positionUpdate":
            x, y = tuple(map(float, message.split(',')[2:]))
            registered_rovers[roverName]["position"] = (x, y)
        elif task == "targetReturn":
            registered_rovers[roverName]["status"] = "unavailable"
            target = tuple(map(float, message.split(',')[2:]))
            #print(f"Rover {roverName} returned target {target}. Adding back to unassigned targets.")
            unassigned_targets.append(target)
        elif task == "unavailable":
            #print(f"Rover {roverName} is now unavailable.")
            registered_rovers[roverName]["status"] = "unavailable"
        receiver.nextPacket()
    if coordinator.getTime() >= next_target_time:
        target = (random.uniform(-10, 10), random.uniform(-10, 10))
        unassigned_targets.append(target)
        next_target_time += target_interval
    if canSendTarget() and len(unassigned_targets) > 0:
        target = unassigned_targets.pop(0)
        available_rovers = [rover for rover, info in registered_rovers.items() if info["status"] == "available"]
        if available_rovers:
            roverName = ClosestRover(available_rovers, target)
            #print(f"Sending target {target} to rover {roverName}")
            registered_rovers[roverName]["status"] = "busy"
            message = f"target,{roverName},{target[0]},{target[1]}".encode('utf-8')
            emitter.send(message)
