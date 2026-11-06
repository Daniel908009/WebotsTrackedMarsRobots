from controller import Supervisor
import random

coordinator = Supervisor()

timestep = int(coordinator.getBasicTimeStep())
next_target_time = 0
target_interval = 15.0
unassigned_targets = [(6,0), (6,0), (6,0), (6,0)] # template rock spawn position 
registered_rovers = {}
stopped_rovers = []
maximumTargetsPerRover = 3
rocks = {}
emitter = coordinator.getDevice("emitter")
receiver = coordinator.getDevice("receiver")
receiver.enable(timestep)
rockNumber = 0
safetyDistance = 8

def UpdateStoppedList():
    for i in range(len(registered_rovers) - 1, -1, -1):
        for j in range(len(registered_rovers) - 1, -1, -1):
            if i != j:
                rover_i = list(registered_rovers.keys())[i]
                rover_j = list(registered_rovers.keys())[j]
                position_i = registered_rovers[rover_i]["position"]
                position_j = registered_rovers[rover_j]["position"]
                distance = ((position_i[0] - position_j[0]) ** 2 + (position_i[1] - position_j[1]) ** 2) ** 0.5
                #print(f"Distance between {rover_i} and {rover_j}: {distance}")
                if distance < safetyDistance and stopped_rovers.count({"roverStopped": rover_i, "roverWithPriority": rover_j}) == 0 and stopped_rovers.count({"roverStopped": rover_j, "roverWithPriority": rover_i}) == 0:
                    print(f"Rover {rover_i} and Rover {rover_j} are too close! Distance: {distance}. Determining which rover has priority.")
                    if len(registered_rovers[rover_i]["targets"]) > len(registered_rovers[rover_j]["targets"]):
                        stopped_rovers.append({"roverStopped": rover_j, "roverWithPriority": rover_i})
                        message = f"stop,{rover_j}".encode('utf-8')
                        emitter.send(message)
                    elif len(registered_rovers[rover_i]["targets"]) < len(registered_rovers[rover_j]["targets"]):
                        stopped_rovers.append({"roverStopped": rover_i, "roverWithPriority": rover_j})
                        message = f"stop,{rover_i}".encode('utf-8')
                        emitter.send(message)
                    else:
                        if i < j:
                            stopped_rovers.append({"roverStopped": rover_i, "roverWithPriority": rover_j})
                            message = f"stop,{rover_i}".encode('utf-8')
                            emitter.send(message)
                        else:
                            stopped_rovers.append({"roverStopped": rover_j, "roverWithPriority": rover_i})
                            message = f"stop,{rover_j}".encode('utf-8')
                            emitter.send(message)
                elif stopped_rovers.count({"roverStopped": rover_i, "roverWithPriority": rover_j}) > 0 or stopped_rovers.count({"roverStopped": rover_j, "roverWithPriority": rover_i}) > 0:
                    if distance >= safetyDistance:
                        print(f"Removing from stopped_rovers: {rover_i} and {rover_j} as distance is now {distance}")
                        if stopped_rovers.count({"roverStopped": rover_i, "roverWithPriority": rover_j}) > 0:
                            stopped_rovers.remove({"roverStopped": rover_i, "roverWithPriority": rover_j})
                            message = f"resume,{rover_i}".encode('utf-8')
                            emitter.send(message)
                        if stopped_rovers.count({"roverStopped": rover_j, "roverWithPriority": rover_i}) > 0:
                            stopped_rovers.remove({"roverStopped": rover_j, "roverWithPriority": rover_i})
                            message = f"resume,{rover_j}".encode('utf-8')
                            emitter.send(message)

def ClosestRoverPath(rovers, target):
    closest_rover = None
    closest_distance = float('inf')
    print("rovers positions: " + str({rover: registered_rovers[rover]["position"] for rover in rovers}))
    for rover in rovers:
        rover_position = registered_rovers[rover]["position"]
        #print(f"Rover {rover} position: {rover_position}, target: {target}")
        distance = ((rover_position[0] - target[0]) ** 2 + (rover_position[1] - target[1]) ** 2) ** 0.5
        if distance < closest_distance:
            closest_distance = distance
            closest_rover = rover
        for targetPoint in registered_rovers[rover]["targets"]:
            distance = ((targetPoint[0] - target[0]) ** 2 + (targetPoint[1] - target[1]) ** 2) ** 0.5
            if distance < closest_distance:
                closest_distance = distance
                closest_rover = rover
    print(f"Closest rover to target {target} is {closest_rover} at distance {closest_distance}")
    return closest_rover

def SpawnRock(x, y):
    global rockNumber
    template = coordinator.getFromDef("TemplateRock")
    root = coordinator.getRoot()
    if template is not None:
        rock_string = template.exportString()
        rock_string = rock_string.replace("TemplateRock", f"Rock_{rockNumber}")
        root.getField("children").importMFNodeFromString(-1, rock_string)
        new_rock = coordinator.getFromDef(f"Rock_{rockNumber}")
        new_rock.getField("translation").setSFVec3f([x, y, 0])
        new_rock.getField("name").setSFString(f"Rock_{rockNumber}")
        rockNumber += 1
        rocks[f"Rock_{x}_{y}"] = new_rock

frameCount = 0
while coordinator.step(timestep) != -1:
    frameCount += 1
    while receiver.getQueueLength() > 0:
        message = receiver.getString()
        task, roverName = tuple(map(str, message.split(',')[0:2]))
        if task == "register":
            registered_rovers[roverName] = {"status": "available", "position": (0, 0), "targets": []}
        elif task == "complete":
            #print(f"Rover {roverName} has completed its task. Coordinator")
            target_x, target_y = tuple(map(float, message.split(',')[2:]))
            if f"Rock_{target_x}_{target_y}" in rocks:
                rocks[f"Rock_{target_x}_{target_y}"].remove()
                del rocks[f"Rock_{target_x}_{target_y}"]
            registered_rovers[roverName]["status"] = "available"
            registered_rovers[roverName]["targets"].remove((target_x, target_y))
        elif task == "positionUpdate":
            x, y = tuple(map(float, message.split(',')[2:]))
            registered_rovers[roverName]["position"] = (x, y)
        elif task == "targetReturn":
            registered_rovers[roverName]["status"] = "unavailable"
            target = tuple(map(float, message.split(',')[2:]))
            #print(f"Rover {roverName} returned target {target}. Adding back to unassigned targets.")
            registered_rovers[roverName]["targets"].remove(target)
            unassigned_targets.append(target)
        elif task == "unavailable":
            #print(f"Rover {roverName} is now unavailable.")
            registered_rovers[roverName]["status"] = "unavailable"
        elif task == "available":
            registered_rovers[roverName]["status"] = "available"
        receiver.nextPacket()
    if coordinator.getTime() >= next_target_time and len(unassigned_targets) < 8:
        target = (random.uniform(-10, 10), random.uniform(-10, 10))
        SpawnRock(target[0], target[1])
        unassigned_targets.append(target)
        next_target_time += target_interval
    if len(unassigned_targets) > 0 and frameCount > 1:
        target = unassigned_targets[0]
        available_rovers = [rover for rover, info in registered_rovers.items() if info["status"] != "unavailable" and len(info["targets"]) < maximumTargetsPerRover]
        if available_rovers:
            roverName = ClosestRoverPath(available_rovers, target)
            #print(f"Assigning")
            registered_rovers[roverName]["status"] = "busy"
            registered_rovers[roverName]["targets"].append(target)
            #print("registered rover targets: " + str(registered_rovers[roverName]["targets"]))
            message = f"target,{roverName},{target[0]},{target[1]}".encode('utf-8')
            emitter.send(message)
            print("popping unassigned target: " + str(unassigned_targets[0]))
            unassigned_targets.pop(0)
    if frameCount > 1:
        UpdateStoppedList()
