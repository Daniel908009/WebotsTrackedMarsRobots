from controller import Supervisor
import random

coordinator = Supervisor()

timestep = int(coordinator.getBasicTimeStep())
next_target_time = 0
target_interval = 20.0
unassigned_targets = []
registered_rovers = {}
emitter = coordinator.getDevice("emitter")
receiver = coordinator.getDevice("receiver")
receiver.enable(timestep)

def canSendTarget():
    return True

while coordinator.step(timestep) != -1:
    if receiver.getQueueLength() > 0:
        message = receiver.getString()
        task, roverName = tuple(map(str, message.split(',')))
        if task == "register":
            registered_rovers[roverName] = "available"
        elif task == "complete":
            registered_rovers[roverName] = "available"
        receiver.nextPacket()
    if coordinator.getTime() >= next_target_time:
        target = (random.uniform(-10, 10), random.uniform(-10, 10))
        unassigned_targets.append(target)
        next_target_time += target_interval
    if canSendTarget() and len(unassigned_targets) > 0:
        target = unassigned_targets.pop(0)
        available_rovers = [rover for rover, status in registered_rovers.items() if status == "available"]
        if available_rovers:
            roverName = random.choice(available_rovers)
            registered_rovers[roverName] = "busy"
            message = f"{roverName},{target[0]},{target[1]}".encode('utf-8')
            emitter.send(message)