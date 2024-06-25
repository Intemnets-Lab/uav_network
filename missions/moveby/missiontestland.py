import olympe
import time
import os
import olympe.messages.ardrone3.Piloting as piloting

DRONE_IP="192.168.42.1"
drone = olympe.Drone(DRONE_IP)
drone.connect()
assert drone(piloting.Landing()).wait().success()
drone.disconnect()
	
