## Initial installation

To run the script you'll need to install Olympe. To do so follow the set up guide. (https://developer.parrot.com/docs/olympe/installation.html)

You will also need to install python on your machine. This following guide will help install python on whatever system you are using. (https://kinsta.com/knowledgebase/install-python/#linux)

Another essential requirement is installing Air SDK 7.7 onto your system. ([https://airsdk.dev/docs/basics/install/windows](https://developer.parrot.com/docs/airsdk/general/installation.html))

# Drone Automation Services
For the drone to make it's own movement commands, it uses the following services named "go_to" and "drone".

At the beginning you must have the drone on with it's wings extended and placed in an area with a lot of space. Then from a computer you need to connect to the drone's wifi network. Make sure that computer you use also has a wired network connection seprate from the drone, not doing so will result in an error.

To run the services and have the drone take off, you need to build and install the "moveby" mission into the device. From the computer's terminal, you need to navigate into the mission folder labeled "move" and run the command

```bash
airsdk uninstall com.parrot.missions.samples.milcom && airsdk build && airsdk install --default
```
This command uninstalls the exsisting mission from the drone, then builds and installs a new one as the default with all the code in this folder. If no milcom mission exists yet, then just enter.
```bash
airsdk build && airsdk install --default 
```
Make sure the mission you install has these service folders labeled correctly. Since it's set to default the mission will automaticly activate when the installtion is done, however the drone won't take off.

To get the drone in the air, from the terminal you need to command into the Olympe folder and execute the script "missiontestland.py" in a python3 shell.
```bash
python3 missiontestland.py
```
The script has only one command, to forcefully make the drone either take off or land depending on it's current state. Once in the air the mission will begin to execute it's flight commands. Once all waypoints have been visited, the drone will automatically land. Running the script again while the drone is already in the air will immediately trigger a landing.

Note: If the mission fails to build, that usually indicates an error in the code. Look at where the error is located to find and resolve the issue. Any error that pops up that mentions "media" hasn't appeared to stop the drone from executing it's flight plan, so keep that in mind.

### WARNING

Make sure to test this mission in an area with a lot of space and little obstacles. The drone may move or drift a bit as it calibrates it self in the air, so avoid testing in tight spaces if you can and keep a safe distance.

The move_along and new_inputs services in this folder are the same services that can be found in the "move" mission, I've provided it here for quicker access. In the folder you will find the following files.
# go_to

main.cpp - This is the main file that contain the primarly code loop for the mission

control_interface.cpp - This file deals with the sending and receveing of commands. Commmands will be for the drone to move or land for example.

configuration.cpp - Sets up the inital configuration for values and functions in the control_interface file.

Each cpp file will also have an associated hpp file.

# drone
main.py - This is the main entry point for the mission. It initializes and launches the drone logic by creating an instance of the Drone class.

drone_class.py - Contains the main logic for autonomous drone operations within the mission. Handles all major functionalities of the drone, including telemetry data collection, waypoint navigation, photo capture through SSH to a remote Raspberry Pi, and communication using XBee radios. It also handle socket communication and waypoint progression to the control_interface.cpp.

way.py - The waypoints are stored here.

receive.py - This script is used to listens for XBee data packets.