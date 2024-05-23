import asyncio
import logging
import signal
import ulog

# This is limited to 15 charecters
PROCESS_NAME = b"trajectory_determination"

async def service_main():
    # Initialisation code
    ulog.setup_logging(PROCESS_NAME)
    logger = logging.getLogger("main")
    run = True
    def sig_handler(*_):
        nonlocal run
        run = False

    loop = asyncio.get_running_loop()
    loop.add_signal_handler(signal.SIGTERM, sig_handler)
    #set up different paths
    PATH_STATUS = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/statusfile.txt"
    PATH_WAYPOINT = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/waypointfile.txt"
    PATH_STATUSTEST = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/statustest.txt"
    PATH_HEADINGTEST = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/headingtest.txt"
    PATH_COOR = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/coortest.txt"
    PATH_END="/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/endtest.txt"

    await asyncio.sleep(5)
    # file path
    # Loop Code
    #coordinate array
    coor=["3.1, 0, 0, 0\n", "3.2, 0, 0, 0\n","2.0, 0, 0, 0\n","4.0, 0, 0, 0\n"]
    counter=0;
    while run:
        #if the counter reaches the length of the array, then have the loop skip all the code and start the loop again.
        #This will have the drone just hover until another command is added to to the array.
        if counter==len(coor):
            continue;
        #try to open the status file. Use while loop to wait and try to open status file.
        #Have the service wait about 15 second at the beginning of the loop.
        await asyncio.sleep(15)
        while True:
            try:
                with open(PATH_STATUS,"r") as rfile:
                    value =rfile.read().strip()
            except:
                #wait five seconds before trying again.
                await asyncio.sleep(5)
            else:
                break
        confirm= open(PATH_STATUSTEST, "a")
        #confirmation of status being opened
        confirm.write("was able to open status\n")
        confirm.close()

        #check the status. 1 means ready for new waypoint, the program then send a new waypoint to the waypoint file
        if (value=="1"):
            #while loop used to wait until we open the waypoint file and override it's content with a new value. 
            while True:
                try:
                   waypoint= open(PATH_WAYPOINT, "w")
                except:
                    #wait 5 seconds before trying to open the waypoint file again.
                    await asyncio.sleep(5)
                else:
                    break
            confirm2= open(PATH_HEADINGTEST, "a")
            #confirmation of opening heading value.
            confirm2.write("was able to open heading\n")
            confirm2.close()
            #write the new way point to the file
            waypoint.write(coor[counter])
            waypoint.close()
            #log all the coordinates that should be processed 
            with open(PATH_COOR,"a") as doc:
                doc.write(coor[counter])
            # increment counter
            counter+=1
            #Change statuss value to let the go_to service know that a new way point has been entered.
            #The serive will then start waiting again until status value is 1. 
            while True:
                try:
                    with open(PATH_STATUS,"w") as file:
                        file.write("0\n")
                except:
                    #wait 3 seconds before trying to open the status file again.
                    await asyncio.sleep(3)
                else:
                    break
        else:
            confirm2= open(PATH_HEADINGTEST, "a")
            confirm2.write("was not able to open heading\n")
            confirm2.close()

    # Cleanup code
    # await asyncio.sleep(5)
    logger.info("Cleaning up")
    return 0

def main():
    asyncio.run(service_main())