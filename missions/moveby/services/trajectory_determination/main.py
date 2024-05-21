import asyncio
import logging
import signal
import ulog

# This is limited to 15 charecters
PROCESS_NAME = b"new_inputs"

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
    PATH_HEADING = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.move/waypointfile.txt"
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
        #if the counter reaches the length of the array, then have the loop skip all the code and have the drone just hover
        if counter==len(coor):
            continue;
        #try to open the status file.
        await asyncio.sleep(15)
        while True:
            try:
                # status = open(PATH_STATUS, "r")
                with open(PATH_STATUS,"r") as rfile:
                    value =rfile.read().strip()
            except:
                await asyncio.sleep(5)
            else:
                break
        confirm= open(PATH_STATUSTEST, "a")
        confirm.write("was able to open status\n")
        confirm.close()

        #check the status value and respond to whatever it's value is.
        if (value=="1"):
            #while loop to open heading file and override the value with a new value.
            while True:
                try:
                   heading= open(PATH_HEADING, "w")
                except:
                    await asyncio.sleep(5)
                else:
                    break
            confirm2= open(PATH_HEADINGTEST, "a")
            confirm2.write("was able to open heading\n")
            confirm2.close()
            heading.write(coor[counter])
            heading.close()
            #log all the coordinates that should be processed 
            with open(PATH_COOR,"a") as doc:
                doc.write(coor[counter])
            # increment counter
            counter+=1
            while True:
                try:
                    with open(PATH_STATUS,"w") as file:
                        file.write("0\n")
                except:
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