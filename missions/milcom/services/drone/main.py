import time
import sys
import os
import time
import asyncio
import logging
import signal
import ulog

PROCESS_NAME = b"milcom"

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
drone_module = __import__('drone_class')
drone = drone_module.Drone

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
    my_drone = drone("/dev/ttyUSB0", 9600, "2", "Monitoring", "Router", "0013A20040D5D3C5", 0)

    while True:
        time.sleep(1)    
    return 0


def main():
    asyncio.run(service_main())