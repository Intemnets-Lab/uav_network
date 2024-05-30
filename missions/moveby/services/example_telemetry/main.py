# Copyright (c) 2023 Parrot Drones SAS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
# * Neither the name of the Parrot Company nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# PARROT COMPANY BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.

'''
     This file is part of telemetry_thermal_versions.

    telemetry_thermal_versions is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

    telemetry_thermal_versions is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    GNU General Public License: <https://www.gnu.org/licenses/>. 
'''

import asyncio
import logging
import signal
import ulog
import telemetry_binding
import time
import math
import paramiko
from scp import SCPClient
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice #new
from digi.xbee.models.address import XBee64BitAddress #new
# This is limited to 15 charecters
PROCESS_NAME = b"ex_tlm_py"


async def service_main():
    # Initialisation code
    #
    # The service is automatically started by the drone when the mission is
    # loaded.
    ulog.setup_logging(PROCESS_NAME)
    logger = logging.getLogger("main")
    logger.info("Hello from example telemetry")
    run = True
    def sig_handler(*_):
        nonlocal run
        run = False

    # Setup Consumer
    consumer = telemetry_binding.Consumer("/dev/shm")

    # Setup Sensors
    altitude = telemetry_binding.types.float32_t(0.0)
    drone_attitude_w = telemetry_binding.types.float32_t(0.0)
    drone_attitude_x = telemetry_binding.types.float32_t(0.0)
    drone_attitude_y = telemetry_binding.types.float32_t(0.0)
    drone_attitude_z = telemetry_binding.types.float32_t(0.0)
    ################ GPS ##################
    gps_latitude = telemetry_binding.types.float64_t(0.0)
    gps_longitude = telemetry_binding.types.float64_t(0.0)
    gps_altitude = telemetry_binding.types.float64_t(0.0)
    gps_satellites = telemetry_binding.types.int32_t(0)
    ############### Gimbal ################
    gimbal_attitude_w = telemetry_binding.types.float32_t(0.0)
    gimbal_attitude_x = telemetry_binding.types.float32_t(0.0)
    gimbal_attitude_y = telemetry_binding.types.float32_t(0.0)
    gimbal_attitude_z = telemetry_binding.types.float32_t(0.0)

    consumer.reg(altitude, "drone_controller.altitude_agl")
    consumer.reg(drone_attitude_w, "drone_controller.attitude_ned_q.w")
    consumer.reg(drone_attitude_x, "drone_controller.attitude_ned_q.x")
    consumer.reg(drone_attitude_y, "drone_controller.attitude_ned_q.y")
    consumer.reg(drone_attitude_z, "drone_controller.attitude_ned_q.z")
    consumer.reg(gps_latitude, "drone_controller.position_geo.latitude_north_deg")
    consumer.reg(gps_longitude, "drone_controller.position_geo.longitude_east_deg")
    consumer.reg(gps_altitude, "sensors_gps.location.altitude_amsl")
    consumer.reg(gps_satellites, "sensors_gps.extra.sat_used_count")
    consumer.reg(gimbal_attitude_w, "fcam@eis@video.view_ned_start_q.w")
    consumer.reg(gimbal_attitude_x, "fcam@eis@video.view_ned_start_q.x")
    consumer.reg(gimbal_attitude_y, "fcam@eis@video.view_ned_start_q.y")
    consumer.reg(gimbal_attitude_z, "fcam@eis@video.view_ned_start_q.z")
    consumer.regComplete()

    PATH_DATA = "/mnt/user-internal/missions-data/com.parrot.missions.samples.move/telemetry_data.csv"
    PATH_GUIDANCE = "/mnt/user-internal/missions-data/com.parrot.missions.samples.move/telemetry_service.txt" #new #while true; do cat telemetry_service.txt | wc -l; sleep 5; done
    server = '192.168.42.99'
    port = 22
    user = 'intemnetslab'
    password = '1357924680'
    local_path = '/mnt/user-internal/missions-data/com.parrot.missions.samples.move/'

    # write header once
    with open(PATH_DATA, 'w') as file:
        file.write("Time,Altitude,Drone Roll,Drone Pitch,Drone Yaw,Gimbal Roll,Gimbal Pitch,Gimbal Yaw,GPS Latitude,GPS Longitude,GPS Altitude,GPS Satellites\n")
    # ssh_client = create_ssh_client(server, port, user, password)
    # theFile = "testfile.txt"
    # scp_transfer_file(ssh_client, theFile, local_path)
    # ssh_client.close()
    while run:
        consumer.getSample(
            telemetry_binding.types.timespec(0, 0),
            telemetry_binding.Method.Latest,
        )
        
        currentTime = time.strftime("%m_%d_%Y_%H:%M:%S") + ".png"

        # convert Orientation into euler angles from quaternion
        drone_roll, drone_pitch, drone_yaw = euler_from_quaternion(float(drone_attitude_x), float(drone_attitude_y), float(drone_attitude_z), float(drone_attitude_w))
        gimbal_roll, gimbal_pitch, gimbal_yaw = euler_from_quaternion(float(gimbal_attitude_x), float(gimbal_attitude_y), float(gimbal_attitude_z), float(gimbal_attitude_w))

        data_string = f"{currentTime},{altitude},{drone_roll},{drone_pitch},{drone_yaw},{gimbal_roll},{gimbal_pitch},{gimbal_yaw},{gps_latitude},{gps_longitude},{gps_altitude},{gps_satellites}\n"

        with open(PATH_DATA, 'a') as file:
            file.write(data_string)

        with open(PATH_GUIDANCE, 'a') as file:#new
            file.write("telemetry_py example mission service\n")

        try:
            ssh_client = create_ssh_client(server, port, user, password)
            take_snapshot(ssh_client, currentTime)
            await asyncio.sleep(5)
            while True:
                try:
                    if check_remote_file_exists(ssh_client, currentTime):
                        scp_transfer_file(ssh_client, currentTime, local_path)
                        # await asyncio.sleep(1)
                        break
                        #check to see if the file is available locally and you dont continue until its there locally
                        #transmit floating point numbers
                except:
                     pass
            await asyncio.sleep(1)
            delete_remote_file(ssh_client, currentTime)
            ssh_client.close()
            xbee = XBeeDevice("/dev/ttyUSB0", 9600)
            xbee.open()
            remote = RemoteXBeeDevice(xbee, XBee64BitAddress.from_hex_string("0013A20040D5D3C5"))
            xbee.send_data(remote, data_string)
            # integer_value = 48
            # xbee.send_data(remote, integer_value.to_bytes(len(str(integer_value)), 'big'))
            xbee.close()
        except:
             pass
        
        await asyncio.sleep(5)
    # Cleanup code
    logger.info("Cleaning up")
    consumer = None
    
    return 0


def main():
    asyncio.run(service_main())

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return math.degrees(roll_x), math.degrees(pitch_y), math.degrees(yaw_z) # in radians

def create_ssh_client(server, port, user, password):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # Automatically add host key
        client.connect(server, port=port, username=user, password=password)
        return client

def scp_transfer_file(ssh_client, remote_path, local_path):
        with SCPClient(ssh_client.get_transport()) as scp:
            scp.get(remote_path, local_path)
    
def delete_remote_file(ssh_client, remote_path):
    ssh_client.exec_command(f'rm {remote_path}')

def check_remote_file_exists(ssh_client, remote_path):
    _, stdout, _ = ssh_client.exec_command(f'test -f {remote_path} && echo "1" || echo "0"')
        # _, stdout, _ = ssh_client.exec_command(f'[[ $(find ~ -maxdepth 1 -name "*.png" | wc -l) -gt 0 ]] && echo "1" || echo "0"')
    return stdout.read().strip().decode() == '1'
    
def take_snapshot(ssh_client, now):
    _, _, _ = ssh_client.exec_command(f'python3 ~/Desktop/thermal_snapshot_V2.py {now}')
    
