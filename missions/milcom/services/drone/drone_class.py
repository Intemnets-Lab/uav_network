import asyncio
import time
from datetime import datetime
import threading
import paramiko
import math
import telemetry_binding
import way
import socket
from scp import SCPClient
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
# import random
import struct

PATH_DATA = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/telemetry_data.csv"
# PATH_GUIDANCE = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/drone1cofirm.txt"
# PATH_HEADINGTEST = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/statusCHECK.txt"
# PATH_COOR = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/coortest.txt"
# PATH_SERVICE = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/telemetry_service.txt" #new #while true; do cat telemetry_service.txt | wc -l; sleep 5; done
# PATH_TEST = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/telemetry_test.txt"
PATH_CHECK = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/datatest.txt"
PATH_SOCKET = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/socket_python.txt"
PATH_ERROR = "/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/errorcheck.txt"
class Drone:
    # SSH Server at Raspberry Pi
    server = '192.168.42.99'
    nport = 22
    user = 'intemnetslab'
    password = '1357924680'
    local_path = '/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/'
    with open(PATH_DATA, 'w') as file:
        file.write("Time, Altitude, Drone Roll, Drone Pitch, Drone Yaw, Gimbal Roll, Gimbal Pitch, Gimbal Yaw, GPS Latitude, GPS Longitude, GPS Altitude, GPS Satellites\n")

    def __init__(self, port, baud_rate, drone_id, mode, drone_type, next_hop_address,status):

        # """
        # Initialize the XBeeHandler with a given port, baud rate, and device name.
        # """
        # self.status = status
        # self.drone_type = drone_type
        # self.mode = mode
        # self.xbee_radio = XBeeDevice(port, baud_rate)
        # self.drone_id = drone_id
        # # self.broadcast_message = f"Hello from {self.drone_id}!"
        # self.xbee_radio.open()
        # self.next_hop_address = next_hop_address

        # try:
        #     self.s.bind(('127.0.0.1', self.port))
        # except Exception as e:
        #     with open(PATH_ERROR,"w") as file:
        #         file.write(f"error occurred: {e} ")


        # Setup Consumer
        self.consumer = telemetry_binding.Consumer("/dev/shm")
        # Setup Sensors
        self.altitude = telemetry_binding.types.float32_t(0.0)
        self.drone_attitude_w = telemetry_binding.types.float32_t(0.0)
        self.drone_attitude_x = telemetry_binding.types.float32_t(0.0)
        self.drone_attitude_y = telemetry_binding.types.float32_t(0.0)
        self.drone_attitude_z = telemetry_binding.types.float32_t(0.0)
        ################ GPS ##################
        self.gps_latitude = telemetry_binding.types.float64_t(0.0)
        self.gps_longitude = telemetry_binding.types.float64_t(0.0)
        self.gps_altitude = telemetry_binding.types.float64_t(0.0)
        self.gps_satellites = telemetry_binding.types.int32_t(0)
        ############### Gimbal ################
        self.gimbal_attitude_w = telemetry_binding.types.float32_t(0.0)
        self.gimbal_attitude_x = telemetry_binding.types.float32_t(0.0)
        self.gimbal_attitude_y = telemetry_binding.types.float32_t(0.0)
        self.gimbal_attitude_z = telemetry_binding.types.float32_t(0.0)

        self.consumer.reg(self.altitude, "drone_controller.altitude_agl")
        self.consumer.reg(self.drone_attitude_w, "drone_controller.attitude_ned_q.w")
        self.consumer.reg(self.drone_attitude_x, "drone_controller.attitude_ned_q.x")
        self.consumer.reg(self.drone_attitude_y, "drone_controller.attitude_ned_q.y")
        self.consumer.reg(self.drone_attitude_z, "drone_controller.attitude_ned_q.z")
        self.consumer.reg(self.gps_latitude, "drone_controller.position_geo.latitude_north_deg")
        self.consumer.reg(self.gps_longitude, "drone_controller.position_geo.longitude_east_deg")
        self.consumer.reg(self.gps_altitude, "sensors_gps.location.altitude_amsl")
        self.consumer.reg(self.gps_satellites, "sensors_gps.extra.sat_used_count")
        self.consumer.reg(self.gimbal_attitude_w, "fcam@eis@video.view_ned_start_q.w")
        self.consumer.reg(self.gimbal_attitude_x, "fcam@eis@video.view_ned_start_q.x")
        self.consumer.reg(self.gimbal_attitude_y, "fcam@eis@video.view_ned_start_q.y")
        self.consumer.reg(self.gimbal_attitude_z, "fcam@eis@video.view_ned_start_q.z")
        self.consumer.regComplete()

        self.waypoints = way.get_waypoints()
        self.waypointcounter = 0
        self.status = "HOVERING"
      
        # """
        # Start threads that drive all operations
        # """
        # threading.Thread(target=self.process_received_messages, daemon=True).start()
        # threading.Thread(target=self.print_waiting_message, daemon=True).start()
        # threading.Thread(target=self.send_broadcast_messages_randomly, daemon=True).start()
        # threading.Thread(target=self.switch_mode_periodically, daemon=True).start()
        # if  self.drone_type == "Coordinator":
        #     self.token_timeout = 10
        #     self.token_received = False
        #     self.counter = 0  # Initialize counter
        #     self.token_timer = threading.Timer(self.token_timeout, self.generate_token)
        #     self.token_timer.start()

        #Intialize port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.port = 2000
        self.s.bind(('127.0.0.1', self.port))
        self.s.listen(1)
        #Put the socket server on its own thread
        threading.Thread(target=self.accept_clients, daemon=True).start()

        threading.Thread(target=self.main_loop, daemon=True).start()

        #xbee
        self.xbee_radio = XBeeDevice("/dev/ttyUSB0", 9600)
        try:
            self.xbee_radio.open()
        except Exception as e:
            with open("/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/xbee_log.txt", "a") as log_file:
                log_file.write(f"[XBee INIT ERROR] {e}\n")

        

    def main_loop(self):
        #main loop where we take a photo, collect data
        time.sleep(20)
        while True:
            self.test_xbee()
            #if there is a next waypoint, do current_time, photo, get_telemetry, and goto
            if self.waypointcounter < len(self.waypoints):
                #If the drone is hovering
                if self.status == "HOVERING":
                    current_time = datetime.now().strftime("%m_%d_%Y_%H:%M:%S")
                    self.photo(self.server, self.nport, self.user, self.password, self.local_path, current_time)
                    self.get_telemetry(self.consumer, self.drone_attitude_x, self.drone_attitude_y, self.drone_attitude_z, self.drone_attitude_w, self.gimbal_attitude_x, self.gimbal_attitude_y, self.gimbal_attitude_z, self.gimbal_attitude_w,self.gps_latitude, self.gps_longitude, self.gps_altitude, self.gps_satellites, self.altitude, current_time)
                    self.status = "FLYING"
            #set the status to done after we finsh all the waypoints
            else:
                self.status = "DONE"
                
    def accept_clients(self):
        # Continuously listen for incoming client connections
        while True:
            try:
                client_socket, addr = self.s.accept()
                with open(PATH_SOCKET, 'a') as log:
                    log.write(f"[Socket] Client connected: {addr}\n")
                # Start a new daemon thread to handle communication with this client
                threading.Thread(target=self.handle_client, args=(client_socket,), daemon=True).start()
            except Exception as e:
                with open(PATH_ERROR, 'a') as log:
                    log.write(f"[Socket] Accept failed: {e}\n")

    def handle_client(self, client_socket):
        try:
            while True:
                data = client_socket.recv(1024).decode().strip()
                if not data:
                    break
                with open(PATH_SOCKET, 'a') as log:
                    log.write(f"[Socket] Received: {data}\n")

                #Figure out what command was sent
                if data == "GET_WAYPOINT":
                    waypoint = self.get_waypoint()
                    client_socket.sendall((waypoint + "\n").encode())
                elif data == "GET_STATUS":
                    status = self.status
                    client_socket.sendall((status + "\n").encode())
                elif data == "WAYPOINT_INCREMENTED":
                    self.waypointcounter += 1
                    client_socket.sendall(b"WAYPOINT_INCREMENTED\n")
                elif data.startswith("SET_STATUS:"):
                    new_status = data.split(":", 1)[1].strip()
                    if new_status in {"FLYING", "HOVERING"}:
                        self.status = new_status
                        client_socket.sendall(b"STATUS_UPDATED\n")
                    else:
                        client_socket.sendall(b"INVALID_STATUS\n")

        except Exception as e:
            with open(PATH_ERROR, 'a') as log:
                log.write(f"[Socket] Error: {e}\n")
        finally:
            client_socket.close()
            with open(PATH_SOCKET, 'a') as log:
                log.write("[Socket] Client disconnected\n")

    def get_waypoint(self):
        with open(PATH_SOCKET, 'a') as log:
            log.write(f"[DEBUG] Waypoint counter: {self.waypointcounter}, Total: {len(self.waypoints)}\n")
        if self.waypointcounter < len(self.waypoints):
            wp = self.waypoints[self.waypointcounter]
            return wp
        else:
            return "NO_WAYPOINT"

    def create_ssh_client(self, server, port, user, password):
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # Automatically add host key
        client.connect(server, port=port, username=user, password=password)
        return client
    
    def take_snapshot(self,ssh_client, now):
        _, _, _ = ssh_client.exec_command(f'python3 ~/Desktop/thermal_snapshot_V2.py {now}')

    def delete_remote_file(self, ssh_client, remote_path):
        ssh_client.exec_command(f'rm {remote_path}')

    def scp_transfer_file(self,ssh_client, remote_path, local_path):
        with SCPClient(ssh_client.get_transport()) as scp:
            scp.get(remote_path, local_path)

    def check_remote_file_exists(self, ssh_client, remote_path):
        _, stdout, _ = ssh_client.exec_command(f'test -f {remote_path} && echo "1" || echo "0"')
        # _, stdout, _ = ssh_client.exec_command(f'[[ $(find ~ -maxdepth 1 -name "*.png" | wc -l) -gt 0 ]] && echo "1" || echo "0"')
        return stdout.read().strip().decode() == '1'

    def photo(self, server, port, user, password, local_path, tm):
        try:
            currentTime =  tm + ".png"
            ssh_client = self.create_ssh_client(server, port, user, password)
            self.take_snapshot(ssh_client, currentTime)
            asyncio.sleep(5)
            while True:
                try:
                    if self.check_remote_file_exists(ssh_client, currentTime):
                        self.scp_transfer_file(ssh_client, currentTime, local_path)
                        # with open(PATH_TEST, 'a') as testfile:#new
                            # testfile.write("telemetry_py example mission service\n")
                        # await asyncio.sleep(1)
                        break
                        #check to see if the file is available locally and you dont continue until its there locally
                        #transmit floating point numbers
                except:
                    pass
            asyncio.sleep(1)
            self.delete_remote_file(ssh_client, currentTime)
            ssh_client.close()
        except:
             pass
    
    def euler_from_quaternion(self, x, y, z, w):
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
    

    def get_telemetry(self,consumer, drone_attitude_x, drone_attitude_y, drone_attitude_z, drone_attitude_w, gimbal_attitude_x, gimbal_attitude_y, gimbal_attitude_z, gimbal_attitude_w,gps_latitude, gps_longitude, gps_altitude, gps_satellites, altitude, tm):
        consumer.getSample(
            telemetry_binding.types.timespec(0, 0),
            telemetry_binding.Method.Latest,
        )
        currentTime =  tm + ".png"
        # convert Orientation into euler angles from quaternion
        drone_roll, drone_pitch, drone_yaw = self.euler_from_quaternion(float(drone_attitude_x), float(drone_attitude_y), float(drone_attitude_z), float(drone_attitude_w))
        gimbal_roll, gimbal_pitch, gimbal_yaw = self.euler_from_quaternion(float(gimbal_attitude_x), float(gimbal_attitude_y), float(gimbal_attitude_z), float(gimbal_attitude_w))
        data_string = f"{currentTime},{altitude},{drone_roll},{drone_pitch},{drone_yaw},{gimbal_roll},{gimbal_pitch},{gimbal_yaw},{gps_latitude},{gps_longitude},{gps_altitude},{gps_satellites}\n"
        with open(PATH_DATA, 'a') as file:
            file.write(data_string)
    
    def test_xbee(self):
        #send data using the xbee_radio
        try:
            if not self.xbee_radio.is_open():
                self.xbee_radio.open()
            data1 = 1.0
            data2 = 2.0
            data3 = 3.0
            data_bytes = struct.pack(">fff", data1, data2, data3)
            broadcast_device = RemoteXBeeDevice(self.xbee_radio, XBee64BitAddress.BROADCAST_ADDRESS)
            self.xbee_radio.send_data(broadcast_device, data_bytes)

            #debug
            with open("/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/xbee_log.txt", "a") as log_file:
                log_file.write("[XBee] Data sent successfully.\n")
        except Exception as e:
            with open("/mnt/user-internal/missions-data-tmp/com.parrot.missions.samples.milcom/xbee_log.txt", "a") as log_file:
                log_file.write(f"[XBee] Error {e}\n")


    # def generate_token(self):
    #     if self.mode == "Updating":
    #         print("Timeout: no data received")
    #         self.counter = 0
    #         # Send data again after timeout
    #         float_data1 = 3.14  # Replace with your first float value
    #         float_data2 = 2.71  # Replace with your second float value
    #         self.transmit_to_next_hop(float_data1, float_data2)
    #     self.token_timer = threading.Timer(self.token_timeout, self.generate_token)
    #     self.token_timer.start()
       
        
    # def transmit_to_next_hop(self, data1, data2):
    #     try:
    #         # Convert two floats to bytes
    #         data_bytes = struct.pack(">ff", data1, data2)
    #         print("preparing to send data")
    #         next_hop = RemoteXBeeDevice(self.xbee_radio, XBee64BitAddress.from_hex_string(self.next_hop_address))
    #         print("remote xbee device created")
    #         self.xbee_radio.send_data(next_hop, data_bytes)
    #         print("Data sent from coordinator: {data1}, {data2}")
    #     except Exception as e:
    #         print("Error2: {e}")


    # def receive_from_previous_hop(self):
    #     try:
    #         data = self.xbee_radio.read_data(1000)
    #         if data is not None:
    #             # Unpack bytes to two floats
    #             if len(data.data)>=8:
    #                 data_tuple = struct.unpack(">ff", data.data[:8])
    #                 return data_tuple
    #             # print("data read sucessfully")
    #             # return data_tuple
    #     except Exception as e:
    #         print("Error3: {e}")
    #     return None
    

    # def transmit_broadcast(self, drone_id_float):
    #     """
    #     Broadcast a message to all devices in the XBee network.
    #     Includes a timestamp in the printed message for verification.
    #     """
    #     current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    #     while True:
    #         try:
    #             with open(PATH_GUIDANCE, 'a') as broadfile:#new
    #                 broadfile.write(f"{current_time} - Broadcasting message from {self.drone_id}: '{drone_id_float}'\n")
    #         except:
    #             #wait five seconds before trying again.
    #             asyncio.sleep(3)
    #         else:
    #             break
    #     print(f"{current_time} - Broadcasting message from {self.drone_id}: '{drone_id_float}'")
    #     broadcast_device = RemoteXBeeDevice(self.xbee_radio, XBee64BitAddress.BROADCAST_ADDRESS)
    #     data_bytes = struct.pack(">f", drone_id_float)
    #     self.xbee_radio.send_data_async(broadcast_device, data_bytes)
    # def process_received_messages(self):
    #     """
    #     Continuously process and print any received messages.
    #     Includes a timestamp in the printed message for verification.
    #     """
    #     while True:
    #         if self.mode == "Monitoring":
    #             try:
    #                 message = self.xbee_radio.read_data(1)
    #                 if message is not None:
    #                     source_address = message.remote_device.get_64bit_addr()
    #                     received_data = struct.unpack(">f", message.data[:4])[0]
    #                     current_time = datetime.now().strftime("%m_%d_%Y_%H:%M:%S")
    #                     with open(PATH_GUIDANCE, 'a') as file:#new111
    #                         file.write(f"{current_time} - Message received from {source_address} ({self.drone_id}): {received_data}\n")
    #                         print(f"{current_time} - Message received from {source_address} ({self.drone_id}): {received_data}")
    #                     # self.goto()
    #                     # self.photo(self.server, self.nport, self.user, self.password, self.local_path, current_time)
    #                     self.get_telemetry(self.consumer, self.drone_attitude_x, self.drone_attitude_y, self.drone_attitude_z, self.drone_attitude_w, self.gimbal_attitude_x, self.gimbal_attitude_y, self.gimbal_attitude_z, self.gimbal_attitude_w,self.gps_latitude, self.gps_longitude, self.gps_altitude, self.gps_satellites, self.altitude, current_time)
    #                     # self.addsnap(self.local_path, self.current_time)
    #             except Exception as e:
    #                 if "timeout" not in str(e):
    #                     print(f"Error: {e}")
    #         elif self.mode=="Updating" and self.drone_type=="Coordinator":
    #             received_float_data = self.receive_from_previous_hop()
    #             if received_float_data:
    #                 print("coordinator received float data: {}, {}".format(received_float_data[0], received_float_data[1]))
    #                 # Send data again after receiving
    #                 # float_data1 = 3.14  
    #                 # float_data2 = 2.71  
    #                 self.transmit_to_next_hop(received_float_data[0], received_float_data[1])
    #                 self.token_timer.cancel()
    #                 self.token_timer.start()
    #                 self.counter = self.counter+1
    #                 print("Round completed. Counter value:", self.counter)
    #         else:
    #             received_float_data= self.receive_from_previous_hop()
    #             if received_float_data:
    #                 print("router received float data: {}, {}".format(received_float_data[0], received_float_data[1]))
    #                 time.sleep(1)
    #                 float_data1 = 3.14  
    #                 float_data2 = 2.71 
    #                 self.transmit_to_next_hop(float_data1,float_data2)
    #         time.sleep(1)


    # def print_waiting_message(self):
    #     """
    #     Continuously print a waiting message to indicate the device is ready to receive messages.
    #     """
    #     while True:
    #         print(f"Waiting for messages at {self.drone_id}...")
    #         time.sleep(1)


    # def send_broadcast_messages_randomly(self):
    #     """
    #     Send broadcast messages at random intervals.
    #     """
    #     while True:
    #         if self.mode == "Monitoring":
    #             drone_id_float = float(self.drone_id)
    #             self.transmit_broadcast(drone_id_float)
    #         random_interval = 10
    #         time.sleep(random_interval)

    # def switch_mode_periodically(self):
    #     while True:
    #         time.sleep(300)
    #         self.mode= "Monitoring" if self.mode=="Updating" else "Updating"
    #         print(f"mode switched to {self.mode} for {self.drone_id}")