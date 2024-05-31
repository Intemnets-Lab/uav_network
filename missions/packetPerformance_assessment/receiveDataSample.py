from digi.xbee.devices import XBeeDevice
import time

PORT = "/dev/ttyUSB0" #might have to change based on your system
BAUD_RATE = 9600

def main():
    print(" +-----------------------------------------+")
    print(" | XBee Python Library Receive Data Sample |")
    print(" +-----------------------------------------+\n")

    device = XBeeDevice(PORT, BAUD_RATE)
    message_count = 0
    start_time = None
    expected_packets = 0
    packet_loss = 0
    total_bytes_received = 0

    try:
        device.open()

        def data_receive_callback(xbee_message):
            nonlocal message_count, start_time, expected_packets, packet_loss, total_bytes_received
            if message_count == 0:
                start_time = time.time()
            message_count += 1
            received_packets = int.from_bytes(xbee_message.data[:4], 'big')
            integer_value = int.from_bytes(xbee_message.data[4:], 'big')
            print(f"Message #{message_count}: From {xbee_message.remote_device.get_64bit_addr()} >> {integer_value}")
            total_bytes_received += (len(xbee_message.data) / 8)
            # print("message size:", len(xbee_message.data))
            
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= 300:  #5 minutes
                data_rate = total_bytes_received / elapsed_time
                print(f"Elapsed time: {elapsed_time} seconds. Data rate: {data_rate} packets/second")
                start_time = current_time
                total_bytes_received = 0

        device.add_data_received_callback(data_receive_callback)

        print("Waiting for data...\n")
        input()

    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == '__main__':
    main()
