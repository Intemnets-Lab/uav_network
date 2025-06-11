from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
import struct

# initialization
port = "/dev/ttyUSB0"
baud_rate = 9600
xbee_radio = XBeeDevice(port, baud_rate)
xbee_radio.open()

# receive three floating numbers
while True:
    try:
        message = xbee_radio.read_data(1)
        if message is not None:
            source_address = message.remote_device.get_64bit_addr()
            received_data = struct.unpack(">fff", message.data[:12])
            data1 = received_data[0]
            data2 = received_data[1]
            data3 = received_data[2]
            print(data1,data2, data3)
    except Exception as e:
        print(f"Error: {e}")