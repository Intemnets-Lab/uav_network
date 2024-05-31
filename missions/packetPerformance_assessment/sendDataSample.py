from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
from digi.xbee.exception import TransmitException

xbee = XBeeDevice("/dev/ttyUSB1", 9600) #might have to change based on your system
xbee.set_sync_ops_timeout(10000)
integer_value = 3
packet_counter = 0
integer_length = len(str(integer_value))
xbee.open()

remote = RemoteXBeeDevice(xbee, XBee64BitAddress.from_hex_string("0013A20040DD2F5F"))

while True:
    data_send = packet_counter.to_bytes(4, 'big') + integer_value.to_bytes(4, 'big')
    try:
        xbee.send_data(remote, data_send)
        packet_counter += 1
    except TransmitException as e:
        print("TransmitException occurred: ", e)