# Packet Rate Evaluation & Packet Loss Measurement

Utilizing the [XBee Python library](https://xbplib.readthedocs.io/en/latest/), two scripts were developed to test the packet rate and the correlation between packet loss and distance.

sendDataSample.py transmits two four byte values (total 8 bytes of user data sent):

#1 packet_counter: Can be used on the receiver side if nessary for debugging amount of data packets transmitted and received.

#2 integer_value: Can be any integer value one would want to transmit.

receiveDataSample.py Parses the data received from sendDataSample.py and outputs the internal message counter, transmiter's MAC address, integer_value received, and lastly every 5 minutes prints out the data rate (packets/s).

## Python virtual enviroment setup & XBee Python library installation
Create Python virtual enviroment
```bash
python -m venv xbee_env
```
Activate Python virtual enviroment and install XBee Python library
```bash
. ./test_env/bin/activate && pip install digi-xbee
```

After completing the above steps running the receiveDataSample.py and the sendDataSample.py should operational
```bash
python3 receiveDataSample.py
```
```bash
python3 sendDataSample.py
```


Note: Some systems require the use of sudo for accessing ones USB ports on their system.