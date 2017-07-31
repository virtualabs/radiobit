How to use Radiobit's radio module ?
====================================

Radiobit's *radio* module exposes multiple methods allowing sniffing, receiving and injecting
ESB/SB/BLE packets.

Sniffing ESB/SB packets
-----------------------

If you want to sniff Enhanced ShockBurst packets, you need to enable the radio
and activate the sniffing mode, as shown below:

``` python
import radio

radio.on()
radio.sniff_on()
```

You may then want to loop over each sniffed packets:

``` python
while True:
    pkt = radio.sniff()
    if pkt is not None:
        # do something with the pkt
```

The sniffing algorithm tries by default to find ESB packets rather than SB packets, but you may use raw sniffing to get them:

``` python
import radio

radio.on()
radio.config(raw=1, length=40, channel=80)
radio.sniff_on()

while True:
    pkt = radio.sniff()
    if pkt is not None:
        # pkt should be either ESB or SB, as no CRC check is made
        # you have to sort it out and find your SB packets :S
```

This raw sniffing feature may also be used to investigate unknown protocols, as it operates as a simple GFSK demodulator using a specific data rate.


Receiving ESB packets
---------------------

In order to receive ESB packets, you need to tune the transceiver to a specific channel and set
a 5-byte address as shown below:

``` python
import radio

radio.on()

# Listen on channel 80 for address 0x1122334455
radio.config(channel=80, address=0x11223344, group=0x55)
radio.esb()
```

The received packets can then be processed:

``` python
pkt = radio.receive_bytes()
if pkt is not None:
    # process packet
```

Sending ESB packets
---------------------

You may also want to send packets:

``` python
radio.send_bytes(b'Trolololo')
```


Receiving SB packets
--------------------

In order to receive Legacy SB packets, you need to tune the transceiver to a specific channel and set
a 5-byte address as shown below:

``` python
import radio

radio.on()

# Listen on channel 80 for address 0x1122334455, packet size of 10 bytes
radio.config(channel=80, address=0x11223344, group=0x55, length=10)
radio.sb()
```

Note the packet size is mandatory when using ShockBurst (SB) procotol, as the packet format
contains no length field. By default, the length used is 32.

The received packets can then be processed the usual way, by calling *radio.receive_bytes()*:

``` python
pkt = radio.receive_bytes()
if pkt is not None:
    # process packet
```
