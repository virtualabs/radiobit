"""
Micro:bit ESB sniffer optimized Middleware

This piece of software sniffs ESB packets and provides
an UART interface.
"""
from microbit import *
import radio

class Globals:
    # GLOBALS
    RATE = radio.RATE_2MBIT
    CHANNEL = 1
    TARGET = None

    # 0 - scan, 1 - follow MAC, 2 - listen channel
    MODE = 0

def readhex(n):
    """
    Read n bytes from hex
    """
    o = b''
    while len(o) < 2*n:
        p = uart.read(2*n - len(o))
        if p is not None:
            o += p
    return h2b(o)

def h2b(h):
    """
    Quick convert hex string to bytes
    """
    nb = int(len(h)/2)
    o = bytearray()
    for i in range(nb):
        o.append(int(h[2*i:2*i+2],16))
    return o

def reset(rate=radio.RATE_2MBIT, raw=0):
    """
    Reset UART & sniffer
    """
    Globals.RATE = rate
    Globals.MODE = 0
    Globals.TARGET = None
    Globals.CHANNEL = 1

    uart.init(baudrate=115200)
    radio.off()
    radio.on()
    radio.config(data_rate=rate, channel=Globals.CHANNEL, raw=raw)
    radio.sniff_on()
    radio.config(data_rate=rate, channel=Globals.CHANNEL, raw=raw)
    radio.sniff_on()

def set_channel(channel):
    """
    Set ESB channel (1-100)
    """
    Globals.CHANNEL = channel
    radio.config(channel = Globals.CHANNEL)

def follow_target(target):
    """
    Follow target
    """

    # convert target bytes to address/group values
    addr  = target[0]<<24
    addr |= target[1]<<16
    addr |= target[2]<<8
    addr |= target[3]
    group = target[4]

    # config radio
    radio.config(
        data_rate=Globals.RATE,
        address=addr,
        group=group,
        channel=Globals.CHANNEL
    )
    radio.esb()
    # if mode is 2, keep with this mode (no channel loop)
    if Globals.MODE == 0:
        Globals.MODE = 1

# main routine
reset()
timeout = 200
ping_timeout = 1000
last = running_time()
while True:
    # check if we got a command on serial port
    if uart.any():

        # First hex byte => size
        header = readhex(1)

        # compute size
        size = header[0]

        if size > 0:
            # read and decode
            payload = readhex(size)

            # process command
            cmd = payload[0]
            if cmd == 0x72 and size >=3:
                rate = payload[1]
                raw = payload[2]
                # reset cmd
                reset(raw=raw,rate=rate)
                uart.write(b'r')
            elif cmd == 0x63 and size >= 2:
                # channel is placed in the next byte
                channel = payload[1]
                set_channel(channel)
                uart.write(b'c')
            elif cmd == 0x66 and size >= 6:
                # address selection
                follow_target(payload[1:6])
                uart.write(b'f')
            elif cmd == 0x70:
                # read packet buffer
                pkt = radio.receive()
                if pkt is not None:
                    if Globals.MODE == 0:
                        uart.write(b'p' + chr(len(pkt)+1) + chr(Globals.CHANNEL) + pkt)
                    else:
                        uart.write(b'p' + chr(len(pkt)+1) + chr(Globals.CHANNEL) + pkt)
                else:
                    uart.write(b'p'+chr(0))
            elif cmd == 0x74:
                found = False
                for i  in range(1,100):
                    radio.config(channel=i)
                    if radio.ping():
                        uart.write(b't'+chr(i))
                        found = True
                        break
                if not found:
                    uart.write(b't'+chr(101))
            elif cmd == 0x73:
                data = payload[1:]
                radio.send_bytes(data)
                uart.write(b's')
            elif cmd == 0x6D and size >= 2:
                mode = payload[1]
                if mode == 0:
                    reset()
                elif mode == 1:
                    Globals.MODE = 1 # disable scan
                else:
                    Globals.MODE = 2
                uart.write(b'm')
            elif cmd == 0x62 and size >= 2:
                rate = payload[1]
                if rate>=0 and rate<=2:
                    Globals.RATE = rate
                uart.write(b'b')

    # loop on channels if scan mode
    if Globals.MODE == 0:
        now = running_time()
        if (now - last) > timeout:
            last = now
            c = Globals.CHANNEL + 1
            if c > 100:
                c = 1
            set_channel(c)
    elif Globals.MODE == 1:
        now = running_time()
        if (now - last) > ping_timeout:
            for i in range(1,100):
                radio.config(channel=i)
                if radio.ping():
                    Globals.CHANNEL = i
                    last = now
                    break
