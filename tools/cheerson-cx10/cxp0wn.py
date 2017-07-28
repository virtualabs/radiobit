from microbit import *
import radio

def readhex(n):
    c = uart.read(2)
    if c is not None and len(c) == 2:
        return bytes([int(c,16)])
    return None

uart.init(baudrate=115200)

radio.on()
radio.cx()
radio.config(channel=2)

found = False
txid = None
aid = None
channel = 3
timeout = 24
last=running_time()
while not found:
    now = running_time()
    if (now - last)>24:
        channel += 1
        if channel > 80:
            channel = 3
        last = now
        radio.config(channel=channel)
    pkt = radio.receive()
    if pkt is not None:
        if pkt[0]==0x55:
            # check if current channel matches txid
            txid = list(pkt[1:5])
            channels = [
                (txid[0]&0x0f)+0x3,
                (txid[0]>>4)+0x16,
                (txid[1]&0x0f)+0x2d,
                (txid[1]>>4)+0x40
            ]
            if channel in channels:
                found = True
                aid = list(pkt[5:9])

channels = [
    (txid[0]&0x0f)+0x3,
    (txid[0]>>4)+0x16,
    (txid[1]&0x0f)+0x2d,
    (txid[1]>>4)+0x40
]

asked  = False
last_asked = running_time()
b = b''
p = b''

# reinit radio
counter = 0
radio.config(channel=channels[counter])
radio.cx()

t = 0x3c9
l = running_time()

t,r,e,a = 0,0,0,0
ctl = 0

display.show(Image.HAPPY)

# sync
pkt = None
while pkt is None:
    pkt = radio.receive()
next_at = running_time()+6

while True:
    now = running_time()
    if now >= next_at:
        next_at = next_at + 6
        counter = (counter + 1)%4
        radio.config(channel=channels[counter])
        radio.send(p)
        ctl = 0

    if ctl == 0:
        t = pin0.read_analog()
        t = int(2031 * (t/1023)) + 0x386
        r = pin4.read_analog()
        r = int(3000 * (r/1034))
        ctl += 1
    elif ctl == 1:
        e = pin10.read_analog()
        e = int(3000 * (e/1023))
        a = pin1.read_analog()
        a = int(3000 * (a/1023))
        p = bytes([0x55] + txid + aid + [a&0xff, a>>8, e&0xff, e>>8, t&0xff, t>>8, r&0xff, r>>8, 0x00, 0x00])
        ctl = 2
