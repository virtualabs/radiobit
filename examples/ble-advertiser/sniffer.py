from microbit import *
import radio

radio.on()
radio.config(channel=38)
radio.ble()

while True:
    pkt = radio.receive_bytes()
    if pkt is not None:
        if len(pkt) > 13:
            addr = '%02x:%02x:%02x:%02x:%02x:%02x' % (
                pkt[13], pkt[12], pkt[11], pkt[10], pkt[9], pkt[8]
            )
            advinfo = ' '.join(['%02x'%c for c in pkt[14:]])
            print('+ %s > %s' % (addr, advinfo))
            del advinfo
            del addr
            del pkt
