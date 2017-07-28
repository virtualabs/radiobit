"""
This script simply loops on advertising channels and
sniffs BLE connection requests.
"""
import radio
from microbit import *

radio.on()
radio.ble()
timeout = 50
last = running_time()
chan = 0

while True:
    if (running_time() - last) >= timeout:
        chan = (chan+1)%3
        radio.config(channel=(chan+37))
        last = running_time()
    p = radio.receive()
    if p is not None and p[5]&0x0F == 5 and p[6]==0x22:
        #print(' '.join(['%02x'%c for c in p]))
        payload = p[8:]
        pl =  ' '.join(['%02x' % c for c in payload])
        print(pl)
        # parse connect_req
        inita = ':'.join(['%02x'%c for c in payload[:6]])
        adva = ':'.join(['%02x'%c for c in payload[6:12]])
        aa = payload[12] | payload[13]<<8 | payload[14]<<16 |payload[15]<<24
        crcinit = (payload[16])|(payload[17]<<8)|(payload[18]<<16)
        hop_increment = (payload[33]&0x1f)
        hop_interval = payload[22] | payload[23]<<8

        print('[%08x] %s -> %s (CRCInit: %06x, hopInc: %d, hopInter: %d)' % (aa, inita, adva, crcinit, hop_increment, hop_interval))
