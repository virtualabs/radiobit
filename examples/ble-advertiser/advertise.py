"""
BLE advertising example

This example uses a little hack to send BLE scan responses
in time: it sends it straight forward even if not required.
"""
from microbit import *
import radio

adv_pkt = bytes([0x40, 0x42, 0xd8, 0x2a, 0x41, 0x32, 0x65,0x02, 0x01, 0x1a, 0x09, 0x09])+b'DEFCON25'
scan_rsp = bytes([0x44, 0x42, 0xd8, 0x2a, 0x41, 0x32, 0x65, 0x03, 0xff,0x12,0x13])
radio.on()
radio.config(channel=38)
radio.ble()
while True:
    for  i in range(37,40):
        radio.config(channel=i)
        radio.send(adv_pkt)
        radio.send(scan_rsp)
        sleep(50)
