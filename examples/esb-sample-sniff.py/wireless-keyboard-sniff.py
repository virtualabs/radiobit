"""
Quick'n'dirty ESB sniffer listening on channel 74 at 2MBit/s
"""
import radio

radio.on()
radio.config(data_rate=radio.RATE_2MBIT, channel=74)
radio.sniff_on()

while True:
    pkt = radio.sniff()
    if pkt is not None:
        addr = ':'.join(['%2x'%c for c in pkt[:5]])
        payload = ''.join(['%02x '%c for c in pkt[5:]])
        print('%s > %s' % (addr, payload))
