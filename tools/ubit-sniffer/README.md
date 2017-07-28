Micro:Bit ESB/SB/BLE/raw sniffer
================================

This tool comes into two parts:

* the first one is a middleware you need to put in your Micro:Bit along with the Radiobit firmware
* the second is a classic Python2.x (shame on me) CLI you may use to sniff

This tool supports different data rates:

* 2 MBit/s
* 1 MBit/s
* 250 Kbit/s
* Bluetooth Low Energy specific data_rate

Be careful while sniffing Bluetooth Low Energy communications, the provided channel is understood as a BLE channel number !

How to flash your Micro:Bit with the right middleware
-----------------------------------------------------

Simply use *uflash*:

```
$ uflash -r precompiled/radiobit.hex tools/ubit-sniffer/middleware/ubit-sniffer-mw.py
```
