Radiobit, a BBC Micro:Bit RF firmware
=====================================

Introduction
------------

Radiobit is composed of a dedicated Micropython-based firmware and a set of tools allowing security researchers to:

* sniff, receive and send data over Nordic's Enhanced ShockBurst protocol (ESB)
* sniff, receive and send data over Nordic' ShockBurst protocol (SB)
* sniff, receive and send data over Bluetooth Smart Link Layer
* sniff raw 2.4GHz GFSK demodulated data

Using Radiobit tools and examples
---------------------------------

Radiobit provides the following tools:

* **ubit-sniffer**: a ESB/SB/BLE/raw sniffer able to collect data and display them.
* **cheerson-cx10**: the firmware corresponding to the hack described in my DefCon25 presentation, used to hijack a Cheerson CX-10 quadcopter in flight.
* **wireless-keylogger**: Microsoft wireless keyboard sniffer firmware, as shown at DefCon25

See the corresponding README.md files for each tool of the *tools* directory.

Radiobit also provides many examples of what can be achieved with its firmware:

* A BLE advertiser and its associated sniffer
* A BLE connection request packet sniffer
* An ESB basic sniffer running on the Micro:Bit

**A precompiled version of the Radiobit firmware is provided in the *precompiled* directory, as a convenience.**

If you want to try Radiobit's custom radio Python module, read the [specific documentation](doc/README.md).

How to program the Micro:Bit with some Python code
--------------------------------------------------

Use *uflash* to program your Micro:Bit:

1. Plug your Micro:Bit to your computer with a USB cable, it will be recognised as a mass storage device
2. Mount it
3. Use uflash to merge the firmware with your main Python3 program and flash the device

```
$ uflash -r precompiled/radiobit.hex yourprogramhere.py
```

*Uflash* will program your Micro:Bit and reset it once it's done !

Recompiling the firmware
------------------------

If you want to modify the Radiobit firmware, you must follow the procedure below. This procedure has been tested on Debian, but should work on Ubuntu as well.

### How to setup the environment

First, install all the required packages (Debian):

```
$ sudo apt-get install cmake ninja-build gcc-arm-none-eabi srecord libssl-dev
```

Create a virtual environment with *virtualenv* for Python 3.x:

```
$ virtualenv venv
```

This will create a Python3.x virtual environment in a dedicated folder named *venv*. You then need
to activate this environment in order to install all the required tools with pip3.

```
$ source venv/bin/activate
```

Use pip to install **yotta** in our newly created virtual environment:

```
(venv)$ pip install yotta
```

Finally, use **yotta** to setup the build environment:

```
(venv)$ cd micropython
(venv)$ yt target bbc-microbit-classic-gcc-nosd
(venv)$ yt up
```

### How to build the firmware

You can now uild the modified micropython firmware. Make sure you are in the *micropython* directory:

```
(venv)$ yt build
```

This may produce a lot of warnings (who said python developers produce clean code ?) but at last a valid binary (in Ihex format).

Your compiled firmware should be located in the *build/bbc-microbit-classic-gcc-nosd/source* directory, named *microbit-micropython.hex*.

### How to flash a Micro:Bit with this new firmware

Micropython should not be used alone (as a REPL), but combined with a Python script merged into the firmware. This is a usual way to *program* a Micro:Bit using CLI rather than online services.

First, you need to install **uflash** in your virtual environment:

```
(venv)$ pip install uflash
```

You then can use **uflash** to flash your Micro:Bit using the following command:

```
(venv)$ uflash -r build/bbc-microbit-classic-gcc-nosd/source/microbit-micropython.hex ../examples/helloworld/helloworld.py
```

Obviously, you must have your Micro:Bit connected to your host machine when launching the previous command, or *uflash* will complain.
