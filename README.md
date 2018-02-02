# PicoFlash


## Synopsis

**PicoFlash** is a command-line tool allowing to update the firmware of a **PicoSmartTAG** accessory.


## Build instructions

To build PicoFlash, a Unix environment with the GNU Make tool available is required (Linux, Cygwin or MacOS).

Then, in the *PicoFlash* folder, you just need to type:
```
$ make
```

On Linux, you may also need to intall the proper Udev rules in order to grant any user access to the PicoSmartTAG dock:
```
$ sudo cp ~/PicoWAN-SDK/utils/PicoFlash/60-picoflash.rules /etc/udev/rules.d/
$ sudo udevadm trigger
```


## Usage

```
$ ./PicoFlash <tty_device> <binary_firmware>
```

For instance:
```
$ ./PicoFlash /dev/ttyUSB0 my_application.bin
```

Please note that in the Cygwin environment, **COM1, COM2, ...** corresponds respectively to **/dev/ttyS0, /dev/ttyS1, ...**


## History

v1.0:
* Initial version of PicoFlash

__  
Copyright (c) 2017 Archos S.A.
