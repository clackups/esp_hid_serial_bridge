ESP32-S3 BLE-HID serial bridge
==============================

This is a firmare project for ESP32-S3 that implements a BLE keyboard
and mouse device that takes its inpiut from a USB-Serial interface.

The purpose of this device is to allow building a smart input
appliance that will help compensating various disabilities for users
who can't use the standard keyboard or mouse. A dedicated small
computer, such as a Raspberry Pi, could be programmed to take the user
input, transform it with the means of language dictionaries, LLM or
some other tools, and generate the HID input for the user's desktop.

Hardware requirements
---------------------

Any ESP32-S3 chip with USB interface and Bluetooth antenna would be
suitable.

For example, a LILYGO T-Dongle S3 would make a compact and convenient
device (the screen would not be utilized in this case).


Building the firmware
---------------------

Set up the ESP-IDF version 5.5.1 and follow the standard
instructions. The default sesttoingss would be suitable for work.

Control interface
-----------------

On a Linux host, the device would be visible as /dev/ttyACM0 (or a
different number if other ACM USB Serial adapters are
present). Picocom can be utilized for quick tests.

The device automatically accepts secure BLE 4.1 pairing requests. It
allows only one active connection at a time, but multiple hosts can be
paired with it.

The control commands consist of ASCII strings of fixed length,
starting with a capital letter. Other characters are ignored, so you
can utilize spaces or newlines as separators for convenience.

Some commands require several hexadecimal byte values. The hexadecimal
inputs can use both lower- and uppercase letters.

Commands:

* `S`: display connection status. The response is either
  `STATUS:NOTCONNECTED` or `STATUS:CONNECTED:xxxxxxxxxxxx` (indicating
  the BLE address of the connected host), followed by newline
  (`\r\n`).

* `Z`: delete all Bluetooth pairings. Keep in mind that some OSes may
  keep retrying and failoing to connect in a loop after this
  operation, until you "forget" the device in the host OS.

* `Kxxxxxx`: send a Keryboard HID report. The first byte indicates the
  special keys bitmask, the second byte contains the number of key
  codes, and the following bytes contain up to 6 key codes. See the
  [USB Human Interface
  Devices](https://wiki.osdev.org/USB_Human_Interface_Devices) article
  for more details. Each keypress should be followed by a key release
  report (`K0000`).

* `Mxxxxxxxxxx`: a 5-byte mouse HID report. The first 3 bytes indicate
  the buttons status and X and Y movement. The fourth byte specifies a
  vertical scroll move. The fifth byte is reserved for horizontal
  scroll wheel (currently not implemented).


The following example prints "Hello World!" on the host, followed by
return key:

```
K02010bK0000
K000108K0000
K00010fK0000
K00010fK0000
K000112K0000
K00012cK0000
K02011aK0000
K000112K0000
K000115K0000
K00010fK0000
K000107K0000
K02011eK0000
K000128K0000
```

While sending the keyboard commands, you need to make sure not to send
them too fast, otherwise some input could be lost because of Bluetooth
congestion.


## Copyright and license

This work is licensed under the GNU GENERAL PUBLIC LICENSE Version 3.

Copyright (c) 2025 clackups@gmail.com

Fediverse: [@clackups@social.noleron.com](https://social.noleron.com/@clackups)
