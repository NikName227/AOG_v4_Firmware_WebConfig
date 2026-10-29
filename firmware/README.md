# Ready-to-flash firmware

`AIO_v4_v1.0.24.hex` — AIO v4 WebConfig firmware v1.0.24 for the Teensy 4.1
(built with Teensyduino core 1.57.3, `FW_VERSION "v1.0.24"`, shown in the web GUI).

Upload without building:

1. Install the [Teensy Loader](https://www.pjrc.com/teensy/loader.html) (it comes with Teensyduino).
2. Connect the Teensy 4.1 over USB.
3. In Teensy Loader: *File → Open HEX File* → `AIO_v4_v1.0.24.hex`, then *Operate → Program* (or press the button on the Teensy).

Command line (`teensy_loader_cli`): `teensy_loader_cli --mcu=TEENSY41 -w -v AIO_v4_v1.0.24.hex`

Later versions can be uploaded from the web page: Config tab → *Firmware update* (PIN, default `1234`).

After the first boot of a new layout the settings may be reset to defaults (EEPROM ident) — set up
everything in the web GUI again (see the main README). Source: `AIO_v4_Firmware_WebConfig/`.
