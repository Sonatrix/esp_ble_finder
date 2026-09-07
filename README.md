# BLE Finder

[findphone](https://github.com/ben-z/findphone) for the
[Waveshare ESP32-S3-Touch-AMOLED-1.8](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8).

Starts scanning by itself. Dark dotted RSSI and a line that fills as you close in, yellow tip on the last dot.

## Demo

<video src="demo/demo.mp4" width="368" controls playsinline></video>

To hunt by name, edit `main/target.h` and reflash:

```c
#define TARGET_NAME "iPhone"
```

## Mute

Clicks from the speaker speed up as RSSI rises. Tap **Mute** at the bottom of the screen to silence them. The button turns yellow and reads **Sound** — tap it again to turn clicks back on. The hunt keeps running while muted.

## Flash

Find the serial port first. `XXXX` is not a path — the number changes each time you plug the board in:

```sh
ls /dev/cu.usbmodem*
```

Then:

```sh
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem2101 flash
```

Use the name `ls` printed, not `usbmodemXXXX`. If nothing appears, reconnect the data cable and try BOOT + RESET.