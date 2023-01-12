# ESP32 Bluetooth Audio Receiver
---

This little script for the ESP32 WROOM 32 Board uses the A2DP Library of [Phil Schatzmann](https://github.com/pschatzmann) to realize all its functions.
I used said ESP32 in combination with a 32bit external DAC (I know, 32bit is a bit overkill, but i wanted to try anyway). The DAC IC is an PCM5 102A from Texas Instruments, which uses a I²S to communicate with the ESP32. Additionally there is a confirmation/wakeup button mounted to the board and two status LED.

## Setup / Arduino IDE
I used Arduino IDE to code this script. The setup is quite easy. There are dozen tutorials online, but for reference i used this one: [Arduino Setup](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)