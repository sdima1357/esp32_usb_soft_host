# esp32_usb_soft_host
esp32 USB-LS pure software host thru general IO pins. Up to 4 HID devices simultaneously. 

Code is compatible with ESP-IDF version < 5.0, latest: https://github.com/espressif/esp-idf/releases/tag/v4.4.8

![image1](https://github.com/sdima1357/esp32_usb_soft_host/blob/main/images/IMG_20210303_184755_1.jpg?raw=true)

board ~$3 :https://www.aliexpress.com/premium/LOLIN32.html or any of https://www.aliexpress.com/premium/ESP32.html

usb connectors(for example): https://www.aliexpress.com/item/1005002027124387.html


//set right esp32 env for me it:

>export IDF_PATH=$HOME/esp/esp-idf

>source $HOME/esp/esp-idf/export.sh


//connect the board, build & flash

>git clone  https://github.com/sdima1357/esp32_usb_soft_host

>cd esp32_usb_soft_host/usb_test/

> idf.py set-target esp32

> idf.py menuconfig

**Please set in Menuconfig->compiler options -> optimization level> O2 ( must be in O2 options. "idf.py set-target esp32" resets it to Og ,I don't know why)**

> idf.py flash monitor


Test run with 3 mouses, $3 CY7C68013A logic analyser  and amazing program https://sigrok.org/wiki/PulseView :

![image2](https://github.com/sdima1357/esp32_usb_soft_host/blob/main/images/PulseView.jpg?raw=true)

>Tue Aug 31 15:50:48 IDT 2021

Add esp32c3 support. 

>idf.py set-target esp32c3 

>idf.py menuconfig

set in: 

Menuconfig->compiler options -> optimization level> O2

**only for esp32c3: Component config-> ESP System Setting -> Memory protection-> Disable.**

> idf.py flash monitor



**Warning(disclaimer):
Sorry, but target of this project right now - only research & proof of feasibility. I can't debug hardware which i don't have. Some hardware(devices) may not work, from different reasons. In the case of problem i suggest to check wires &  use $3 CY7C68013A logic analyser. Sometimes i can help.**
