# esp32_usb_soft_host
esp32 USB-LS pure software host thru general IO pins. Up to 4 HID devices simultaneously. 

board ~$3 :https://www.aliexpress.com/premium/LOLIN32.html or any of https://www.aliexpress.com/premium/ESP32.html

usb connectors(for example): https://www.aliexpress.com/item/1005002027124387.html


//set right esp32 env for me it:

export IDF_PATH=$HOME/esp/esp-idf

source $HOME/esp/esp-idf/export.sh


//connect the board, build & flash

git clone  https://github.com/sdima1357/esp32_usb_soft_host

cd esp32_usb_soft_host/usb_test/

make flash monitor

