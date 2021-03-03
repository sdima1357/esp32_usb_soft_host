# esp32_usb_soft_host
esp32 low speed USB soft host up to 4 HID devices simultaneously

board :https://www.aliexpress.com/premium/LOLIN32.html

//set right esp32 env for me it:

export IDF_PATH=$HOME/esp/esp-idf

source esp-idf/export.sh


//connect the board, build & flash

git clone  https://github.com/sdima1357/esp32_usb_soft_host

cd esp32_usb_soft_host/usb_test/

make flash monitor

