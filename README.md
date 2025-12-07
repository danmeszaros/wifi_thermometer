# wireless battery powered wifi thermometer

## operation

* after reset, the device reads temperature/humidity/pressure from the bmp280 module
* these values are reported over wifi via http protocol to the server
* battery voltage is also reported
* device will set the alarm and enter deep sleep mode

## setup

since the device has no UI, wifi network parameters are set as follows:
* hold the custom setup switch while pressing the reset button
* device boots in setup mode
* it will run in WIFI access point mode allowing you to connect to the device via mobile phone
* device will search for available wifi networks
* run browser in you mobile phone and you will receive the form in which you can setup wifi for the thermometer
* submit the form and after few seconds thermometer will reboot and switch to normal operation mode

# documentation

## wiring

* device operates on 3.3V. you need to connect li-po/li-ion battery + to the 5V pin of the D1 board. it will then use onbaord 3.3V regulator, which is very efficient
* sda/scl/gnd/vc into the bmp280 module
* 
