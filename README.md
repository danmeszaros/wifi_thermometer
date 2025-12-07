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

* device operates on 3.3V. you need to connect li-po/li-ion battery + to the 5V pin of the D1 board.
    it will then use onboard 3.3V regulator, which is very efficient
* sda/scl/gnd/vcc into the bmp280 module
* switch between D7 and GND for entering the wifi setup
* D0 into RST for waking up from deepsleep
* + battery pole into A0 via 1M resistor for battery voltage reading (bypassing the regulator)

## notes 
* esp32 current during deep sleep is said to be 10 µA, bmp280 board adds a little bit. common optimisation
   is to connect bmp280 vcc into esp digital output and power it only when the esp is not sleeping
* no htts
* device has no meaning of time, does no buffering when the network is down
