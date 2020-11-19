# BL0940

BL0940 communication library for Arduino and ESP8266/ESP32

This IC is produced by [Shanghai Belling Co., Ltd][1] and can be found in some chinese products like [BlitzWolf® BW-SHP10 3680W][2]
The BL0940 is a high accuracy, single-phase electrical energy measurement IC. It measures line voltage and current and calculates active energy, rms voltage and current.

Some BL0940 Features

* Two independent Sigma-Delta ADCs
* Voltage Zero-Crossing Logic Output
* Uart/SPI Communication Interface
* On-chip voltage reference of 1.218V
* On-chip oscillator circuit
* Low Power Consumption 10mW (Typical)

## Features

The main features of the BL0940 library are:

* Full access to IC register
* Easy IC configuration 
* Full measurements reading
* Internal Apparent/Reactive power calculation

## Usage

TODO some examples. 
For code test enthusiasts - Do not forget to apply any internal register changes :-)

### Notes

The main reason to write this library is to use [Xose Pérez][3] [ESPurna Firmware][4] with [BlitzWolf® BW-SHP10 3680W][2], so all measurement calibration is for my own device. But I'm sure there couldn't be a significant differences in voltage divider and shunt resistor value. In future version, I'm going to add some code to allow user manipulation of this two parameters.

And picture taken from [Espurna][4] test environment
![Pure resistive load](/pics/shp10-status.png)


[1]:http://www.belling.com.cn/en/product_info.html?id=211
[2]:https://www.blitzwolf.com/3680W-16A-WIFI-Smart-Socket-p-453.html
[3]:https://github.com/xoseperez
[4]:https://github.com/xoseperez/espurna/tree/master

## License

Copyright (c) 2020 Rossen Dobrinov

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
