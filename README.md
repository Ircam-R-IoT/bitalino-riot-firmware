# BITalino R-IoT firmware
firmware of the BITalino R-IoT Hardware v2.0 (USB charging, USB serial port)

v2.1
- Fixed the magnetometers scale to 4 gauss instead of 2 but this doesn't change the orientation filter since normalized values are used there
- Fixed the Madgwick filter implementation as per some online updates and found the the original published filter had a few errors in the calculations
- Filter more stable as a result

v2.044
- Some fixes for compiling correctly with Energia 23. Minor fixes for SFLS api and return values when opening the prefs file

v2.043
- Couple of fixes on the LSM9DS1 reading
- migration do Energia 23 for better compatibility (especial on MacOS, now signed software)
- Discards the use of an external SFLS library, now part of Energia 23


v2.041
- Cosmetics fixes
- Added a serial message for calibration enabled then disabled after timeout is reached

v2.04
- Fixed RGB LED order (swapped green and blue)
- Cosmetics
- Fixed : access to calibration for 6s after booting, even if not wifi connected
- Fixed : LED blinking (white) during MAG calibration

v2.03
- aligned IRCAM and Bitalino R-IoT versions and code
- Sensor axis matching between LSM9DS0 and LSM9DS1 + board layout.
- Safety on calibration: active for a short time after power cycling only (JP LAMBERT).

#### quick start

* Download and install Energia version 23 from [energia.nu/download/](http://energia.nu/download/).
* Note : Version 23 now uses the board manager and hardware support isn't located in the Energia folder anymore.
* Modify the `cc3200.ld` file and change the `HEAP_SIZE` value to `0x00008000` and comment the last line /*PROVIDE(end = _end);*/
  * on Windows this file is located into `C:\Users\<USER NAME>\AppData\Local\Energia15\packages\energia\hardware\cc3200\1.0.3\cores`
  * on Mac OS, go on your libraries folder (might be hidden), then go to `Energia15\packages\energia\hardware\cc3200\1.0.3\cores`
* Additionally, the WiFiUdp.cpp source file must be modified (Energia15\packages\energia\hardware\cc3200\1.0.3\libraries\WiFi), as its function parsePacket() blocks during 10ms. The code has been modified to reduce this timeout to 1ms (change timeout.tv_usec = 1000 instead of 10000)
  to maintain a low latency and a 200Hz update rate of the IMU streaming.
* Get the BITalino Energia library from [github.com/Ircam-R-IoT/bitalino-energia-library](https://github.com/Ircam-R-IoT/bitalino-energia-library) and drop it into `Documents/Energia/libraries`.
* Open the firmware.ino file with Energia and hit the "Verify" button in the upper left corner. If it builds, you're ready to upload it to the R-IoT board.


#### IMU offsets calibration
Power up the module *then* within 6 seconds after power up, press the (right most) general purpose switch, the nearest to the RGB led. Keep it pressed for 3 seconds and the module will enter calibration mode.
For best practice, the R-IoT can be hooked up to a serial terminal, including the one from Energia. We also use CoolTerm and Docklight, the board uses a 115200 baud rate. 
Messages are output on the serial port during the calibration to indicate the calibration stages and progress. The RGB led will also wink and blink with various styles to reflect this.

* Once entering calibration the module will output "ACC+GYRO+MAG Calibration Started" and will start blinking the LED red.
* Place the module steady on a flat surface with the motion sensor facing up (RGB led and switches facing down)
* Once found stead enough, the RGB led will turn white and the board will output the found offsets for the gyro and accelerometer.
* Grab the module and press the (right most) general purpose switch once
* The module will output "*** Proceeding to MAG calibration - Max out all axis **** " and the led will blink white quickly
* Spin the module in all axis, possibly away from strong magnetic sources (speakers for instance)
* Press the GP switch again, the module will output the found magnetometer offsets and will save all offsets in the NV memory
* Reboot the module (reset switch or power cycle) to use the calibrated module

** IMU Offsets are also displayed during the boot and printed on the USB serial port


#### going further

For more information, please visit
[bitalino.com/en/r-iot-kit](http://bitalino.com/en/r-iot-kit) or [ismm.ircam.fr/riot/](http://ismm.ircam.fr/riot/).

#### credits

This project has been developed by the ISMM team at IRCAM-Centre Pompidou within the CoSiMa project (funded by ANR), the MusicBricks project funded by the European Union's Horizon 2020 research and innovation programme, and the RAPID-MIX project (H2020-ICT-2014-1 Project ID 644862).
