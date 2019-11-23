SwitchDoc Labs LoRa SolarMAX Source code for the Transmitter (Tx) in SolarMAX

Designed for the Mini Pro LP Arduino Board from SwitchDoc Labs<BR>
(select Mini Pro 16MHz/5V in Arduino IDE)

November 22, 2019

SOFTWAREVERSION 003 

This uses Protocol_ID of 8 and 10 (versus 3 for WXLink Weather Sensor Data)

Selects Protocol 8 if device has LiPo battery (SolarMAX LiPo)<BR>
Selects Protocol 10 if device has Lead Acid battery (SolarMAX Lead Acid)


This software requires you to install RH_RF95 as an Arduino Library.  You can download it here:

http://www.airspayce.com/mikem/arduino/RadioHead/index.html

Copy these libraries into the Arduino IDE Library:

Jeelib.zip
Time.zip

Then unzip these files in the libraries directory to install.

