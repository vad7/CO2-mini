# CO2-mini
Sends the CO2 value received from the Senseair S8 sensor (PWM out) using nRF24l01 mini
<br>
<br>Microcontroller: AVR Attiny44A  
Radio: nRF24l01+ mini SMD  
CO2 sensor: Senseair S8 with PWM out (0..2000 PPM)  
  
Setup with Flipper Zero app "nRF24 Batch" within 15 seconds after power on.  
Default nRF24l01 settings:  
Setup RX address: C8C8CF
Fans over air: 1
Fan 1 address: C8C8C1
Channel: 121  
Data rate: 1 Mbps  
CRC: 2 byte  
Send CO2 value period: 30 sec  
Reset settings - press KEY more than 10 sec after power on.  

<br>
<img src="https://raw.githubusercontent.com/vad7/CO2-mini/master/3D.png">
<br>
<br>
<img src="https://raw.githubusercontent.com/vad7/CO2-mini/master/Schema.png">
