# GPIO connections from Raspberry Pi 4 to DUAL Tb6612fng motor controllers to motors for a TANK TRACK DIFFERENTIAL DRIVE Chassis  

### TB6612 #1 — LEFT SIDE MOTORS  

**Left Front Motor** (Channel A)  

Motor wires  

Motor + to A01  

Motor − to A02  

**GPIO connections**  

PWMA to GPIO 12 (PWM)  

AIN1 to GPIO 5  

AIN2 to GPIO 6  

**Left Rear Motor** (Channel B)  

Motor wires  

Motor + to B01  

Motor − to B02  

**GPIO connections**  

PWMB to GPIO 13 (PWM)  

BIN1 to GPIO 16  

BIN2 to GPIO 20  

**Standby (board enable)**  

STBY to GPIO 21   
(or tie to 3.3V if you never want software disable)  

<br>

### TB6612 #2 — RIGHT SIDE MOTORS  

**Right Front Motor** (Channel A)  

Motor wires  

Motor + to A01  

Motor − to A02  

GPIO connections  

PWMA to GPIO 18 (PWM)  

AIN1 to GPIO 23  

AIN2 to GPIO 24  

**Right Rear Motor** (Channel B)  

Motor wires  

Motor + to B01  

Motor − to B02  

GPIO connections  

PWMB to GPIO 19 (PWM)  

BIN1 to GPIO 25  

BIN2 to GPIO 8  

**Standby (board enable)**  

STBY to GPIO 7  
(you may also tie both STBY pins together and drive them from ONE GPIO if desired)  

<br>  

### Logic power  

Pi 3.3V to VCC on both TB6612 boards  

Pi GND to GND on both boards  

**(3.3V logic is ideal with Pi GPIO)**

<br>  

### Motor power  

If you’re using your 2×18650 (2S) pack:  

Battery + to VM on both boards  

Battery – to GND on both boards  

<br>  

### Common ground (critical)  

All grounds must connect together:  

**Pi GND to TB6612 GND to Battery – (negative)**  

<br>  

### STBY (enable)  

For simplest wiring:  

On TB6612 #1: STBY to VCC (same board)  

On TB6612 #2: STBY to VCC (same board)  
