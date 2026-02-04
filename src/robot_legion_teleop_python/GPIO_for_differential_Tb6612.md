# GPIO connections from L298N motor controller to Raspberry Pi 4 for DIFFERENTIAL DRIVE Chassis  

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
<br>
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
