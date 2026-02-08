# GPIO connections from Raspberry Pi 4 to DUAL Tb6612fng motor controllers for a MECANUM Chassis  

### TB6612 #1 — FRONT MOTORS  

**Front-Left Motor (Channel A)**  

**Motor wires**  

Motor + to A01  

Motor − to A02  

**GPIO connections**  

PWMA to GPIO 12 (PWM0)  

AIN1 to GPIO 5  

AIN2 to GPIO 6  

**Front-Right Motor (Channel B)**  

**Motor wires**  

Motor + to B01  

Motor − to B02  

**GPIO connections**  

PWMB to GPIO 13 (PWM1)  

BIN1 to GPIO 16  

BIN2 to GPIO 19  

**Standby (board enable)**  

STBY to VCC (same motor driver)  

<br>  
<br>  

### TB6612 #2 — REAR MOTORS  

**Rear-Left Motor (Channel A)**  

**Motor wires**  

Motor + to A01  

Motor − to A02  

**GPIO connections**  

PWMA to GPIO 18  

AIN1 to GPIO 20  

AIN2 to GPIO 21  

**Rear-Right Motor (Channel B)**  

**Motor wires**  

Motor + to B01  

Motor − to B02  

**GPIO connections**  

PWMB to GPIO 26  

BIN1 to GPIO 23  

BIN2 to GPIO 24  

**Standby (board enable)**  

STBY to VCC (same motor driver)  

<br>  
<br>  

### Logic power  

Pi 3.3V to VCC on both TB6612 boards  

Pi GND to GND on both boards  

**(3.3V logic is ideal with Pi GPIO)**

<br>  

### Motor power  

**If you’re using your 2×18650 (2S) pack**:  

Battery + to VM on both boards  

Battery – to GND on both boards  

<br>  

### Common ground (critical)  

**All grounds must connect together**:  

Pi GND to TB6612 GND to Battery – (negative)  

<br>  

### STBY (enable)  

On TB6612 #1: STBY to VCC (same board)  
On TB6612 #2: STBY to VCC (same board)  

<br>  
<br>  

### COMPLETE THE CIRCUIT  

### To complete the circuit, connect STBY and VCC on BOTH MOTOR DRIVERS to the single 3.3V pin on the Raspberry Pi  

**Create a one-to-four jumper cable unless you are using a breadboard**  

**One way to do this is to make two three-ended jumper cables and connect one end of each to a third three-ended jumper cable.**  

**Then connect**:  

One end (the main stem) to the 3.3V pin on the Pi  

**Then connect the remaining four ends**:  

One end to VCC on MOTOR DRIVER 1  

One end to STBY on MOTOR DRIVER 1  

One end to VCC on MOTOR DRIVER 2  

One end to STBY on MOTOR DRIVER 2  
