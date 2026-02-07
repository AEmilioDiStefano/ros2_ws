# GPIO connections from Raspberry Pi 4 to DUAL Tb6612fng motor controllers to motors for a TANK TRACK DIFFERENTIAL DRIVE Chassis  

### **This version wires each SIDE as a paired set**:  
###   - LEFT FRONT and LEFT REAR receive the SAME PWM and direction signals  
###   - RIGHT FRONT and RIGHT REAR receive the SAME PWM and direction signals  
###   This prevents motors from fighting each other in a tank-style track system.  

#  
#  

### TB6612 #1 - LEFT SIDE MOTORS (paired together)  

**Left Front Motor (Channel A)**  

**Motor wires**  

Motor + to A01  

Motor − to A02  

**Left Rear Motor (Channel B)**  

**Motor wires**  

Motor + to B01  

Motor − to B02  

**GPIO connections (tied so BOTH channels get identical commands)**  

PWMA **and** PWMB tied together → GPIO 12 (PWM)  

AIN1 **and** BIN1 tied together → GPIO 5  

AIN2 **and** BIN2 tied together → GPIO 6  

#  
#  

<br>  

#  
#  

### TB6612 #2 - RIGHT SIDE MOTORS (paired together)  

**Right Front Motor (Channel A)**  

**Motor wires**  

Motor + to A01  

Motor − to A02   

**Right Rear Motor (Channel B)**  

**Motor wires**  

Motor + to B01  

Motor − to B02  

**GPIO connections (tied so BOTH channels get identical commands)**  

PWMA **and** PWMB tied together → GPIO 18 (PWM)  

AIN1 **and** BIN1 tied together → GPIO 23  

AIN2 **and** BIN2 tied together → GPIO 24  

#  
#  

<br>  

#  
#  

### ADD BATTERIES TO POWER MOTORS  

### Common ground
**All grounds must connect together (critical)**:  

Pi GND to TB6612 GND (LEFT SIDE)

Pi GND to TB6612 GND (RIGHT SIDE)

#  
#   

### Motor power  

Battery + to VM on both boards (make a 3-sided jumper wire **OR** use a breadboard)  

Battery – to GND on both boards (make a 3-sided jumper wire **OR** use a breadboard)  

#  
#   

<br>  

#  
#  

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

#  
#  

<br>  

#  
#  

### ADDITIONAL CONNECTIONS  

### Add a fuse holder and an ON/OFF switch to the circuit  

**Immediately after the positive side of the battery within the circuit**:  

Add a fuse holder (with a 5 amp fuse inside) and an ON/OFF switch to your circuit  

#  
#  

### Add a LED as a circuit debugging tool  

**Connect an LED to the motor driver circuit so that you know that the circuit is powered (the Tb6612 motor driver does not come with a debugging light)**  

**First connect one LED to one 1 kΩ resistor (500 Ω will be brighter, 2 kΩ will be dimmer)**:  

Connect the longer wire (the positive side) of the LED to either side of the resistor  

**Then connect the LED and resistor to the circuit**:  

Connect the free end of the resistor to the positive side of the circuit (after the fuse holder and the ON/OFF switch, and before everything else)  

Connect the free side of the LED to the negative side of the circuit (right before the negative side of the battery)  






