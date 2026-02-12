# GPIO connections from Raspberry Pi 4 to DUAL Tb6612fng motor controllers to motors for a TANK TRACK DIFFERENTIAL DRIVE Chassis  

### **This version wires each SIDE as a paired set**:  
###   - LEFT FRONT and LEFT REAR receive the SAME PWM and direction signals  
###   - RIGHT FRONT and RIGHT REAR receive the SAME PWM and direction signals  
###   This prevents motors from fighting each other in a tank-style track system.  

#  
#  

### IMPORTANT:  This robot uses a <u>Tb6612</u> motor driver.  

### ALL WIRING from the Li-Ion batteries to the motor drivers MUST BE at least 18 AWG    

### For wiring from the motor drivers to the Pi, common Dupont jumper cables are ok.

#  
# 

## POWER YOUR MOTOR CIRCUIT (USE 18 AVG!)

**This project uses battery holders for two 3.7V 2800mAh Li-Ion battteries**

**IF** your battery holder(s) have attached cable thinner than 18 AWG, replace all connections with 18 AWG cable

**FIRST** connect the **positive** side of the battery holder(s) to your ON/OFF switch  

**THEN** connect the other side of the ON/OFF switch to your fuse holder (with fuse) 

**THEN** solder **THREE MORE 18 AWG CABLES** (around 6-8 inches long) to the other side of the fuse holder

### This is the POSITIVE SIDE of your circuit 

**SOLDER ONE OF POSITIVE 20 AWG CABLES** to the **VM** on one of your Tb6612 motor controllers  

**SOLDER A SECOND POSITIVE 20 AWG CABLE** to the **VM** on the other Tb6612 motor controller  

**SOLDER THE THIRD POSITIVE 20 AWG CABLE** to a **1 kΩ (1000 ohm) 1 watt resistor**  

**THEN** connect the other side of the resistor to the positive side of a small LED  

### Now connect three more 18 AWG cables to the NEGATIVE SIDE of your battery holder(s) 

**CONNECT ONE OF YOUR NEGATIVE 18 AWG CABLES** to one of the **GND**s on one of your motor drivers

**CONNECT A SECOND NEGATIVE 18 AWG CABLE** to one of the **GND**s on **the other motor driver**  

**FINALLY**, connect the third and last remaining 18 **NEGATIVE 18 AWG CABLE** to the free side of the LED already connected to the positive side of the circuit  

### Now test your circuit by switching the ON/OFF button  

**THE LED SHOULD LIGHT UP**  

**This LED is used as a debugging tool** informing the operator when the circuit is successfully powering the motor system

If the LED does not light up, check your circuit and your batteries. 

### IMPORTANT: Make sure to leave the ON/OFF switch on OFF while you perform ANY CHANGES to your circuit or your GPIO pins, otherwise you could blow a fuse  

#  
#  

<br>  

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






