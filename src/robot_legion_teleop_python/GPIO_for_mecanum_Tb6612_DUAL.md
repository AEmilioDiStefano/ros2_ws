# GPIO connections from Raspberry Pi 4 to DUAL Tb6612 motor controllers for a MECANUM Chassis  

### IMPORTANT:  This robot uses a <u>Tb6612</u> motor driver.  

### ALL WIRING from the Li-Ion batteries to the motor drivers MUST BE at least 18 AWG    

### For wiring from the motor drivers to the Pi, common Dupont jumper cables are ok.

<br>  

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

<br>  
<br>  

## TB6612 #1 — FRONT MOTORS  

**FRONT-LEFT MOTOR (Channel A)**  

**Motor wires**  

Motor + to A01  

Motor − to A02  

**GPIO connections**  

PWMA to GPIO 12 (PWM0)  

AIN1 to GPIO 5  

AIN2 to GPIO 6  

<br>

**FRONT-RIGHT MOTOR (Channel B)**  

**Motor wires**  

Motor + to B01  

Motor − to B02  

**GPIO connections**  

PWMB to GPIO 13 (PWM1)  

BIN1 to GPIO 16  

BIN2 to GPIO 19   

<br>  
<br>  
<br>  

## TB6612 #2 — REAR MOTORS  

**REAR-LEFT MOTOR (Channel A)**  

**Motor wires**  

Motor + to A01  

Motor − to A02  

**GPIO connections**  

PWMA to GPIO 18  

AIN1 to GPIO 20  

AIN2 to GPIO 21  

<br>  

**REAR-RIGHT MOTOR (Channel B)**  

**Motor wires**  

Motor + to B01  

Motor − to B02  

**GPIO connections**  

PWMB to GPIO 26  

BIN1 to GPIO 23  

BIN2 to GPIO 24  

<br>  
<br>  

## COMPLETE THE CIRCUIT

### Logic power  

Pi 3.3V to VCC on both TB6612 boards  

Pi GND to GND on both boards  

<br>  

### Motor power  

Battery + to VM on both boards  

Battery – to GND on both boards  

<br>  

### Common ground (critical)  

**All grounds must connect together**:  

Pi GND to TB6612 GND (both drivers) 

TB6612 GND to Battery **Negative** (both drivers)  

<br>  
<br>  

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
