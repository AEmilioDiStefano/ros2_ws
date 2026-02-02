# GPIO connections from TB6612 dual motor controllers to Raspberry Pi 4 for MECANUM Chassis

### TB6612 #1 (Front motors)

**Front-Left** (Motor A)

PWMA to GPIO 12 AKA pin 32 AKA PWM0

AIN1 to GPIO 5  AKA Pin 29

AIN2 to GPIO 6  AKA Pin 31

**Front-Right** (Motor B)

PWMB to GPIO 13 AKA Pin 33 AKA PWM1

BIN1 to GPIO 16 AKA Pin 36

BIN2 to GPIO 19 AKA Pin 35

### TB6612 #2 (Rear motors)

**Rear-Left** (Motor A)

PWMA to GPIO 18 AKA Pin 12 AKA PCM_CLK

AIN1 to GPIO 20 AKA Pin 38 AKA PCM_DIN

AIN2 to GPIO 21 AKA Pin 40

**Rear-Right** (Motor B)

PWMB to GPIO 26 AKA Pin 37 

BIN1 to GPIO 23 AKA Pin 16

BIN2 to GPIO 24 AKA Pin 18