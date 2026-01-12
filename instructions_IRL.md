This is the minimal version of **AEmilioDistefano's ros2_ws repository**.  This version **does not include any virtualization** and **only includes what is necvessary control your robot(s) in the real world**.

When cloning onto a Raspberry Pi, **always clone this repository rather than the full version of the ros2_ws repository**.

# DIY Robot Setup (differential drive) 

By the end of this instructional you will have built **your own differential drive robot**, controllable via WiFi with your keyboard and ready for autonomous functions

<br>

![host-and-port](media/hide-and-seek-GIF.gif)

<br>

# MATERIALS :gear:

## ONE Raspberry Pi 4 with 4 gigs of RAM (we used Model B)
<br> 

![host-and-port](media/raspberry_pi_picture_1.jpeg) ![host-and-port](media/raspberry_pi_picture_2_box.jpeg) 
<br>
You can **use another Pi model**, but the flash SD card **preparation will be slightly different** (albeit **self-explanatory**) 
<br>

## ONE L298N Motor Controller

![host-and-port](media/motor_controller_1.jpeg) ![host-and-port](media/motor_controller_2_closeup.jpeg)
<br>
(depicted above is one example of a viable option, your motor controller may look different)


## ONE power bank or portable phone charger (at least 10,000 mAh, QC at least 18W, PD at least 18W),  **must include USB to type-c charge cable**) 

![host-and-port](media/portable_power_bank.png)
<br>
(depicted above is one example of a viable option, your power bank may look different)


## ONE fuse holder **with at least one 5W fuse** (can be for car, motorcycle, or other type of machine)

![host-and-port](media/fuse_and_holder_1.jpeg) ![host-and-port](media/fuse_and_holder_2_insertion.jpeg)
<br>
(depicted above is one example of a viable option, your fuse and fuse holder may look different)


## ONE simple ON /OFF switch 

![host-and-port](media/on_off_switch.jpeg) 
<br>
(depicted above is one example of a viable option, your ON / OFF switch may look different)


## AT LEAST TWO 3.7V 18650 Li-ion battery (at least 2,200 mAh) (make sure to also get the battery charger!)

![host-and-port](media/li-ion_batteries.jpeg) 
<br>
(depicted above is one example of a viable option, your batteries may look different)


## AT LEAST TWO battery holders for your Li-ion batteries

![host-and-port](media/li-ion_battery_holders.jpeg) 
<br>
(depicted above is one example of a viable option, your battery holders may look different)


## ONE USB webcam 

![host-and-port](media/webcam.png) 
<br>
(depicted above is one example of a viable option, your webcam may look different)


## ONE PACK of female-to-female jumper cables 

![host-and-port](media/jumper_cables_female_to_female.jpeg) 
<br>
(depicted above is one example of a viable option, your jumper cables may look different)

## ONE PACK of male-to-female jumper cables

**NOTE:** The female side of these jumper cables will be cut off, so **male-to-male jumper cables would also work here**
**HOWEVER**, you will definitely need at least one female-to-male jumper cable to connect the GND on the Pi to GND on the motor controller

![host-and-port](media/jumper_cables_male_to_female.jpeg) 
<br>
(depicted above is one example of a viable option, your jumper cables may look different)


## ONE chassis with **FOUR** DC gear motors (make sure to buy a wheel for each motor, 4 total)

**These DC gear motors are easy to find at hobby shops**.  You can make your own chassis or buy an assembled chassis that already includes four motors similar to the yellow DC gear motors depicted below.

The tracks in the image were 3D printed on a Creality K1C.  Hand-straightened paperclips were used as connectors for the track segments.

![host-and-port](media/motors_with_chassis.jpeg) 
<br>
(depicted above is one example of a viable option, your chassis may look different)


We will not include instructions on how to make your own chassis, these can be bought ready-made or assembled easily from a small rectangle of sheet metal and some simple tools. 

The outer shell of the robot in the GIF was made using a 3D printer, but this can be made by hand or made from a recycled plastic shell of any kind.

<br>
<br>
<br>

# TOOLS :toolbox:

Almost all of these tools can be replaced with basic household items, but we will include this list of tools for those who have them on-hand

**at least ONE** pair of tweezers to hold wires steady within the robot during final build steps

**ONE** screwdriver set, or a screwdriver with a tip small enough for M2, M2.5, or M3 screws (these screws will be used to attach your pi and your motor controller to your chassis, if you dedice to connect these components to the chassis in another way then the screwdriver will not be needed)

**ONE** wire stripper (or a nail clipper and very steady hands) 

**ONE** hot silicone gun (or any thick adhesive)

<br>
<br>
<br>

# 1. PREPARE your SD card for the Raspberry Pi by flashing it with Ubuntu 24.04 Noble (server)

### Use Raspberry Pi Imager

**Then insert the SD into your Pi**

<br>
<br>
<br>

# 2. Connect your Pi to your motor controller

![host-and-port](media/PI_MC_all_1.jpg)

After **mounting the Pi and the Motor Controller onto the chassis**, it's time to wire everything together.

The GPIO pins on the Raspberry Pi will connect to the pins on your L298N Motor Controller.  The controler used in this tutorial has pin labels **ENA, IN1, IN2, IN3, IN4, and ENB**.  These are the controler pins that look like the GPIO pins on your Raspberry Pi.  If your controler's pin labels are different, check the list below to see which of your pins 

**ENA** is sometimes labeled as **EN A**, **EA**, **PWM_A**, **PWMA**, **EN1**, or **ENABLE_A**

**IN1** is sometimes labeled as **INA1**, **AIN1**, or **I1**

**IN2** is sometimes labeled as **INA2**, **AIN2**, or **I2**

**IN3** is sometimes labeled as **INA3**, **AIN3**, or **I3**

**IN4** is sometimes labeled as **INA4**, **AIN4**, or **I4**

**ENB** is also sometimes labeled as **ENB**, **EN B**, **EB**, **PWM_B**, **PWMB**, **EN2**, **ENABLE_B**

### 2.1 Connect the following pins on the motor controller to the folowing pins on the Raspberry Pi:

**ENA** to **GPIO 12** AKA **Pin 32** AKA **PWM0** 

**IN1** to **GPIO 23** AKA **Pin 16**

**IN2** to **GPIO 22** AKA **Pin 15**

**IN3** to **GPIO 27** AKA **Pin 13**

**IN4** to **GPIO 17** AKA **Pin 11**

**ENB** to **GPIO 13** AKA **Pin 33** AKA **PWM1**

These connections are depicted below:

![host-and-port](media/GPIO_to_controller_2.jpeg) 

![host-and-port](media/GPIO_closeup.jpg)

The GPIO pins connected to the motor controller will be referenced in our code:

![host-and-port](media/GPIO_code_1.png) ![host-and-port](media/GPIO_code_2.png)

<br>
<br>
<br>

# 3. Connect your motors to your motor controller 

## 3.1 Take eight of your male-to-female (or male-to-male) jumper cables and perform the following connections:

<br>

![host-and-port](media/MC_motors_2.jpg) 

<br>
<br>
<br>

# 4. 

![host-and-port](media/circuit_all.jpg)

![host-and-port](media/robot_guts_GIF.gif)

<br>
<br>
<br>

# 5. Install required software onto your Raspberry Pi

## 5.1 SSH into your Pi 
<br>
In the following command, replace **name** with **the name you set for the Pi** when you flashed the Ubuntu image with **Raspberry Pi Imager**, and **replace <host>** with **the host name you set** when you flashed the Ubuntu image with **Raspberry Pi Imager** (DO NOT include the pointed brackets / carrot symbols):

```shell
ssh <name>@<host>.local

```

You will be prompted to enter **the password you set for the Pi** when you flashed the Ubuntu image with **Raspberry Pi Imager**

<br>

## 5.2 Install ROS 2 Jazzy (minimal) on the Pi
<br>
Enter the following commands to prepare your system:

```shell
sudo apt update && sudo apt upgrade -y

# Set locale (if not already)
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

```

Then install **build-essential** (includes a **C++ compiler**)

```shell
sudo apt-get update
sudo apt-get install build-essential
```

Then add the ROS2 repo:

```shell
sudo apt install -y curl gnupg2 lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions

```

Now add sourcing to your .bashrc file on the Pi:

```shell
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

```

Add some packages for GPIO control, camera and CV bridge, and other tools:

```shell
sudo apt install -y python3-rpi.gpio python3-opencv \
  ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs ros-jazzy-std-msgs

```

<br>

## 5.3 Set Up ROS2 networking between your laptop and your Pi
<br>
Add these lines to the .bashrc file of both machines (Laptop and Pi):

```shell
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Same domain for laptop and Pi
export ROS_DOMAIN_ID=17

# Allow DDS to use the network (not localhost only)
export ROS_LOCALHOST_ONLY=0

```

Then source .bashrc

```shell
source ~/.bashrc

```

<br>

## 5.4 Clone the Robot Legion Teleop package
<br>
Clone this repository into your Raspberry Pi:

```shell
git clone https://github.com/AEmilioDiStefano/ros2_ws_minimal.git ros2_ws
```

<br>


## 5.5 Enable GPIO Access (One-Time Setup)
<br>
The motor driver node uses Raspberry Pi GPIO. By default, Linux restricts GPIO hardware access, which will cause this error:

```shell
RuntimeError: No access to /dev/mem. Try running as root!

```

Add the current user to the GPIO group:

```shell
sudo usermod -aG gpio $USER

```

Then create a udev rule so GPIO is accessible to non-root users:

```shell
sudo usermod -aG gpio $USER

```

Reload these new rules and reboot

```shell
sudo tee /etc/udev/rules.d/99-gpiomem.rules >/dev/null <<'EOF'
KERNEL=="gpiomem", GROUP="gpio", MODE="0660"
EOF

```

**SSH back into your Raspbarry Pi** and confirm the changes

```shell
ls -l /dev/gpiomem

```

You should see **GPIO** and **crw-rw----** included in the output.

<br>
<br>
<br>

# Start and control your robot 

<br>

## 1. Power up your robot

**Turn on the power bank** that powers the Raspberry Pi

**Switch on** the 'ON' switch for your motors and motor controller.

** Wait about 120 seconds** for the Pi to get online

<br>

## 2. Open two different terminals 

**Both will stay opened** while you are controlling your robot

**In one terminal**, SSH into your robot's Raspberry Pi

**Now enter the newly cloned ros2_ws directory**:

```shell
cd ros2_ws
```

**IF YOU DO NOT HAVE THIS DIRECTORY**, clone **the minimal version of the ros2_ws workspace** onto your Pi:

```shell
git clone https://github.com/AEmilioDiStefano/ros2_ws_minimal.git ros2_ws
```



**In the other terminal (on your laptop)**, enter your existing ros2_ws directory:

```shell
cd ros2_ws
```

<br>

**In both terminals**, enter the following command:

```shell
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

**What this does:** 

   - Ensures that **the ROS2_LOCALHOST_ONLY environment variable is not set** (otherwise all ROS 2 communications are restricted to the local loopback interface)
   - Puts both the **laptop and the robot on the same ROS2_localhost_domain** (otherwise the machines will not be able to communicate)
   - Sources **Jazzy's setup.bash** file
   - Sources **your workspace's setup.bash** from the install directory (this is generated when you **colcon build**)
<br>

## 3. On your Pi, run the motor controller node with the following command:


```shell
ros2 run robot_legion_teleop_python motor_driver_node
```

<br>
<br>
<br>

## 4. On your laptop, run the teleop node to control your robot:

```shell
ros2 run robot_legion_teleop_python legion_teleop_key
```

<br>
<br>
<br>

## 5. Play hide-and-seek with your cat 

<br> 

![host-and-port](media/hide-and-seek-GIF.gif) 

<br>

**Make sure the terminal running the teleop node is selected** (click anywhere inside the terminal itself)

<br>

**Use the arrow keys for direction** and the spacebar to stop.  You can also use your **numpad with numlock** on to control as follows:

**8** to **GO FORWARD**  
**2** to **GO BACKWARD**  
**4** to **ROTATE LEFT**  
**6** to **ROTATE RIGHT**  

**7** to **CIRCLE LEFT FORWARD**  
**9** to **CIECLE RIGHT FORWARD**  
**1** to **CIRCLE LEFT BACKWARD**  
**3** to **CIRCLE RIGHT BACKWARD**  

**5** to **STOP**  

<br>


