 task by stuti jain
 
 # TASK STATEMENT 
 You are requested to make a single UAV follow a pre-planned Trajectory on Ardupilot SITL such that it goes to a Specified Drop Location, drops a Payload and Returns back to Its Take-off Location. At the payload drop, a servo motor is flicked as a confirmation for successful mission.
 
  # TASK DETAILS  
Write a single script incorporating functions mentioned below 
  
  # UAV Navigation:  
Develop a python script such that UAV Take-off, follow specified Coordinates and Lands back at starting point all Autonomously. The script must be showing/printing Real-Time UAV Location and Distance from the next waypoint at all times during the mission.​ ​A coordinate file consisting of 5 coordinates (Take-off/Landing Point, 
Point A, Point B, Point C, Payload Drop Coordinate, Point D) will be provided to you.  
 
  # Payload Drop:  
Develop a function for Autonomous drop of Payload by a ​moving​ UAV using Haversine Formula such that a spherical Payload of 2 Kg drops along a​ parabolic trajectory​ in an ​acceptable range of distance​ ​from drop 
Location​ and ​print a confirmation message​ that payload has been Dropped. (Assume Air Drag Coefficient and Height of UAVs) 

  # Servo Motor Flick. 
As soon as the UAV reaches the desired co-ordinates for the payload drop, the program is supposed to execute a simple function to move a servo motor using Arduino uno microcontroller to indicate the successful payload drop.


# installation :
## install ubuntu 20.4 in dual mood with windows 10
###### Link u can consider  https://www.youtube.com/watch?v=qNeJvujdB-0  
   
## Download python 3.9.7
   ###### https://www.python.org/downloads/
   ###### learn basic python from https://automatetheboringstuff.com/​ (First 6 chapters are sufficient)
## Dronekit installation
   ### on terminal
   ##### sudo apt update
   ##### sudo apt-get install python3-pip python3-dev
   ##### pip3 install dronekit
   ##### pip3 install dronekit-sitl
   ##### pip install dronekit-sitl -UI
   ##### git clone http://github.com/dronekit/dronekit-python.git
## Ardupilot installation
   ### on terminal
   #####    sudo apt-get update
   #####    sudo apt-get install git
   #####    sudo apt-get install gitk git-gui
   #####    git clone https://github.com/ArduPilot/ardupilot
   #####    cd ardupilot
   #####    git submodule update --init --recursive
   #####    Tools/environment_install/install-prereqs-ubuntu.sh -y
   #####    . ~/.profile
   #####    docker build . -t ardupilot
   #####    docker run --rm -it -v `pwd`:/ardupilot ardupilot:latest bash
      
## for arduino
   ### on terminal
#####    pip install pyserial
#####   	https://www.arduino.cc/en/guide/linux
		
		
## mavproxy
  ### for terminal 
##### sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
##### 	pip3 install PyYAML mavproxy --user
##### 	echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
##### 	pip3 install mavproxy --user --upgrade
##### 	mavproxy.py --master=/dev/ttyUSB0 --cmd="param load init.parm; module load map;"

     
     
# running the code
 run the code on terminal with python3 main.py(if saved on home)
 
 
# code explaination
 modules used:
``` from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import math
from pymavlink import mavutil
import serial  ```


#####  for connecting the vehicle
```from dronekit import connect

vehicle = connect('127.0.0.1:14550', wait_ready=True)```

##### for finding the angle at which servo motor should be rotated 

```def haversine(lat1, lon1, lat2, lon2):
      dLat = radians(lat2 - lat1)
      dLon = radians(lon2 - lon1)
      lat1 = radians(lat1)
      lat2 = radians(lat2)
      a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
      c = 2*asin(sqrt(a))
      return (rad(90) + c)```
##### for finding the distance

```def get_distance_metres(aLocation1, aLocation2):
   
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5```
		
##### for sending value to arduino
```ArduinoUnoSerial = serial.Serial('com15',9600)       #Create Serial port object called ArduinoUnoSerialData time.sleep(2)                                                             #wait for 2 secounds for the communication to get established

ArduinoUnoSerial.write(angle_to_be_rotated)
time.sleep(5)```



     
