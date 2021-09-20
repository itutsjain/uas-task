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
1. install ubuntu 20.4 in dual mood with windows 10
   Link u can consider  https://www.youtube.com/watch?v=qNeJvujdB-0  
   
2. Download python 3.9.7
     https://www.python.org/downloads/
     learn basic python from https://automatetheboringstuff.com/​ (First 6 chapters are sufficient)
3. Dronekit installation
     on terminal
       sudo apt update
       sudo apt-get install python3-pip python3-dev
       pip3 install dronekit
       pip3 install dronekit-sitl
       pip install dronekit-sitl -UI
       git clone http://github.com/dronekit/dronekit-python.git
4. Ardupilot installation
    on terminal
      sudo apt-get update
      sudo apt-get install git
      sudo apt-get install gitk git-gui
      git clone https://github.com/ArduPilot/ardupilot
      cd ardupilot
      git submodule update --init --recursive
      Tools/environment_install/install-prereqs-ubuntu.sh -y
      . ~/.profile
      docker build . -t ardupilot
      docker run --rm -it -v `pwd`:/ardupilot ardupilot:latest bash
     
     

     


 
 
 
 
 
    • Learn Python 3: ​https://automatetheboringstuff.com/​ (First 6 chapters are sufficient) 
    • Drone kit- Python: ​https://dronekit-python.readthedocs.io/en/latest/ 
    • Pyserial- Python: https://pythonhosted.org/pyserial/
    • Ardupilot SITL:  ​https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html  
