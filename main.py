from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import math
from pymavlink import mavutil
import serial                                                              #Serial imported for Serial communication           


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def haversine(lat1, lon1, lat2, lon2):
      dLat = radians(lat2 - lat1)
      dLon = radians(lon2 - lon1)
      lat1 = radians(lat1)
      lat2 = radians(lat2)
      a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
      c = 2*asin(sqrt(a))
      return (rad(90) + c)


def get_distance_metres(aLocation1, aLocation2):
   
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
  
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def download_mission():
   
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(16)

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

#for current location
while True:
    nextwaypoint=vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))

#points

print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(-35.364260, 149.164037, 20)
vehicle.simple_goto(point1)

# sleep so we can see the change in map
time.sleep(5)

print("Going towards second point for 5 seconds (groundspeed set to 10 m/s) ...")
point2 = LocationGlobalRelative(-35.362609, 149.163528, 20)
vehicle.simple_goto(point2, groundspeed=10)

# sleep so we can see the change in map
time.sleep(5)

print("Going towards third point for 5 seconds (groundspeed set to 10 m/s) ...")
point3 = LocationGlobalRelative(-35.362452, 149.166185, 20)
vehicle.simple_goto(point3, groundspeed=10)

# sleep so we can see the change in map
time.sleep(5)

print("Going towards fourth point for 5 seconds (groundspeed set to 10  m/s) ...")
point4 = LocationGlobalRelative(-35.363811, 149.166405, 20)
vehicle.simple_goto(point4, groundspeed=10)

# sleep so we can see the change in map
time.sleep(5)

lon1 = 149.166405
lat1 = -35.363811
lon2 = 149.16512541
lat2 = -35.36248343

angle_to_be_rotated = haversine(lat1, lon1, lat2, lon2)

########code for arduino

ArduinoUnoSerial = serial.Serial('com15',9600)       #Create Serial port object called ArduinoUnoSerialData time.sleep(2)                                                             #wait for 2 secounds for the communication to get established

ArduinoUnoSerial.write(angle_to_be_rotated)
time.sleep(5)
      
print("payload is dropped")

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
