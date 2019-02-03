# SAWYER
- Prerequisite
 1. python <3.6, ubuntu 16.04
 2. install ros, intera sdk and gazabo (not necessary) following intruction on https://github.com/RethinkRobotics/intera_sdk
- Usage  
  - The script has to be run under ros environment and connected to the robot.  
  - Open ros (the intera.sh is in ros_ws folder by default):
  > cd ~/row_ws  
  > ./intera.sh  
  - Check connevtivity
  > rostopic list
  - run  (change goal xyz and xyzw in test.py)
  > python test.py  
