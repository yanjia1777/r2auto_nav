**Main Files**

final.py - Final code use for the demo

omrond6t.py - To test condition of thermal sensor

combined.py - Subscriber to Finalz.py to receive angular change in barrel and command to fire

r2auto_nav.py - Original r2auto_nav with firing mechanism only.

dcmotorsolenoid.py - To test dcmotor, servo solenoid and servo motor for changing the angle of the barrel


**Objective:**

The objective of this program is to navigate and map an unknown area of up to 8m x 8m. The mapped data will be converted to an occupancy map image and save. It is also required to locate, aim and fire at an infrared target within the map while the robot is navigating the map.


**Autonomous Navigation:**

Autonomous navigation is done through obtaining the occupancy map information and forming two images of the map, one with the unknown regions and walls of the map and one with just the walls. The code obtains the coordinates of unknown regions by overlapping the two images and subtracting one from the other. The coordinates are then passed through the astar pathfinding algorithm to find the shortest path to the destination. Lastly, the robot then makes it way to the destination. After each instance of pathfinding to a target coordinate, the robot will spin 360 degrees to check for IR targets. It then repeats its pathfinding with the new occupancy map information unless there are no more unknown regions.


**Firing Mechanism:**

During the 360 degree rotation to check for IR targets. If it detects max(temperature) of 85 Fahrenheit in any of the array, it stops and calibrates its rotation and angle of the barrel. If the distance to the target is more than 40cm, it will move forward in its existing direction and stop once it is 0.6m away from the target. It will calibrate once more to the target. If the distance to the target is less than 40cm, it will not move forward and readjust again. After any of the 2nd Calibration, the dc motor will turn on and after 5 seconds, the servo solenoid will push the ball forward to the flywheel, pushing the ping pong ball to hit it's target  
