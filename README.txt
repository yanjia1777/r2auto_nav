Finalz.py - the final code use for the demo

Autonomous navigation

Autonomous navigation is done through obtaining the occupancy map information and forming two images of the map, one with the unknown regions and walls of the map and one with just the walls. The code obtains the coordinates of unknown regions by overlapping the two images and subtracting one from the other. The coordinates are then passed through the astar pathfinding algorithm to find the shortest path to the destination. Lastly, the robot then makes it way to the destination. After each instance of pathfinding to a target coordinate, the robot will spin 360 degrees to check for IR targets. It then repeats its pathfinding with the new occupancy map information unless there are no more unknown regions.
