# chaser_drone

A robot and a drone are given. The robot is the Runner representing an intruding animal and the drone is the Catcher. To avoid more damage to the crops the Catcher is a drone and not another ground based robot. The Runner negotiates the arena, representing a farm, following a random path given. The Catcher flies above the arena and 'catches' the Runner by landing on it. Both the Catcher and Runner are localized by markers and an overhead camera is used to track them. A PC/Laptop running the Robot Operating System (ROS) is used to track the robots and issue motion commands for the Catcher. 

While moving in the farm, the Runner can hide in small caves while the Catcher tries to locate it within the field. The challenge is to land the Catcher on the Runner in the shortest time possible. 
