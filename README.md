# Indoor Autunomous Navigation with Parrot AR Drone 2.0

My senior project; <br/> 
An implementation of ai  to the parrot ar drone in order navigate through 
indoor corridor. The UAV will detect pedestrians and try to avoid them with stopping
until pedestrians move out from its way with image processing. 


## Used Technologies

* OpenGl for pedestrian detection
* Robotic Operating System (ROS) for the comminucation in program
* ROS Driver for Parrot AR-Drone 1.0 & 2.0 Quadrocopters -> [ardrone_autonomy](https://github.com/AutonomyLab/ardrone_autonomy)


## Features

The main controller program written in c++, becuase of the python drivers run in parallel tasks which prevents immediate commands. 
In order to stop when a pedestration detection, the program must be asyn and it should be thread safe. The controller receives inputs from odometry, localization and image programs and decides what do to next during the UAV flight. 

Below you may found, youtube link of full autonomous flight of ar drone.
<br/>
<br/>

**[Full Autonomous In Door Navigation Flight](https://www.youtube.com/watch?v=Q-z_uf2PPwA)** <br/>

[![Full Autonomous In Door Navigation Flight](https://img.youtube.com/vi/Q-z_uf2PPwA/0.jpg)](https://www.youtube.com/watch?v=Q-z_uf2PPwA)


## Disclaimers

* This project is implemented for specific corridor, It will work as expected for similiar corridors.

* The horizontal alignment is maintained with its front camera with a decision making program to look forward lights which is centered on the ceiling of the corridor

* The drone understands its final location with the marks of the corridor
