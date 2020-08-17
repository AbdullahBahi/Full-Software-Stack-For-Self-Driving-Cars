# Course 1 Final Project | Self-Driving Vehicle Control
  
## Project Overview  
In this project, I wrote and implemented a controller for the CARLA simulator. Our goal was to control the vehicle to follow a race track by navigating through preset waypoints. The vehicle needs to reach these waypoints at certain desired speeds, so both longitudinal and lateral control will be required.  
  
## Project Specifications  
In this project, I implemented a simple controller in Python and used it to drive a car around a track in Carla. The track is a loop shown in this [figure](https://github.com/AbdullahBahi/Building-Full-Self-Driving-Car-Software-stack/blob/master/Course_1_Final_Project_Vehicle_Control/figure.PNG). I was given a sorted list of waypoints which are equally spaced on this track. The waypoints include their positions as well as the speed the vehicles should attain. As a result, the waypoints become the reference signal for the controller and navigating to all the waypoints effectively completes the full track.  
  
Since the controller reference contains both position and speed, I implemented both:
- longitudinal control - PID controller
- lateral control - Stanley controller
  
The [output of the controller](https://github.com/AbdullahBahi/Building-Full-Self-Driving-Car-Software-stack/tree/master/Course_1_Final_Project_Vehicle_Control/controller_output) is the vehicle throttle, brake, and steering angle commands. The throttle and brake come from the longitudinal speed control and the steering comes from the lateral control.  
  
## How to run th simulation
To run this simulation, the CARLA simulator along with the assignment code needs to be installed as follows:  
  
1. install CARLA simulator as instructed in week 7 course 1 "Introduction to Self-Driving Cars".
2. Create a subfolder "Course1FinalProject" under "PythonClient" folder which is found inside the "CarlaSimulator" (root) folder.
3. Copy the contents of this folder "Course_1_Final_Project_Vehicle_Control" to the folder you created.
4. Follow the rest of the instructions in the course to run the simulation.
  
## Simulation in Action
You can view the simulation in real-time from [here](https://youtu.be/SwaFDSyjZF0).
