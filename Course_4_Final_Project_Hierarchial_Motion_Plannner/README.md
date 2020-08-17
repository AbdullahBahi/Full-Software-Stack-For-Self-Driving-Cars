# Course 4 Final Project | Hierarchial Motion Plannner for Self-Driving Cars
  
## Project Overview  
In this project I implemented many of the behavioral and local planning concepts discussed in Course 4. The goal of this project is building a functional motion planning stack that can avoid both static and dynamic obstacles while tracking the center line of a lane, while also handling stop signs. To accomplish this, I implemented The following blocks and the integrated them:
- Behavioral planning logic
- Static collision checking
- Path generation
- Path selection 
- Velocity profile generation
  
## Implementing the Motion Planner  
  
To implement this project, I edited the "behavioural_planner.py", "collision_checker.py", "local_planner.py", "path_optimizer.py", and "velocity_planner.py" class files in the skeleton provided by the course. Then these files are integrated with CARLA simulator as well as the controller that we implemented in the final project of course 1.
  
## How to run th simulation
  
To run this simulation using my code, the CARLA simulator along with the assignment code needs to be installed as follows:  
  
1. install CARLA simulator as instructed in week 7 course 4 "Motion Planning for Self-Driving Cars".
2. Create a subfolder "Course4FinalProject" under "PythonClient" folder which is found inside the "CarlaSimulator" (root) folder.
3. Copy the contents of this folder "Course_4_Final_Project_Hierarchial_Motion_Plannner" to the folder you created.
4. Follow the rest of the instructions in the course to run the simulation.
  
## Simulation in Action
You can view the simulation in real-time from [here](https://youtu.be/Adl7bE_Xtp8).
