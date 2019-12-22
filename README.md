# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
The goal of this project is to navigate a car around a simulated highway scenario, including traffic and given waypoint, telemetry, and sensor fusion data. The car must not violate a set of motion constraints, namely maximum velocity, maximum acceleration, and maximum jerk, while also avoiding collisions with other vehicles, keeping to within a highway lane (aside from short periods of time while changing lanes), and changing lanes when doing so is necessary to maintain a speed near the posted speed limit.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./path_planning`.


## Rubic points
### Compilation
The code compiles correctly.
A new file "spline.h" was added in src. It is the Cubic Spline interpolation implementation: a single .h file that uses splines instead of polynomials. It was a great suggestion from Project Q&A and worked great.

### Valid trajectories
The car is able to drive at one loop(4.32 miles) without any incidents.
I ran the car for 9 miles without any incident.

### The car drives according to the speed limit.
No speed limit message(red) was seen during test execution.

### Max Acceleration and Jerk are not Exceeded.
No Max Acceleration and jerk message(red) was seen during test execetion.

### Car does not have collisions.
No collisions happened.

### The car stays in its lane, except for the time between changing lanes.
The car stays in its lane most of the time unless when its changes lane because of traffic or to return to the center lane.

### The car is able to change lanes
The car change lanes when the there is a slow car in front of it, and it is safe to change lanes (no other cars around) or when it is safe to return the center lane.

### Reflection
The main path planning code start at main.cpp line 99 to the line 349. The code can be separated into different functions to show the overall process, but I preferred to have everything in a single file than into different files. Comments were provided to improve the code readability.

My planning code consists of three parts:

### Prediction from line 107 to 158
Prediction block of code deals with the telemetry and sensor fusion data. It intents to reason about the car environment. The below aspects will be known from this code.
  Is there a car in front blocking the traffic.
  Is there a car to the right, making a lane change not safe.
  Is there a car to the left, making a lane change not safe.
These questions are answered by calculating the lane each other car is in and the position where it will be at the end of the last plan trajectory. A car is considered "dangerous" when its distance to our car is less than 30 meters in front or behind.

### Behavior from line 159 to 194
Behavior part decides what to do:
  If we have a car in front of us, do we change lanes?
  Do we speed up or slow down?

Based on the prediction of the situation where the car is, this code increases the speed, decrease speed, or make a lane change when it is safe. Instead of increasing the speed at this part of the code, a speed_diff is created to be used for speed changes when generating the trajectory in the last part of the code. This approach makes the car more responsive and acting faster to changing situations like a car in front applies breaks that causes a collision.

### Trajectory from line 230 to 348
This code does the calculation of the trajectory based on the speed and lane output from the behavior, car coordinates and past path points.
I take the ptsx and ptsy vectors and shift and rotate the points to local car coordinates (lines 231 to 292).To create a smooth path, two last points from the previous planned path and another 3 predicted future points are used together for computing a smooth spline curve(fitting spline is done in line 298). Using the spline curve calculated, I take the first 30m chunk (my target_x distance) and split it to generate the waypoints of my new trajectory. Each point represents the position of my car every 20ms, and for each I compare my velocity (ref_vel) to max speed or max acc, and accelerate or decelerate accordingly. I use each point velocity to calculate the x position of my car at the end of the 20ms interval, and use the spline to calculate my y coordinate. Finally, I shift and rotate these points back to global coordinates and save them in a next_x_vals and next_y_vals vectors. This new trajectory generate is fed to the simulator (lines 311 to 348).


## Additional information of the project

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```