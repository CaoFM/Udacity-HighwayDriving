
# **Highway Driving** 

**Udacity Highway Driving Project**

The goals / steps of this project are the following:
* Design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic
* Run the test vehicle in Udacity Simulator successfully around at least one loop and be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.
* Summarize the results with a written report


## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1971/view) individually and describe how I addressed each point in my implementation.  

---
### Compilation

#### 1. The code compiles correctly.
My code compiles.
```sh
cmake .. && make
```

### Valid Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident.
I ran simulator and the car drove a full lap without incident.

#### 2. The car drives according to the speed limit.
The car drives ~ 49 mph when path is clear.

The desired speed is limited to `SPEED_LIMIT` which is defined in `configuration.h`.

#### 3. Max Acceleration and Jerk are not Exceeded.
This is achieved by using history points to smooth the trajectory and limit the speed change per loop.

#### 4. Car does not have collisions.
This is made sure by 
* controlling speed based on the time and distance gap to the vehicle ahead 
* punish lane changes when there are vehicle near the future path 

#### 5. The car stays in its lane, except for the time between changing lanes.
The trajectory is generated to maintain `d` value in the lane when not changing lanes.

#### 6. The car is able to change lanes
I built 3 cost function to decide when to keep current lane or change lane. When the cost of changing lane is lower, the car will change lane. The cost function are as below:
* `calculate_speed_cost()`: punishes if the target lane travels slower than speed limit.
* `calculate_contact_cost()`: punishes if the maneuver will lead into close contact with other vehicle.
* `calculate_trouble_cost()` : punishes unnecessary lane changes only to gain very little speed advantage; and favor left lane change over right because it is better to pass others on the left.

 
### Reflection

#### 1. There is a reflection on how to generate paths.
To generate path:
- Carry some points from previous path if they are available (up to X points)
- Run my state machine and decide whether to keep lane or change lane based on the overall cost associated with each possibility.
- Choose a future point on the map based on which lane I will be in.
- Use spline to fit all the known points.
- Calculate desired speed based on the gap between ego vehicle and the vehicle in front.
- Add more points on the spline based on the speed.
- Finally append all points to the array and pass to the simulator.

