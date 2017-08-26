# Path Planning Project

In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provides the car's localization and sensor fusion data. There is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

### Output

Click to view the video

[![Final results with the implementation](http://img.youtube.com/vi/fvdJNPNk_7k/0.jpg)](http://www.youtube.com/watch?v=fvdJNPNk_7k)

### Some aspects of Valid Trajectories

**1. The car drives according to the speed limit of 50mph: **This has been addressed by setting a variable called `ref_vel` in the `main()` function. The car is allowed to accelerate only as long long as the the value of the variable is less than or equal to 49.3mph. Acceleration is done by increasing this value in increaments of 0.5 in every cycle. The line `double N = (target_dist/(0.02*ref_vel/2.24));` uses this variable to better plan the trajectories.

**2. Max Acceleration and Jerk are not Exceeded: ** The acceleration value has been been empirically set to 0.5. I experimented with various values from 0.2 to 1.5. Values less than 0.4 lead to a very slow acceleration which sometimes made the lane changes a bit too slow. Values more than 0.6 lead the car to sometimes exceed it maximum allowed jerk levels of 10m/s^2 in some instances. Also I observed that there were some instances where the car would perform a single lane change and then find another lane change in the very next iteration. This sometimes lead to sudden swerves and would cause the jerk values to go out of the acceptable range. To take care of this, the car is allowed only one lane change every second, or only one lane change in every 5 iterations of 0.2 seconds. This has been achieved by setting a counter variable called `num_lane_shifts` in the `main()` function. Before every lane change the program must make sure that the counter variable has value that is a multiple of 5. This variable is incremented by 1 in every iteration thus making sure that the lane shift happens only ince in every 5 cycles.

**3. Car does not have collisions: **To avoid rear-ending other vehicles, there is a sensor-fusion and de-acceleration component as well. The program makes sure that our vehicle maintains a safe distance of 30.0 units in Frenet coordinates from the vehicle in-front and a distance of 40.0 units in Frenet coordinates from the vehicle behind it. If it finds another vehicle too close to our vehicle by using the sensor fusion data, it first check starts to reduce its speed in decrements if 0.35. In case the distance between the two vehicles is half of the minimum allowed value of 30.0, there is another component of hard-braking which leads to a decrement of 0.8 units to avoid a collision. Both these de-acceleration values were chosen empirically. Apart from the speed reduction, the program also tries to see if a lane change is viable. This is achieved by checking both the left and the right lanes for vehicles and by making sure that the lanes are in our driving path (Lines 288 to 303 in `main()`). If the conditions are satisifed, the boolean variable `left_lane_clear` or `right_lane_clear` is set to True. The car then makes a lane change.

**4. Path Generation: **The following steps were followed to make sure the path trajectory generated was smooth (Lines 326 to 461 in `main()`):
- A list of widely spaced (x,y) waypoints, evenly spaced at 30m is created
- Define reference states as previous path end points and use these two points that make the path tangent to the previous path's end point
- Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds. In Frenet, add evenly 30m spaced points ahead of the starting reference (Lines 363 to 367 in `main()`)
- Using the `spline()` try to create smooth curve for traversal. Calculate how to break up the spline points so that we travel at out desired reference velocity (Lines 406 to 409 in `main()`)
- Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points (Lines 412 to 430 in `main()`)

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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

### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points used so we can have a smooth transition. previous_path_x, and previous_path_y are helpful for this transition since they show the last points given to the simulator controller with the processed points already removed.

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

---

### Dependencies

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

### Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).