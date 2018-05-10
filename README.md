# MPC Control Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)
[image0]: ./docs/MPC.png

This project implements a Model Predicive Controler in C++. MPC rely on dynamic models of the process, anticipates future events and can take control actions accordingly. The main advantage of MPC over an PID controller is that it allows the current timeslot to be optimized, while keeping future timeslots in account.  

The Udacity code basis for the project can be find [here](https://github.com/udacity/CarND-MPC-Project.git). Only the `MPC.cpp`, `MPC.h` and `main.cpp` files have been modified. The goal of this project is to implement an MPC to drive the vehicle around the track in Udacity's simulator. In addition, a 100 ms latency between actuation commands on top of the connection latency have to be compensated by the controller.

### Vehicle state

The state variables for the vehicle are the following:

* `px`, `py` - The current location in the x-axis and y-axsis of an arbitrary global map coordinate system
 
* `psi` - The current orientation of the vehicle w.r.t. x-axsis
 
* `v` - The current speed of the vehicle
 
* `cte` - The cross track error which is the difference between our desired position and actual position

* `epsi` - The orientation error which is the difference between our desired heading and actual heading. 
 
### Actuator vector
The simulator provides two actuator attributes i.e. the steering angle psi (with bounds [-25°, 25°] mapped to [-1, 1]) and the throttle and break in a single attribute a (with bounds [-1, 1]). A negative value decelerates and a positive value accelerates the vehicle.

The vehicle state and the actuator vector information is provided by the simulatior every time step when the programm asks for it. In addition to this, a series of `waypoints` which are points with respect to an arbitrary global map coordinate system are provided so one can use to fit a polynomial which is a function that estimates the curve of the road ahead. A 3rd degree polynomial is a good estimate of most road curves. 

The simulator returns the vehicle orientation `psi` in radians, the vhicles' global position `x` and `y` in meter, the steering angle `steering_angle` in radians, the throttle position `throttle` and the vehicle's speed `speed` in mph. On the result figure below one ca differenciate two lines - the yellow line represents the waypoints returned by the simulator and the green lane the polynomial fitted reference path of the MPC.

![alt text][image0]

### Kinematic model
The kinematic model of the controller predicts the state vector on the next timestep considering the actual state and actuator values. The equations below describe the kinematic model used (NOTE: the model does not consider dynamics like tire forces, slip angle or slip ratio because Udacity's simulator does not support it). 

```python
x_t+1    = x_t    + v_t * cos(psi_t) * dt
y_t+1    = y_t    + v_t * sin(psi_t) * dt
psi_t+1  = psi_t  + v_t * delta_t/Lf * dt
v_t+1    = v_t    + acc_t * dt
cte_t+1  = cte_t  + v_t * sin(epsi_t) * dt
epsi_t+1 = epsi_t + v_t * delta_t/Lf * dt

Lf - this is the length from front of vehicle to its Center-of-Gravity
```
The equations lead to constraints for the optimizer, while it tries to optimize the cost function. 

### Cost function for the MPC optimizer

The MPC cost function aims at achieving the following goals:

* minimize cross-track-error `cte` in order to stay on the desired position
* minimize orientation error `epsi` in order ro be oriented tp the desired heading
* minimize deviation from reference velocity `v = 100` in odrer to keep the distance and do not stop
* minimize actuation i.e. do not seer and brake in case not needed
* minimize change in actuation in order to to prevent sudden movements.

Mathematically the cost function can be written as:

```
cost = A * cte^2 + B * epsi^2 + C * (v - vmax)^2 +
       D * delta^2 + E * a^2 + F * (a` - a)^2 +  G * (delta` - delta)^2
``` 

NOTE: The cost function is integrated over all time steps.

The weight factors (A, B, C, D, E, F, G) have been tuned manually through trial-and-error in several test runs. The higher weights for the cross track and orientation errors keeps the vehicle in the center of the lane and a lower weight for the velocity enables MPC to slow down in curves and do not drive over the curbs. At the end the following weights are used:

```cpp
const double w_cte_error_ = 3000;
const double w_epsi_error_ = 2600;
const double w_v_error_ = 1.0;
const double w_delta_ = 100; 
const double w_a_ = 10;
const double w_delta_change_ = 100;
const double w_a_change_ = 10;

```
### Timestamp and frequency

The timestep length `N` determines the MPC "lookahead" in the future and the time step frequency `dt` is how much time it is expected the environment to change. If `N` is too small, this makes the MPC too short-sighted which defeat the purpose of planning for the future. However, if `N` is too high the MPC online optimization is computationally quite complex, as for every call the program minimizes the cost function over `6 * N + 2 * (N-1)` variables. This slows down the actuator command rate leading to instability of the car. Small `dt` values reduce the distance between two points in the reference polynomial but increase the processing time at the same time which might lead to an oscillating controller because the steering and throttle commands are send too late. I started with `N = 20` and `dt = 0.05` and I noticed that that the computer was running out of time to find the best variables that have minimized the cost well. With trial-and-error I found that `N = 10` and `dt = 0.1`, which leads to a total look ahead time of 1 second, was performing well.

### Latency

As mentioned above a 100 ms latency between actuation commands on top of the connection latency have to be compensated by the controller. Unlike PID control, MPC can directly account for a latency in the kinematic model used for state prediction, thereby compensating for the latency in advance. To do so the state vector [px, py, psi, v] and the errors `cte` and `epsi` are predicted by the kinematic model 100 ms ahead (implemented in lines 147-152 of `main.cpp`). This is done before the MPC solve function is called.

### Polynomial fitting to waypoints
To estimate the current road curve ahaid a 3rd order polynomial (line 130 of `main.cpp`) is fited to the waypoints received from the simulator. Since, the waypoints are given at an arbitrary global coordinate system, one has to transform them to the vehicle's local coordinate system first before the polyfit is calculated. 

```cpp
for (size_t i = 0; i < ptsx.size(); i++) {
    double x = ptsx[i] - px;
    double y = ptsy[i] - py;
    ptsx_veh[i] = x * cos(-psi) - y * sin(-psi);
    ptsy_veh[i] = x * sin(-psi) + y * cos(-psi);
}
```

## Running the Code

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

### Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

