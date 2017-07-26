# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## The Model 

1. The MPC model uses the current state of the vehicle [x, y, psi, v] and the path trajectory to predict the future of the vehicle by finding the optimal controller (actuators) [a, delta] to minimize the error.  
2. The error we want to minimize is described in the cost function which includes:
 * Cross-track-error between the predicted location and the path trajectory 
 * Orientation error 
3. The cost function also includes:
 * The difference between the vehicle predicted speed and the minimum speed
 * Acceleration 
 * Acceleration Change
 * Orientation 
 * Orientation Change
4. The MPC model will send the steer_value and throttle_value limited with the actuator constraints back to the simulator with a 100 milliseconds delay. 

## Timestep Length and Elapsed Duration (N & dt)

1. The N = 20 and dt = 0.02 and the duration was 0.4 seconds.  The reason i chose these numbers was that I didn't want the optimization function to optimize for steps too far in the path.  Since the trajectory will be updated every time, there was no need to optimize for a long path.  I would rather the optimization function optimize more accurately for the immediate steps within (0.4 seconds). 

2. I tried larger N and same dt and the duration was longer.  As mentioned in #1, when the duration was longer, the optimization function seemed to optimize more for future steps rather than immediate steps. 

3. I also tried to kept the duration and make dt shorter, I didn't find much benefits by doing that. 

## Polynomial Fitting and MPC Preprocessing

1. The waypoints were preprocessed to change from the map coordinate to the vehicle coordinate. 

2. The initial state of the vehicle is [v * 0.1, y, psi, v] where the x value is velocity * 100 millisecond. 

## Model Predictive Control with Latency

The way i deal with the latency is by setting the initial state of the vehcile to velocity * 100 millisecond which will have the optimization function to start optimizing after the first 100 milliseconds to take into account that the car will continue for 100 milliseconds with the current speed before the actuators are applied. 

## Simulator Result

<a href="https://youtu.be/2q3c0MhPKyU" target="_blank"><img src="http://img.youtube.com/vi/2q3c0MhPKyU/0.jpg" alt="PID Controller"/></a>

