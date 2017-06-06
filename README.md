# PID Controller Project (Term 2 Project 4)
This is my submission for the PID Controller project in term 2 of the Udacity Self-Driving Car Engineer Nanodegree Program. For details of the projectm, refer to the source [repo](https://github.com/udacity/CarND-PID-Control-Project).

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

---

## Implementation

The PID controller implementation is following the basic algorithm presented in the lessons. For each frame, the simulator returns a value for the cross-track error (CTE), which measures how much the car deviates from the center of the lane. From the CTE (`p_cte`), two derived errors were calculated - the difference between the current and last CTE (`d_cte`) and the accumulated CTE (`i_cte`). From these three errors, the steering value is calculated from a set of weights as follows (PID.cpp line 46):

```steer = -Kp* p_error -Kd*d_error -Ki*i_error```

By removing one or two of these error terms, we can also get the Proportional-only controller (PID.cpp line 44) or the Proportional+Derivative Controller (PID.cpp line 45).

## Twiddle
The three weights `Kp, Ki, Kd` are tuned using the twiddle algorithm presented in class (main.cpp lines 86-206). To turn Twiddle On, please uncomment the first line in main.cpp (`#define _TWIDDLE_ON_`) before compiling to activate the preprocessor flag.

Here is one set of typical twiddle iterations (with exit threshold set at `sum(dP)<0.15`):
```
---------- Twiddle Starts -------------
Twiddle Exit1 Frame#501 bestcte=0.311179 avgcte=0.311179 P2: Kp=0.2 Ki=0.004 Kd=3.1 dP: dKp=0.1 dKi=0.1 dKd=0.11
Twiddle Exit2 Frame#501 bestcte=0.311179 avgcte=0.322657 P0: Kp=0.1 Ki=0.004 Kd=3.1 dP: dKp=0.1 dKi=0.1 dKd=0.11
Twiddle Exit3 Frame#501 bestcte=0.311179 avgcte=0.554895 P0: Kp=0.2 Ki=0.004 Kd=3.1 dP: dKp=0.09 dKi=0.1 dKd=0.11
Twiddle Exit2 Frame#501 bestcte=0.311179 avgcte=0.338961 P1: Kp=0.2 Ki=0 Kd=3.1 dP: dKp=0.09 dKi=0.1 dKd=0.11
Twiddle Exit3 Frame#501 bestcte=0.311179 avgcte=0.338026 P1: Kp=0.2 Ki=0.1 Kd=3.1 dP: dKp=0.09 dKi=0.09 dKd=0.11
Twiddle Exit1 Frame#501 bestcte=0.233474 avgcte=0.233474 P2: Kp=0.2 Ki=0.1 Kd=3.1 dP: dKp=0.09 dKi=0.09 dKd=0.121
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.244433 P0: Kp=0.11 Ki=0.1 Kd=3.1 dP: dKp=0.09 dKi=0.09 dKd=0.121
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.300936 P0: Kp=0.2 Ki=0.1 Kd=3.1 dP: dKp=0.081 dKi=0.09 dKd=0.121
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.236515 P1: Kp=0.2 Ki=0 Kd=3.1 dP: dKp=0.081 dKi=0.09 dKd=0.121
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.348195 P1: Kp=0.2 Ki=0.09 Kd=3.1 dP: dKp=0.081 dKi=0.081 dKd=0.121
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.278353 P2: Kp=0.2 Ki=0.09 Kd=2.858 dP: dKp=0.081 dKi=0.081 dKd=0.121
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.251906 P2: Kp=0.2 Ki=0.09 Kd=2.979 dP: dKp=0.081 dKi=0.081 dKd=0.1089
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.254498 P0: Kp=0.038 Ki=0.09 Kd=2.979 dP: dKp=0.081 dKi=0.081 dKd=0.1089
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.487975 P0: Kp=0.119 Ki=0.09 Kd=2.979 dP: dKp=0.0729 dKi=0.081 dKd=0.1089
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.34258 P1: Kp=0.119 Ki=0 Kd=2.979 dP: dKp=0.0729 dKi=0.081 dKd=0.1089
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.530335 P1: Kp=0.119 Ki=0.081 Kd=2.979 dP: dKp=0.0729 dKi=0.0729 dKd=0.1089
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.339319 P2: Kp=0.119 Ki=0.081 Kd=2.7612 dP: dKp=0.0729 dKi=0.0729 dKd=0.1089
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.336472 P2: Kp=0.119 Ki=0.081 Kd=2.8701 dP: dKp=0.0729 dKi=0.0729 dKd=0.09801
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.327209 P0: Kp=0 Ki=0.081 Kd=2.8701 dP: dKp=0.0729 dKi=0.0729 dKd=0.09801
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.71224 P0: Kp=0.0729 Ki=0.081 Kd=2.8701 dP: dKp=0.06561 dKi=0.0729 dKd=0.09801
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.419723 P1: Kp=0.0729 Ki=0 Kd=2.8701 dP: dKp=0.06561 dKi=0.0729 dKd=0.09801
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.709544 P1: Kp=0.0729 Ki=0.0729 Kd=2.8701 dP: dKp=0.06561 dKi=0.06561 dKd=0.09801
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.429548 P2: Kp=0.0729 Ki=0.0729 Kd=2.67408 dP: dKp=0.06561 dKi=0.06561 dKd=0.09801
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.424712 P2: Kp=0.0729 Ki=0.0729 Kd=2.77209 dP: dKp=0.06561 dKi=0.06561 dKd=0.088209
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.423774 P0: Kp=0 Ki=0.0729 Kd=2.77209 dP: dKp=0.06561 dKi=0.06561 dKd=0.088209
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.829374 P0: Kp=0.06561 Ki=0.0729 Kd=2.77209 dP: dKp=0.059049 dKi=0.06561 dKd=0.088209
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.452919 P1: Kp=0.06561 Ki=0 Kd=2.77209 dP: dKp=0.059049 dKi=0.06561 dKd=0.088209
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.807099 P1: Kp=0.06561 Ki=0.06561 Kd=2.77209 dP: dKp=0.059049 dKi=0.059049 dKd=0.088209
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.438551 P2: Kp=0.06561 Ki=0.06561 Kd=2.59567 dP: dKp=0.059049 dKi=0.059049 dKd=0.088209
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.482241 P2: Kp=0.06561 Ki=0.06561 Kd=2.68388 dP: dKp=0.059049 dKi=0.059049 dKd=0.0793881
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.472386 P0: Kp=0 Ki=0.06561 Kd=2.68388 dP: dKp=0.059049 dKi=0.059049 dKd=0.0793881
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.805451 P0: Kp=0.059049 Ki=0.06561 Kd=2.68388 dP: dKp=0.0531441 dKi=0.059049 dKd=0.0793881
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.496799 P1: Kp=0.059049 Ki=0 Kd=2.68388 dP: dKp=0.0531441 dKi=0.059049 dKd=0.0793881
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.900397 P1: Kp=0.059049 Ki=0.059049 Kd=2.68388 dP: dKp=0.0531441 dKi=0.0531441 dKd=0.0793881
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.539338 P2: Kp=0.059049 Ki=0.059049 Kd=2.5251 dP: dKp=0.0531441 dKi=0.0531441 dKd=0.0793881
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.506569 P2: Kp=0.059049 Ki=0.059049 Kd=2.60449 dP: dKp=0.0531441 dKi=0.0531441 dKd=0.0714493
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.516504 P0: Kp=0 Ki=0.059049 Kd=2.60449 dP: dKp=0.0531441 dKi=0.0531441 dKd=0.0714493
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.918568 P0: Kp=0.0531441 Ki=0.059049 Kd=2.60449 dP: dKp=0.0478297 dKi=0.0531441 dKd=0.0714493
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.459296 P1: Kp=0.0531441 Ki=0 Kd=2.60449 dP: dKp=0.0478297 dKi=0.0531441 dKd=0.0714493
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.975498 P1: Kp=0.0531441 Ki=0.0531441 Kd=2.60449 dP: dKp=0.0478297 dKi=0.0478297 dKd=0.0714493
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.560656 P2: Kp=0.0531441 Ki=0.0531441 Kd=2.46159 dP: dKp=0.0478297 dKi=0.0478297 dKd=0.0714493
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.594252 P2: Kp=0.0531441 Ki=0.0531441 Kd=2.53304 dP: dKp=0.0478297 dKi=0.0478297 dKd=0.0643044
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.555556 P0: Kp=0 Ki=0.0531441 Kd=2.53304 dP: dKp=0.0478297 dKi=0.0478297 dKd=0.0643044
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.982611 P0: Kp=0.0478297 Ki=0.0531441 Kd=2.53304 dP: dKp=0.0430467 dKi=0.0478297 dKd=0.0643044
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.537049 P1: Kp=0.0478297 Ki=0 Kd=2.53304 dP: dKp=0.0430467 dKi=0.0478297 dKd=0.0643044
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=1.06148 P1: Kp=0.0478297 Ki=0.0478297 Kd=2.53304 dP: dKp=0.0430467 dKi=0.0430467 dKd=0.0643044
Twiddle Exit2 Frame#501 bestcte=0.233474 avgcte=0.622 P2: Kp=0.0478297 Ki=0.0478297 Kd=2.40443 dP: dKp=0.0430467 dKi=0.0430467 dKd=0.0643044
Twiddle Exit3 Frame#501 bestcte=0.233474 avgcte=0.642849 P2: Kp=0.0478297 Ki=0.0478297 Kd=2.46874 dP: dKp=0.0430467 dKi=0.0430467 dKd=0.0578739
 ------- Twiddle Complete -------- 
Using Params: Kp=0.0478297 Ki=0.0478297 Kd=2.46874
 --------------------------------- 
``` 

## Future extension

For future extension, we can also add another Proportional-only controller for throttle control, where the error can be set as the difference between the actual speed and the desired speed of the vehicle. The proportional weight for this throttle control will be tuned together with the other three parameters in Twiddle, i.e. tune 4 instead of 3 parameters.

## Reflection

Each of the `P`, `I` and `D` controllers aims to solve a difference problem with control. The Proportional (`P`) part of the controller tries to maintain the car at the center of the lane by adjusting the CTE towards zero, the Derivative (`D`) part of the controller tries to minimize oscillation around the desired setting, while the Integral (`I`) part corrects for any pre-existing bias in the system, e.g. uncorrected steering bias common in cars.

I have recorded three separate videos for the same beginning section of the track to illustrate these effect.

[Proportional-only](https://youtu.be/IUFvup9EtGQ)
https://youtu.be/IUFvup9EtGQ

[Proportional+Derivative](https://youtu.be/-1b7-WD318s)
https://youtu.be/-1b7-WD318s

[PID Controller](https://youtu.be/HXOWdayu5kY)
https://youtu.be/HXOWdayu5kY

As observed in the videos, the Proportional-only controller oscillates too much and the car could not make it onto the bridge section of the track. With Derivative control added, the oscillation is now slightly more controlled and the car is able to complete the track circuit. Since the car does not has any inherent bias, there is only minimal impact when the Integral control is added as observed in the third video.

## Conclusion

This project demonstrates the application of a traditional PID controller in a self-driving car application. As long as the car sensors return accurate and timely cross track error measurements, it is possible to control a car to drive itself along a well-defined lane. However, the control is based on tracking the CTE and is not as smooth (i.e. with a lot of oscillations) as a deep-learning based behavior cloning approach, e.g. in the previous [Behavioral Cloning Project](https://github.com/lowspin/CarND-Proj03-Behavioral-Cloning).



