# PID Control Project

The goals / steps of this project are the following:

* Complete the PID control algorithm in C++.
* Ensure that the project compiles.
* Test the PID control with the simulator.
* Check for divide by zero.

## [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Compilation
#### 1. Your code should compile.

The code compiles without errors with cmake-3.8.1 and make-3.81 on macOS-10.12.5.

---
### Implementation
#### 1. The PID procedure follows what was taught in the lessons.

The base algorithm follows what's presented in the lessons. The code structure is:
* `src/main.cpp`: Implements the control server for the simulator. Instantiates `PidController`, which does the actual steering and throttle control.
* `src/PidController.h` and `src/PidController.cpp`: Class `PidController` aggregates an instance of `Pid`, which implements the PID control. Also aggregates and instance of `Twiddler` for finding optional PID coefficients. Uses the error returned by `Pid`, normalizes it within -1..1, and applies it as the steering value. The throttle control is computed as normalized value `1 - 2 * (Speed / MaxSpeed) * (abs(CTE) / SafeCTE)`, where `MaxSpeed` is the maximum car speed at throttle=1 (100mph), `SafeCTE` is the safe CTE value (chosen at 60% of off-track CTE).
* `src/Pid.h` and `src/Pid.cpp`: Class `Pid` implements the PID control.
* `src/Twiddler.h` and `src/Twiddler.cpp`: Class `Twiddler` implements the Twiddle algorithm.
* `test/TestPidController.cpp`: Tests class `PidController`.
* `test/TestPid.cpp`: Tests class `Pid`.
* `test/TestTwiddler.cpp`: Tests class `Twiddler`
* `test/Robot.h`: Implements a basic robot for unit-tests.

The executable binary supports command-line parameters to toggle the modes - free driving using default or provided PID coefficients, or finding optimal PID coefficients using the Twiddle algorithm:
```
Usage instructions: ./pid [Kp Ki Kd offTrackCte] [dKp dKi dKd trackLength]
  Kp          Proportional coefficient
  Ki          Integral coefficient
  Kd          Differential coefficient
  offTrackCte Approximate CTE when getting off track
  dKp         Delta of Kp
  dKi         Delta of Ki
  dKd         Delta of Kd
  trackLength Approximate track length in meters
If no arguments provided, the default values are used: Kp=0.12, Ki=1e-05, Kd=4, offTrackCte=5.
If only [Kp Ki Kd] are provided, the PID controller uses those values.
If [dKp dKi dKd trackLength] are also provided, the PID controller finds best coefficients using the Twiddle algorithm, and uses them.
```

---
### Reflection
#### 1. Describe the effect each of the P, I, D components had in your implementation.

* **Proportional (P)** component of the PID controller only takes into account the current cross-track error (CTE) and produces the error value proportional to CTE. It's not enough for a reliable steering control, since a proportional-only controller tends to oscillate around the ideal trajectory. See the P-only recording below.
* **Differential (D)** component accounts for the rate of CTE change, thus able to handle rapid trajectory changes as well as compensate for frequent oscillations caused by the P-component. See the D-only recording below.
* **Integral (I)** component takes into account the accumulated past CTE, responding to a constant added error. It's able to handle the case of a constant drift, but for the cost of a high added initial error, while the accumulated CTE is small. The I-component is quite useless in this project since the simulator seems to implement a perfect steering. There's no difference in full PID and PD driving observed in simulation (see the corresponding recordings below).

P-only | D-only | PD
:---:|:---:|:---:
[![](gif/p-only.gif)](https://youtu.be/E-wOO8_RmYo "P-only, click to see the full footage") | [![](gif/d-only.gif)](https://youtu.be/VhSkjgRqJFo "D-only, click to see the full footage") | [![](gif/pd.gif)](https://youtu.be/uOMmduFpcH8 "PD, click to see the full footage")

#### 2. Describe how the final hyperparameters were chosen.

Final PID coefficients are Kp=0.12, Ki=1e-05, Kd=4, which were found by using the Twiddle algorithm. The error value for Twiddle is computed in the following way:
* If the vehicle is getting off-track before completing the first lap, the error value is inversely proportional to the driven distance, penalized by 1e+6 in order to make it consistently larger than any Twiddle error when the lap is completed safely.
* If the vehicle completes the first lap, the error is computed as the maximum CTE observed during the lap (to penalize dangerous cornering), multiplied by the average CTE (to penalize winding steering).

A recording of finding optimal PID coefficients (click to see the full footage):
[![](gif/twiddler.gif)](https://youtu.be/X2BjdL26SXw "Twiddler")

---
### Simulation
#### 1. The vehicle must successfully drive a lap around the track.

A recording of the final PID control (click to see the full footage):

PID
:---:|
[![](gif/pid.gif)](https://youtu.be/KtHdqrzXhTw "PID")

---
### Notes

* The code is complying with the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
* All function/method comments are made Doxygen-friendly.
* The [Google Test](https://github.com/google/googletest) framework is used for unit-testing the code. To build and run the tests, enter the command in the build directory:
```
$ cmake -Dtest=ON .. && make && make test
-- Configuring done
-- Generating done
-- Build files have been written to: /Users/vernor/Documents/carnd/carnd_pid_control/build
[  8%] Built target pid_controller_lib
[ 16%] Built target pid_lib
[ 24%] Built target twiddler_lib
[ 56%] Built target gtest
[ 64%] Built target test_pid_controller
[ 72%] Built target test_pid
[ 80%] Built target test_twiddler
[100%] Built target pid
Running tests...
Test project /Users/vernor/Documents/carnd/carnd_pid_control/build
    Start 1: test_twiddler
1/3 Test #1: test_twiddler ....................   Passed    0.82 sec
    Start 2: test_pid
2/3 Test #2: test_pid .........................   Passed    0.02 sec
    Start 3: test_pid_controller
3/3 Test #3: test_pid_controller ..............   Passed    0.02 sec

100% tests passed, 0 tests failed out of 3

Total Test time (real) =   0.87 sec
```
