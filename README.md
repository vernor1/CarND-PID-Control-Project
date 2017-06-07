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

---
### Reflection
#### 1. Describe the effect each of the P, I, D components had in your implementation.

* **Proportional (P)** component of the PID controller only takes into account the current cross-track error (CTE) and produces the error value proportional to CTE. It's not enough for a reliable steering control, since a proportional-only controller tends to oscilate around the ideal trajectory. See the P-only recording below.
* **Differencial (D)** component accounts for the rate of CTE change, thus able to handle rapid trajectory changes as well as compensate for frequent oscillations caused by the P-component. See the D-only recording below.
* **Integral (I)** component takes into account the accumulated past CTE, respondidng to a constant added error. It's able to handle the case of a constant drift, but for the cost of a high added initial error, while the accumulated CTE is small. The I-component is quite useless in this project since the simulator seems to implement a perfect steering. There's no difference in full PID and PD driving observed in sumulation (see the corresponding recordings below).

P-only | D-only | PD
:---:|:---:|:---:
[![](gif/p-only.gif)](https://youtu.be/E-wOO8_RmYo "P-only, click to see the full footage") | [![](gif/d-only.gif)](https://youtu.be/VhSkjgRqJFo "D-only, click to see the full footage") | [![](gif/pd.gif)](https://youtu.be/uOMmduFpcH8 "PD, click to see the full footage")
