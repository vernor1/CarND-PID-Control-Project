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

The base algorithm follows what's presented in the lessons.

---
### Reflection
#### 1. Describe the effect each of the P, I, D components had in your implementation.

Proportional (P) component of the PID controller only takes into account the current cross-track error (CTE) and returns the error value propertional to CTE. It's not enough for a reliable steering control, since a proportional-only controller tends to oscilate around the ideal trajectory.



Differencial (D) component ...



Integral (I) ...

P-only | D-only | PD
:---:|:---:|:---:
[![](gif/p-only.gif)](https://youtu.be/E-wOO8_RmYo "P-only, click to see the full footage") | [![](gif/d-only.gif)](https://youtu.be/VhSkjgRqJFo "D-only, click to see the full footage") | [![](gif/pd.gif)](https://youtu.be/uOMmduFpcH8 "PD, click to see the full footage")
