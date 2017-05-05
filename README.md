# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

### Overview
The goals / steps of this project are the following:  

* Complete PID controller in C++.
* Tune PID controller parameters with twiddle algorithm
* Test PID controller on the simulator and make sure the vehicle is able to drive successfully around the track.

### Final Result

[Here](https://youtu.be/mHUyWUgT7bM) is the video that demonstrates the vehicle controlled by PID successfully drives around the track in the simulator.



## PID components

In this project, PID controller is used to derive appropriate steering angle that controls the car driving. The input to PID controller is Cross Track Error, which specifies how far away the car is away from the intended driving route (center of the road). In essence, it's a simple one line equation as below:  

steering_angle = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte 

tau_p * cte : Proportional Controller  
tau_i * int_cte:  Integral Controller  
tau_d * diff_cte: Differential Controller  

The purpose of P controller is to aggressively reduce car's next cross track error. Only using P controller is not enough for effective car driving, the car will overshoot and drive off track very quickly, as evidenced in my experiment of setting D controller coefficient as zero.

The purpose of D controller is to reduce the oscillation problems in P controller. With D Controller added, we can see overshooting problem is visibly reduced.

The purpose of I controller is to eliminate system bias in simulator. In this project, the final I controller coefficient found by twiddle is zero. I think this is mainly because the system bias is small. Another reason could be that the twiddle algorithm never really converge at the end, in the sense that int_cte is always increasing during twiddle fine tunig.


## PID  hyperparameter tuning

Twiddle is used to tune parameters. Technically, at the start of the program, we lanuch a separate worker thread that waits on cte signal from simulator and perform twiddle algorithm. Twiddle algorithm is implemented in twiddle.cpp file.


Below is an example of the search process of twiddle. From it, we can see that twiddle did a farily good job of reducing total error and thus finding optimal PID controller hyper paramters.  

start fine tuning PID gains  
Listening to port 4567  
improvement 0,124  
cycle 0, best error = 26027.2  
p: [0, 0, 0]  
dp:[1, 1, 1]  
improvement 124,272  
improvement 272,391  
cycle 1, best error = 23547.4  
p: [1, 0, 1]  
dp:[1.2, 0.9, 1.2]  
improvement 391,644  
cycle 2, best error = 21301.8  
p: [1, 0, 2.2]  
dp:[1.08, 0.81, 1.44]  
improvement 644,1557  
cycle 3, best error = 13127.3  
p: [1, 0, 3.64]  
dp:[0.972, 0.729, 1.728]  
improvement 1557,1615  
cycle 4, best error = 12661.3  
p: [1, 0, 5.368]  
dp:[0.8748, 0.6561, 2.0736]  
improvement 1615,2893  
cycle 5, best error = 1179.78  
p: [1, 0, 7.4416]  
dp:[0.78732, 0.59049, 2.48832]  
improvement 2893,3001  
cycle 6, best error = 160.584  
p: [1, 0, 9.92992]  
dp:[0.708588, 0.531441, 2.98598]  
cycle 7, best error = 160.584  
p: [1, 0, 9.92992]  
dp:[0.637729, 0.478297, 2.68739]  
cycle 8, best error = 160.584  
p: [1, 0, 9.92992]  
dp:[0.573956, 0.430467, 2.41865]  
cycle 9, best error = 125.863  
p: [1.57396, 0, 9.92992]  
dp:[0.688748, 0.38742, 2.17678]  
cycle 10, best error = 125.863  
p: [1.57396, 0, 9.92992]  
dp:[0.619873, 0.348678, 1.9591]  



## Reflection

This is a fun project in that we can see how essentially a one line equation can help us smoothly drive the car in the simulator. On the other side, there are known problems in current approach and might require future work if we are to perfect the solution.

1. Twiddle fine tuning  
Twiddle is great in that it allows us to tune hyper parameters for a system without knowing much inner working the system. The downside is that we might very well land on sub-optimal parameters. As a result, it's important to experiment different learning step strategy in order to find optimal parameters.
2. Vehicle speed  
Currently we are using a fixed and relatively low speed. Maybe a PID controller for throttle could be a good idea to increase speed.


Lastly, at one point, my twiddle find me a combination of PID parameters ([32.7713, 0, 74.8867]) that can drive perfectly in the track, though the steering angle it outputs are mostly 1 or -1.  It's a nice surprise to see how the "clever" twiddle comes up with answers we never initially expect. Just for fun, [here](https://youtu.be/PIkNhLrCuQA) the video that records the funny movement of the vehicle with these PID parameters.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

