[model]: ./model.png "Model"

# Udacity Model Predicitve Control Project

## Overview

This is the final project for Term 2 of the Udacity Self Driving Car Engineer Nanodegree. The Udacity simulator feeds car state information to our Model Predictive Controller via websockets. The state information includes the car position and heading in map coordinates as well as velocity. In addition to the car state, the simulator passes a set of waypoints along the road / track that we should follow. Our goal is to use the car state and the waypoints to come up with a new steering input and accelerator input for the car that will allow us to follow the given waypoints.

The code used relies heavily on several mathematical libraries including Eigen, CppAD, and Ipopt amongst others.

#### The Model in Detail

This includes the state, actuators and update equations

The actual mechanics of the model used is quite complex, so I shall try to simplify as best, I can.

In its simplest case, we feed in to the model: x position, y position, heading, velocity, cross track error and heading error. Using the equations of the model below (see figure 1) you can see how we update the model for each time step (dt).

![alt text][model]
*figure 1 - Credit: Udacity*

It should be noted that the data coming from the simulator is in map coordinates, which is as you would expect (y up and x to the right). To make things easier in the computation step, the very first thing we need to do is transform all data points and readings to car coordinates (which is x positive along the car line of sight, with Y to the left). We center the axis on our car position. Once this transformation is done, it makes everything thereafter easier. As an example, the car position is now (0,0), the heading is 0. We can also more easily compute the cross track error (how much to the left or right of the simulator provided waypoints we are), and heading error (the offset between the car heading, which is zero, and the line between simulator waypoints.

To take the simplicity further, we can also more easily take latency into account by moving our car along its heading by *t* seconds to represent the system latency. This is discussed in more detail in the Latency section below.

Finally we pass all of our system state, model constraints and cost factors into a solver, which is thankfully provided in a 3rd party optimized library, which tries to compute the best solution, (driven by minimize the cost of the function) to generate the next accelerator and steering inputs.

To give an example on constraints, we can set that the acceperator must be between -1 (full brake) and +1 (full throttle) so the solver will force the accelerator values into that range. I also set the steering between -25 and +25 degrees as suggested in the Udacity lesson.

I played with different cost penalties - factors that increase the penalty on the algorithm for touching various factors. I set a penalty on the use of accelerator to try to discourage the algorithm to touch that, and a very heavy penalty on big changes in steering angle from update step to update step. I found that this latter penalty had a very powerful effect and smoothed out the steering inputs dramatically. I also set a penalty on the speed of the vehicle deviating from 30mph to keep the car moving at a sensible speed.


#### Selecting *N* and *dt* Values

My approach to selecting N (timestep length) and dt (elapsed duration between timesteps) values was to start by using the guidance from the Udacity material that the car should only be predicting state a few seconds into the future, at most. The combination of dt and N equate to how far into the future we look. As an example, dt of 1.0 and N of 5 predicts 5 seconds into the future. The larger dt is, the larger the spacing in time between prediction states and hence we lose granuality, this is especially important around corners. 

I chose to back into the problem using the Udacity course direction, so I set the amount of time I wanted to look into the future as 3 seconds ("a few seconds at most"). Next I set dt as the spacing in time between update steps. I set N by deriving it using the equation:

*N = Time Horizon / dt*

I pretty much started and ended with 3 seconds as being my Time Horizon, and played with dt between 0.1 and slowly increased it until I started getting unstable readings. I landed on 0.4, which makes *N = 3 / 0.4 = 7*.

The result was that I had great tracking around the circuit at ~30mph. I found that increasing the speed caused problems. If I had more time I would experiment with changing the Time Horizon and dt and N values while increasing speed. It could be that changes in target speed should alter these values. It should be noted that a lower value of N is preferable as it means less computation is required as all algorithms I used were running in real-time.

#### Dealing with Latency

An issue with any real-time system is latency. That is the time it takes from when a reading is taken, to calculate a response, and then to make that response happen. Typically, in computer software latency is pretty minimal, often even negligable. When we dealing with controlling mechanical objects (like a car) latency goes up significantly. To try to emulate latency in a car, it was stated that when we issue a command, say to turn the wheel, it will take 100ms for that change to happen. A delay was put in the simulator to emulate this latency. The way that I chose to try and tackle the issue in my model was to take the current velocity of the car, changing the speed from mph to meters per sec, and then increment my y position (in car coords, that is the direction that I am driving) by time delta (100ms or 0.1 seconds) multiplied by my speed. I then used this new y position to calculate my cross track error at this position, and ran the update step on my model using this position. 

The question I was trying to answer using this approach was... if I was 100ms in the future on this heading and at my current speed, then how should I compute my steering and accelerator commands? The reason I tried this approach is because by the time a command takes effect (taking latency into account) it really will be 100ms from now, so we should plan that whatever command we issue will be when the car is in its position current time + latency time state.