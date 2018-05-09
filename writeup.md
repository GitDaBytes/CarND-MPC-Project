# Udacity Model Predicitve Control Project

## Overview

This is the final project for Term 2 of the Udacity Self Driving Car Engineer Nanodegree. The Udacity simulator feeds car state information to our Model Predictive Controller via websockets. The state information includes the car position and heading in map coordinates as well as velocity. In addition to the car state, the simulator passes a set of waypoints along the road / track that we should follow. Our goal is to use the car state and the waypoints to come up with a new steering input and accelerator input for the car that will allow us to follow the given waypoints.

### The model in detail... This includes the state, actuators and update equations

### reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values



### Dealing with Latency

An issue with any real-time system is latency. That is the time it takes from when a reading is taken, to calculate a response, and then to make that response happen. Typically, in computer software latency is pretty minimal, often even negligable. When we dealing with controlling mechanical objects (like a car) latency goes up significantly. To try show latency in a car, it was stated that when we issue a command, say to turn the wheel, it will take 100ms for that change to happen. A delay was put in the simulator to emulate this latency. The way that I chose to try and tackle the issue in my model was to take the current velocity of the car, changing the speed from mph to meters per sec, and then increment my y position (in car coords, that is the direction that I am driving) by time delta (100ms or 0.1 seconds) multiplied by my speed. I then used this new y position to calculate my cross track error at this position, and ran the update step on my model using this position. 

The goal was trying to achieve using this approach was... if I was 100ms in the future on this heading and at my current speed, then how should I compute my steering and accelerator commands. The reason I tried this approach is because by the time a command does get made (taking latency into account) it really be 100ms from now, so we should plan that whatever command we issue will be when the car is in its position current time + latency time state.