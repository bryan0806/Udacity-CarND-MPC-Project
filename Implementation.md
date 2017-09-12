# Implementation


## The Model :Student describes their model in detail. This includes the state, actuators and update equations.
### the state:
  
### actuators:

### update equations:



## Timestep Length and Elapsed Duration (N & dt): Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
>**I first chose N=15 dt = 0.15, because I saw on the discussion forum that dt must be bigger than latency. However, I found that if dt=0.15, sometimes it would be difficult to turn without touching the curbs for some hard turn. So I finaly change it to N=10 dt=0.1.**

## Polynomial Fitting and MPC Preprocessing :A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

## Model Predictive Control with Latency: The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
