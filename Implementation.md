# Implementation


## The Model :Student describes their model in detail. This includes the state, actuators and update equations.
### update equations of the state:
```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
```
### update equations of actuators:
```
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```



## Timestep Length and Elapsed Duration (N & dt): Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
>**I first chose N=15 dt = 0.15, because I saw on the discussion forum that dt must be bigger than latency. However, I found that if dt=0.15, sometimes it would be difficult to turn without touching the curbs for some hard turn. Since I can find a way to deal with latency later, I finaly change it to N=10 dt=0.1.**

## Polynomial Fitting and MPC Preprocessing :A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
> I did transfer the waypoints to car coodinate before sending them to polyfit. Below is my code to transfer:
```
 // Use Eigen vector as polyfit() requires it
 Eigen::VectorXd   x_car_space = Eigen::VectorXd( ptsx.size() ) ;
 Eigen::VectorXd   y_car_space = Eigen::VectorXd( ptsx.size() ) ;

  for (int i = 0;   i < ptsx.size() ;   i++) {
    x_car_space(i) = (ptsx[i] - px) * cos(psi) + (ptsy[i] - py) * sin(psi)  ;
    y_car_space(i) = (ptsy[i] - py) * cos(psi) - (ptsx[i] - px) * sin(psi)  ;

    }
```

## Model Predictive Control with Latency: The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
> Below is my code to deal with latency, so I can make sure the state is latest before sending them to *_MPC::Solve_* function
```
double latency = 0.1;
double Lf = 2.67;
v *= 0.44704; // convert from mph to m/s
px = 0 + v * cos(steer_value) * latency; // px: px0 = 0, due to the car coordinate system
py = 0 + v * sin(steer_value) * latency; // py: py0 = 0, due to the car coordinate system
psi = - v / Lf * steer_value * latency; // psi: psi0 = 0, due to the car coordinate system
double cte = polyeval(coeffs, px) - 0; // since py0=0
double epsi = atan(coeffs[1]+2*coeffs[2]*px + 3*coeffs[3]*px*px);

```
