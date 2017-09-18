## MPC for Autonomous Driving

Goal: Implementation  of a software pipeline using the model predictive control (MPC) method to drive a car around a track in a simulator. The simulator provides a feed of position, speed and heading direction of the car. It also provides the coordinates (in a global coordinate system) of waypoints along a reference trajectory that the car is to follow. Note that, there is 100 millisecond latency between actuation commands apart from the connection latency.


## Model Overview

We use MPC (Model Predictive Control) in the this project to build a stable controller. Goal is to control the steering, velocity and throttle of the car to promise an optimised predicted trajectory.

We model the state of the vehicle and predict the state based on actuators. This is achieved by using an optimizer to find control inputs with the goal of minimizing a cost function.

### State

1. `px`: x-axis vehicle position
2. `py`: y-axis vehicle position
3. `psi`: heading of vehicle
4. `v`: speed of vehicle
5. `cte`: cross track error (offset of vehicle from center of road)
6. `epsi`: heading error (diff between ideal and actual heading/direction)

### Actuators

1. Steering
2. Throttle - this combines acceleration and braking into one value

### Update

Update equations for the model are following:

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t-1] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t-1] / Lf * dt
```

### Timestep Length and Elapsed Duration

* `N` = 10 (number of predictions to make into the future, as large as possible)
* `dt` = 0.1 (the time gap between predictions, as small as possible)

* *Fixing `N`*:  Too large values of N hinders computational efficiency but it should large enough to make accurate plan further ahead. For example, a value of 5 causes the car to be unstable. However,  a large value like 15,  results in low computation frequency, causing the car to be unstable.
* *Fixing `dt`*: `dt` should be small enough to allow accurate computation. But it should be large enough to allow planning further ahead. A dt value of 0.1 matches 100ms delay that we have for actuators, hance making it easy to account for the delay.


### Polynomial Fitting and MPC Preprocessing

We need to transform the incoming data from the simulator.
* Reference trajectory - `x`
* Reference trajectory - `y`
* Vehicle position - `x`
* Vehicle position - `y`
* Heading of the vehicle - `psi`
* Speed of the vehicle - `v`
* Steering
* Throttle

1. *Transformation of waypoint coordinates* to global co-ordinate system:

```
ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi))
ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi))
```

2. *Find cross track error* by fitting the transformed coordinates into third degree polynomial:

```
fg[2 + x_start + i]    = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[2 + y_start + i]    = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[2 + psi_start + i]  = psi1 - (psi0 + v0 * delta0 / Lf * dt);
fg[2 + v_start + i]    = v1 - (v0 + a0 * dt);
fg[2 + cte_start + i]  = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);
fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```


### Model Predictive Control with Latency

Since we have to accommodate 100ms latency, instead of directly using the state as it is, we compute the state with the delay factored in using our kinematic model. Then we feed it to the object that solves for what should be done next.

```
double px_t1 = 0.0 + v * cos (0.0) * latency;
double py_t1 = 0.0 + v * sin (0.0) * latency;
double psi_t1 = 0.0 + (v / Lf) * steer_value * latency;
double v_t1 = v + throttle_value * latency;
double cte_t1 = cte + v * sin(epsi) * latency;
double epsi_t1 = epsi + (v / Lf) * steer_value * latency;
Eigen::VectorXd x0(6);
x0 << px_t1, py_t1, psi_t1, v_t1, cte_t1, epsi_t1;

vector<double> solution = mpc.Solve(x0, coeff);
```


