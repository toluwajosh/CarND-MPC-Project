# Model Predictive Control Poject
Udacity Self-Driving Car Engineer Nanodegree Program

Term 2, Project 5

---

**The objective of this project is to implement a Model Predictive Control (MPC) to drive a car around a simulated track.**
In the model, the cross track error is calculated and a latency of 100 milliseconds is accounted for.

## 1.0 Implementation
### 1.1 The model:
In other to drive the car around we need to know the state of the car, the actions we need to perform and lastly the outcome of our actions. In addition we also have a reference trajectory which we desire to follow. MPC uses the state of the car and the errors between the desired and the reference trajectory to predict an optimal trajectory by simulating different actuator inputs and then selecting a resulting trajectory with the minimum cost.

Our state variables are:
* `x` and `y` the position of the car,
* `psi` the orientation of the car, and
* `v` the velocity of the car.
Our actuations are
* `delta` the steering angle, and
* `a` the acceleration.

The following are the steps followed in implementing the MPC:

1. First, a third degree polynomial is fitted to waypoints recieved from the simulator and the cross track error (`cte`) is obtained by evaluating the polynomial at current `x` position, and the orientation error `epsi`.(See: [main.cpp](https://github.com/toluwajosh/CarND-MPC-Project/blob/debug_and_finish/src/main.cpp));
```cpp
          // change reference position into car's coordinate
          for (unsigned int i = 0; i < ptsx.size(); ++i)
          {
              double shift_x = ptsx[i]-px;
              double shift_y = ptsy[i]-py;

              ptsx[i] = (shift_x*cos(0-psi) - shift_y*sin(0-psi));
              ptsy[i] = (shift_x*sin(0-psi) + shift_y*cos(0-psi));
          }

          // convert points to VecotrXD for use in polyfit function
          double* ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);position

          double* ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

          // calculate coefficients for line fit
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          // The distance between the car and the center of the road
          double cte =  polyeval(coeffs, 0);
          // double epsi = psi - atan(coeffs[1] + 2 *px*coeffs[2] + 3 *coeffs[3]*pow(px,2))
          double epsi = -atan(coeffs[1]); // a simplification, since psi=0 and px=0
```
*Note that the recieved waypoints were transformed into the car's coordinate to allow for easier subsequent calculations.*

The variables are then updated using the global kinematic model given by:

<p align="center">
<img src="/media/global_kinematic_model_eqns.png" alt="Global Kinematic Model Equations" width="300" align="middle">
</p>

as in;
```cpp
          double delta = j[1]["steering_angle"];position
          double a = j[1]["throttle"];
          double dt = 0.1; // accounting for 100ms of latency

          double current_px = v*dt;
          double current_py = 0.0;
          double current_psi = v * (-delta)/Lf * dt;
          double current_v = v + a*dt;
          double current_cte = cte + v*sin(epsi) * dt;
          double current_epsi = epsi + v * (-delta)/Lf * dt;

          // save state
          Eigen::VectorXd state(6);
          state<< current_px, current_py, current_psi, 
                      current_v, current_cte, current_epsi;
```
2. The prediction timesteps `N` and intervals `dt` are defined and used to set up variables for the MPC optimizer.

3. The MPC cost is defined from `cte`, `epsi` and velocity `v`. The cost also accounts for actuators (`delta`, `a`) and the change in actuator values as in;
```cpp
        // Initialize the cost value
        fg[0] = 0;

        // Define the costs
        for (unsigned int t = 0; t < N; t++) {
            fg[0] += 2000*CppAD::pow(vars[cte_start + t], 2);
            fg[0] += 2000*CppAD::pow(vars[epsi_start + t], 2);
            fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
          }

        // Higher weights mean minimizing the use of actuators.
        for (unsigned int t = 0; t < N - 1; t++) {
            fg[0] += 0.001*CppAD::pow(vars[delta_start + t], 2);
            fg[0] += 5*CppAD::pow(vars[a_start + t], 2);
          }

        // Minimize the value gap between sequential actuations.
        // Higher weights will influence the solver into keeping sequential values closer togther
        for (unsigned int t = 0; t < N - 2; t++) {
            fg[0] += 20000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += 0.1*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
          }
```
3. The state variables and the obtained coefficients (from [main.cpp](https://github.com/toluwajosh/CarND-MPC-Project/blob/debug_and_finish/src/main.cpp)) are used to set up constraints for the MPC optimization. The aim of the constraints is to make the difference between values at time `t` and time `t+1` equal to zero. For example, in case of the `cte`, we have;

<p align="center">
<img src="/media/cte_t+1.png" alt="CTE equation" width="300" align="middle">
</p>

hence the constraint;

<p align="center">
<img src="/media/cte_constraint.png" alt="CTE constraint" width="300" align="middle">
</p>

This is implemented in the code as follows ([MPC.cpp](https://github.com/toluwajosh/CarND-MPC-Project/blob/debug_and_finish/src/MPC.cpp)):
```cpp
        //
        // Setup Constraints
        //
        // Initial constraints
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (unsigned int t = 0; t < N-1; t++) {
            .
            .
            .
            .
            .
            
            // Evalutate cte and desired psi
            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
            AD<double> psides0 = CppAD::atan(3*coeffs[3]*x0*x0+2*coeffs[2]*x0+coeffs[1]);

            // constraint for variables and errors
            fg[2 + x_start + t ] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
            fg[2 + cte_start + t] =
                cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_start + t] =
                epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
```
4. An optimizer ([Ipopt](https://projects.coin-or.org/Ipopt)) is given the initial state and then returns the vector of control inputs that minimize the cost function.

5. The fist control input is applied to the vehicle, and we repeat the process for subsequent waypoints.

### 1.2 Timestep Length and Elapsed Duration (`N` and `dt`)
I first used `N=10` and `dt=0.1` to observe the performance of the model. This worked well in most cases but the trajectory sometimes breaks at the curve. Since `dt` signifies elasped time, reducing it will result in the model predicting for a smaller elapsed time, so I chose `0.05`. I also tried `N=20`, but this resulted in the model having a farther prediction horizon. However, since we only use the first predicted value, this is not very useful. `N=10` and `dt=0.05` was finally chosen. This means we have a prediction horizon of `10 * 0.05 = 0.5 sec`, which resulted in a more stable prediction.

### 1.3 Poynomial Fitting and MPC Preprocessing
To make calculations easier, the reference position values were shifted into the car's coordinate (Mentioned in Sec: 1.1 No.1). Also a negative sign was added to `delta` after I observed that the car moves in the opposite direction of the predicted trajectory.

### 1.4 MPC with Latency
Latency of `100ms` was accounted for in the global kinematic model (Sec: 1.1 No.1 ) implementation. A `dt` of `0.1` was chosen. It was observed that the model performed better with the global kinematic model implementation.

## 2.0 Simulation
Follow the link below to see a video of the result of the MPC implementation in the simulator.

[Video Result](https://youtu.be/0DKpPkj5dWM)

### Summary
Once the model has been implemented to correctly update variables and actuators, the weights on each part of the cost function can be tuned to give a better performance. This had an impact on how fast or how smoothly the car could drive. After tuning these weights, the vehicle is able to drive safely up to a speed of 91 mph.