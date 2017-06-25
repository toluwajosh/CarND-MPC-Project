# Model Predictive Control Poject
Udacity Self-Driving Car Engineer Nanodegree Program

Term 2, Project 5

---

**The objective of this project is to implement a Model Predictive Control (MPC) to drive a car around a simulated track.**
In the model, the cross track error is calculated and a latency of 100 milliseconds is accounted for.

## Implementation
### The model:
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
          // change reference pose into car's coordinate
          for (unsigned int i = 0; i < ptsx.size(); ++i)
          {
              double shift_x = ptsx[i]-px;
              double shift_y = ptsy[i]-py;

              ptsx[i] = (shift_x*cos(0-psi) - shift_y*sin(0-psi));
              ptsy[i] = (shift_x*sin(0-psi) + shift_y*cos(0-psi));
          }

          // convert points to VecotrXD for use in polyfit function
          double* ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

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
![gkm](/media/global_kinematic_model_eqns.png)

as in
```cpp
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];
          double dt = 0.1;

          double current_px = v*dt;
          double current_py = 0.0;
          double current_psi = v * (-delta)/Lf * dt;
          double current_v = v + a*dt;
          double current_cte = cte + v*sin(epsi) * dt;
          double current_epsi = epsi + v * (-delta)/Lf * dt;

          // save state
          Eigen::VectorXd state(6);
          // state<< 0,0,0,v,cte,epsi; // zeros because, earlier transformations
          state<< current_px, current_py, current_psi, 
                      current_v, current_cte, current_epsi;
```

2. I used the state variables and the obtained coefficients to set up constraints for the MPC optimization
(See: [MPC.cpp](https://github.com/toluwajosh/CarND-MPC-Project/blob/debug_and_finish/src/MPC.cpp)):
