# Model Predictive Control

The goals of this project are:

* Use Model Predictive Control to drive a car around a track in a simulator.
* Calculate the ideal path of the car from a provided set of waypoints.
* Select a timestep length and duration for use in path calculations.
* Compute cross-track error to determine distance from the desired path.
* Define a cost function and appropriate weights to determine actuator changes.
* Apply the actuator changes (steering and throttle) to drive the car.
* Drive as fast as possible!

---

### Video of Results

Here's a [link to the video](https://github.com/bertciccone/CarND-MPC-Project/blob/master/video/mpc_video.mov) of the car completing a lap around the track in the Unity simulator.

[![MPC Video](https://github.com/bertciccone/CarND-MPC-Project/blob/master/img/Screen%20Shot%202017-12-11%20at%2010.48.12%20AM.png)](https://github.com/bertciccone/CarND-MPC-Project/blob/master/video/mpc_video.mov)

---

## [Rubric Points](https://review.udacity.com/#!/rubrics/896/view)

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### The code should compile.

#### Code must compile without errors with cmake and make.

Bert (master) build $ cmake .. && make  
-- Configuring done  
-- Generating done  
-- Build files have been written to:  
/Users/Bert/Projects/Udacity/CARND/term2/projects/CarND-MPC-Project/build  
[100%] Built target mpc  

### The Model

#### Description of the model in detail. This includes the state, actuators and update equations.

##### State

The car's state includes its X-Y position in the simulation map, its steering angle and its velocity. For this simulation, the state also includes its cross-track error (distance from the ideal path on the track) and its error in steering angle.

This state is stored in the following data structure in the function main() in main.cpp:

`Eigen::VectorXd state(6);`

state[0]: the car's x position  
state[1]: the car's y position  
state[2]: the car's velocity  
state[3]: the car's direction  

Also stored in this data structure are the following:  
state[4]: the car's cross-track error  
state[5]: the car's error in psi (direction)  

##### Actuators

The car's actuators include steering, accelerator and brake. for this simulation, accelerator and brake are combined as an accelerator value, with negative values representing braking.

In the code, the actuators are stored in the following data structure in main() in main.cpp:

`Eigen::VectorXd actuators(2);`

actuators[0]: the car's steering angle  
actuators[1]: the car's throttle  

##### Update Equations

At each timestep, the model is updated in the function FG_eval() in MPC.cpp by the following code which implements the MPC update equations:

`//
fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt); // x  
fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt); // y  
fg[2 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt); // psi (direction)  
fg[2 + v_start + t] = v1 - (v0 + a0 * dt); // v  
fg[2 + cte_start + t] =  
    cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt)); // cross-track error  
fg[2 + epsi_start + t] =  
    epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt); // error in psi
//`  

The following statements in FG_eval() in MPC.cpp calculate a Reference State Cost that specify how far from zero each state value should be. In effect, these statements adjust the update to tune the degree and frequency of actuator changes and how closely the car needs to follow the idea path. A significant amount of experimentation was needed to arrive at acceptable constant weights applied to these calculations to enable the car to successfully navigate the track at high speed.

`// The part of the cost based on the reference state.  
for (int i = 0; i < N; i++) {  
  fg[0] += 20000 * CppAD::pow(vars[cte_start + i] - ref_cte, 2);  
  fg[0] += 20000 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);  
  fg[0] += 3 * CppAD::pow(vars[v_start + i] - ref_v, 2);  
}

// Minimize the use of actuators.  
for (int i = 0; i < N - 1; i++) {  
  fg[0] += 50 * CppAD::pow(vars[delta_start + i], 2);  
  fg[0] += 50 * CppAD::pow(vars[a_start + i], 2);  
}

// Minimize the value gap between sequential actuations.  
for (int i = 0; i < N - 2; i++) {  
  fg[0] += 400 *  
           CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);  
  fg[0] += 50 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);  
}
//`

### Timestep Length and Elapsed Duration (N & dt)

#### Discussion of the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally, details of the previous values tried.

Initially, the timestep length was set at 10 and elapsed duration between timesteps of 0.1 second, which provided a "lookahead window" of one second. It is desirable to choose a combination of timestep length and timestep to minimize processing time while providing good results.

After trying smaller values of timestep length of 5 and 8, I saw that the car exhibited severe swerving left to right, and therefore concluded that 10 served as a reasonable lower limit for timestep length.

Timestep of 0.1 worked well while no delay was introduced into the simulation. However, when the delay was set to 100 milliseconds, the car could not stay on the track. I felt that with the delay, a longer "lookahead window" might be necessary, and so increased the timestep to 0.25, increasing the total time from one second to 2.5 seconds. This indeed enabled the car to stay on the track, but resulted in extreme slowing around curves. With experimentation, I was able to reduce this to 0.18, enabling much faster curve performance.

### Polynomial Fitting and MPC Preprocessing

#### 1. A polynomial is fitted to waypoints.

This code in main() in main.cpp fits a polynomial to the track waypoints:

`//
double poly_inc = 2.5;  
int num_points = 25;  
for (int i = 1; i < num_points; i++) {  
  next_x_vals.push_back(poly_inc * i);  
  next_y_vals.push_back(polyeval(coeffs, poly_inc * i));  
}
//`

#### 2. Preprocessing of waypoints, the vehicle state, and/or actuators prior to the MPC procedure is described.

N/A

### Model Predictive Control with Latency

#### Implementation of Model Predictive Control that handles a 100 millisecond latency. Details on how it deals with latency.

Latency in the system is handled in the function globalKinematic() defined in main.cpp, which predicts the car's location on the simulation map at a point in time 100 ms in the future, based on the car's current state and actuator values. For the purposes of this simulation, the car's accelerator value is substituted for acceleration (which is not available in the simulator) in the equations implemented in the code.

These are the statements in the function globalKinematic() in main.cpp that perform this prediction:

`//
double Lf = 2.67;  
next_state[0] = state[0] + state[3] * cos(state[2]) * dt; // x  
next_state[1] = state[1] + state[3] * sin(state[2]) * dt; // y  
next_state[2] = state[2] + state[3] / Lf * actuators[0] * dt; // v  
next_state[3] = state[3] + actuators[1] * dt; // psi (direction)
//`

### The vehicle successfully drives a lap around the track.

#### 1. No tire leaves the drivable portion of the track surface. The car does not pop up onto ledges or rolls over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).
#### 2. The car does not go over the curb, but does drive on the lines.

Here's a [link to the video](https://github.com/bertciccone/CarND-MPC-Project/blob/master/video/mpc_video.mov) of the car completing a lap around the track in the Unity simulator.
