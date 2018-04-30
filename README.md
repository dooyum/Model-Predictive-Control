# Vehicle Controls using MPC Project
Self-Driving Car Engineer Nanodegree Program

---

## MPC Model

An MPC model was used to control a vehicle around a simulated track with provided waypoints. The ideal state was for the vehicle to remain within the lane while gradually speeding up with confidence.
MPC optimizes the state of the vehicle to minimize any deviation from the waypoints.
The state of the vehicle is characterized by:
* Global polar coordinates: (x, y and psi)
* Vehicle velocity
* Acceleration
* Actuators: (throttle, steering angle)

Every deviation from the ideal state is considered an error. The errors that were taken into consideration were:
* Cross track error (**CTE**): This is the y coordinate deviation from the ideal path.
* Psi Error (**ePSI**): This is the deviation from the angle of the ideal path.

Using the MPC model has the following steps:
1. Transform provided waypoints from global coordinates to vehicle coordinates.
2. Fit a 3rd degree polynomial to the vehicle coordinate waypoints and retrieve the fit coefficients. i.e. **C0 + C1(x) + C2(x^2) + C3(x^3)**
3. Evaluate the current state with respect to the derived polynomial and calculating the CTE and ePSI.
4. Based on the computed errors, update the actuators of the vehicle to minimize the errors and bring the vehicle to a more ideal state. Weights are used to indicate the importance of minimizing each type of error.

## Timestep Length and Elapsed Duration
The timestep length(**N**) and elapsed duration(**dt**) are parameters used to tune the performance of the MPC model. **N** denotes the number of timesteps into the future the vehicle should consider. **dt** denotes the duration between each timestep.
The chosen parameter values significantly affect the performance of the MPC model.
* A high **N** mght consider upcoming waypoints that are too far ahead to affect our current state.
* A low **N** doesn't look far enough at the upcoming waypoints. The model is likely to be surprised at sudden changes in the path, like bends or lane switches.
* A high **dt** might result in infrequent evaluation of upcoming states.
* A low **dt** might result in higher precision but ultimately wasted computation of future states. This is as a result of the fact that the state from one timestep to another is not likely to be markedly different in this situation.

I tried setting **N** to 100, but the model considered points that were too far ahead of our vehicle state, affecting the derived polynomial as a result.
The sweet spot was an **N** value of 10. Based on the **dt** selected, this was equivalent to evaluating waypoints 1 second ahead.

I ran the simulation on a laptop with relatively weak specs. AS a result, I chose a **dt** that enable smooth optimization but did not take up all the computing resources.
A **dt** of 0.1(100ms) was ideal.

## Handling Latency
In a real world scenario, although these computations happen relatively qiuckly with powerful on-board computers, there is always some latency between when the MPC model issues new actuation values and when the physical actuators like the steering, brake and gas pedal are engaged.
As a result of this latency, the vehicle state might have significantly changed from the state at the time the actuator values were dispatched.
To handle such a possibility, when new measurements are recieved for the vehicles current state **state_t**, we project where the vehicle will be in the immediate future (**state_t1**). This projection assumes that the actutor values remain constant.
The state values measured are:
* **x_t**: The current **x** global coordinate of the vehicle
* **y_t**: The current **y** global coordinate of the vehicle
* **psi_t**: The current vehicle orientation in radians
* **velocity_t**: The current velocity of the vehicle
* **acceleration**: The current actuator value responsible for acceleration of the vehicle
* **delta**: The current actuator value responsible for altering the orientation of the vehicle
* **Lf**: Constant that reflects the length of the vehicle relative to it's center of gravity.

This is how we derive various aspects of the future state **state_t1**:
* **x_t1 = x_t + velocity_t * cos(psi_t) * latency**
* **y_t1 = y_t + velocity_t * sin(psi_t) * latency**
* **psi_t1 = psi_t + (delta * velocity_t * latency) / Lf**
* **v_t1**: = v_t * acceleration**

With the future state, we compute our **CTE** and **ePSI** and apply the actuators computed for **state_t1** at time **t**.