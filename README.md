This serves as a document for recording the progress in implementing a MPC controller in GEMstack, with explanations on how to run things
and where to find components necessary to run a basic demo that uses the different controllers available in GEMstack

# Example simulation demo 

In GEMstack, run `python3 main.py --variant=sim launch/fixed_route.yaml` to shoot up a simulation demo that has the vehicle following 
a fixed route. 
- The fixed route is defined in `GEMstack/knowledge/routes/xyhead_highbay_backlot_p.csv`, which is defined as a list of poses consisting of
the x, y, and theta value.
- Since we are not dealing with the planning algorithm, we don't have to worry about how the route is generated. The route will go through
a simple preprocessing step, which is defined in `GEMstack/onboard/planning/motion_planning.py`.

# Controllers
There are different controllers implemented in GEMstack for people to test out. Currently all the controllers assume a simple kinematic bicycle model, which takes the **acceleration** and the **steering wheel angle** as the control input. 

TODO: add some notes on parameter tuning.

## Pure Pursuit Controller
The pure pursuit controller is the given baseline controller, and is defined in `GEMstack/onboard/planning/pure_pursuit.py`. In every update loop, it calculates a desired pose to follow based on the vehicle's current pose, from which we can derive the control input using simple geomtry calculations.

## MPC Controller
The MPC controller is the controller our group works on, and is defined in `GEMstack/onboard/planning/MPC.py`. Let's walk through how it works based on our current implementation:
- We define the vehicle state at time $t$ as $X_t = (x_t, y_t, \theta_t, v_t)$, the x and y coordinates, the heading, and the current velocity of the vehicle.
- We define the control input at time $t$ as $U_t = (a_t, \delta_t)$, the acceleration and the front wheel angle.
- We estimate the state using a simple kinematic bicycle model. Note that $d_t$ is the time step duration.

  $x_{t+1} = x_t + v_t \cos(\theta_t) * dt$ 

  $y_{t+1} = y_t + v_t \sin(\theta_t) * dt$ 

  $\theta_{t+1} = \theta_t + \frac{v_t}{L} \delta_t * dt$ where $L$ is the wheelbase length of the vehicle

  $v_{t+1} = v_t + a_t * dt$

- We define our cost function as following. Note that $n$ is the horizon duration, and $Q = (q_x, q_y, q_\theta, q_v)$, $R = (r_a, r_\delta)$, and $S = (s_a, s_\delta)$ are weight parameters.

  ![Screenshot from 2024-04-17 23-38-22](https://github.com/krishauser/GEMstack/assets/47130002/b74a424b-94eb-4f67-8eca-1da9f8e312a1)


- We want to note that the performance of the MPC is highly dependable on the cost function. Feel free to customize it based on your specific problem and experiment with different parameters.
