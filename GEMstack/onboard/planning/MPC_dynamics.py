import time
import yaml
import numpy as np
import casadi as ca
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.trajectory import Path, Trajectory, compute_headings
from ..component import Component
from ...knowledge.vehicle.geometry import front2steer
from ...knowledge.vehicle.geometry import steer2front
from scipy.interpolate import interp1d
from ...utils import settings
from ...mathutils import transforms        

class MPC(object):
    def __init__(self, dt, horizon, Q, R):
        # Defining the tunable parameters
        self.dt = dt
        self.horizon = horizon # horizon represented in seconds
        self.timesteps = int(self.horizon / self.dt) # horizon represented in the number of time steps
        self.Q = Q
        self.R = R
        self.look_ahead = 2.0
        self.look_ahead_scale = 1.0
        self.prev_yaw_angle = None
        self.prev_ref_yaw = None
        self.path_arg = None
        self.path = None 
        self.trajectory = None
        self.front_wheel_angle = None
        self.wheel_rad = settings.get('vehicle.dynamics.wheel_rad')
        self.gear_ratio = settings.get('vehicle.dynamics.gear_ratio')
        # Defining geometry and dynamic constraints on the vehicle
        # self.front_wheel_angle_scale = 3.0
        self.yaw_moment_of_inertia = settings.get('vehicle.dynamics.yaw_moment_of_inertia')
        self.vehicle_mass = settings.get('vehicle.dynamics.mass')
        self.distance_front_to_cg = settings.get('vehicle.dynamics.lf')
        self.distance_rear_to_cg = settings.get('vehicle.dynamics.lr')
        self.aerodynamic_drag_coefficient = settings.get('vehicle.dynamics.aerodynamic_drag_coefficient')
        self.g = settings.get('vehicle.dynamics.gravity')
        self.front_cornering_stiffness = settings.get('vehicle.dynamics.front_cornering_stiffness')
        self.rear_cornering_stiffness = settings.get('vehicle.dynamics.rear_cornering_stiffness')
        self.peak_torq = settings.get('vehicle.dynamics.peak_torque')
        self.wheelbase = settings.get('vehicle.geometry.wheelbase')
        self.max_deceleration = settings.get('vehicle.limits.max_deceleration')
        self.max_acceleration = settings.get('vehicle.limits.max_acceleration')
        self.min_wheel_angle = settings.get('vehicle.geometry.min_wheel_angle')
        self.max_wheel_angle = settings.get('vehicle.geometry.max_wheel_angle')
        self.min_steering_angle = settings.get('vehicle.geometry.min_steering_angle')
        self.max_steering_angle = settings.get('vehicle.geometry.max_steering_angle')

        self.rolling_resist = settings.get('vehicle.dynamics.rolling_resistance_coefficient')
        self.t_last = None


    def set_path(self, path: Path):
        if path == self.path_arg:
            return
        self.path_arg = path
        if len(path.points[0]) > 2:
            path = path.get_dims([0, 1])
        if not isinstance(path, Trajectory):
            self.path = path.arc_length_parameterize()
            self.trajectory = None
            self.current_traj_parameter = 0.0
        else:
            self.path = path.arc_length_parameterize()
            self.trajectory = path
            self.current_traj_parameter = self.trajectory.domain()[0]
        self.current_path_parameter = 0.0

    def get_model_equations(self, states, controls):

        # dynamics
        x = states[0]  # the x position, in the object's frame. 
        y = states[1]  # the y position, in the object's frame. 
        theta = states[2]  #  the optional yaw, in radians and in the object's frame. If
            # frame=GLOBAL, this is heading CW from north.  Otherwise, it is
            # CCW yaw.
        v = states[3]
        v_y = v * ca.sin(theta)
        v_x = v * ca.cos(theta)
        # v_x = states[3] # longitudinal vel
        # v_y = states[4] # lateral vel
        omega = states[4] # yaw rate of change
        epsilon = 1e-7
        throttle = controls[0] # accleration 
        delta = controls[1] # steering wheel angle
        
        m = self.vehicle_mass
        Iz = self.yaw_moment_of_inertia
        lf = self.distance_front_to_cg
        lr = self.distance_rear_to_cg
        Cf = self.front_cornering_stiffness
        Cr = self.rear_cornering_stiffness
        peak_torque = self.peak_torq
        gear_ratio = self.gear_ratio
        wheel_rad = self.wheel_rad
        rolling_resist = self.rolling_resist
        # x, y, theta, v_x, v_y, omega = states  # Adding lateral velocity (v_y) and yaw rate (omega) to the states
        # throttle, delta = controls  # Throttle (or brake) and steering angle delta as controls

        # Aerodynamic and gravitational forces
        F_aero = self.aerodynamic_drag_coefficient * v_x**2
        F_gravity = m * self.g 
        Fz_front = m * self.g * (lr / (lf + lr))
        Fz_rear = m * self.g * (lf / (lf + lr))

        # Tire forces
        alpha_f = delta - ca.arctan((omega * lf + v_y) / (v_x + epsilon))
        alpha_r = ca.arctan((omega * lr - v_y) /  (v_x + epsilon))
        # Fyf = Cf * ( ca.sin( 1.3 * ca.arctan(12 * alpha_f))  )# force on front tire
        # Fyr = Cr * ( ca.sin( 1.2 * ca.arctan(10 * alpha_r) ) )# force on rear ti
        
        Fyf = Cf * alpha_f  # force on front tire
        Fyr = Cr * alpha_r  # force on rear ti

        # Initial traction force
        
        traction_force = peak_torque * gear_ratio / wheel_rad
        F_t = (traction_force  )* throttle- rolling_resist* m * self.g 
        
        #motion
        # x_next = x + (v_x * ca.sin(theta) - v_y * ca.cos(theta)) * self.dt  # Next x position
        # y_next = y + (v_x * ca.cos(theta) + v_y * ca.sin(theta)) * self.dt  # Next y position
        x_next = x + v_x * ca.cos(theta) * self.dt  # movement in x-direction
        y_next = y + v_y * ca.sin(theta) * self.dt  # movement in y-direction

        # Update velocities
        
        dv_x = (F_t - F_aero - F_gravity  - Fyf * ca.sin(delta)) / m + v_y * omega  # Change in longitudinal velocity
        dv_y = (Fyr - Fyf * ca.cos(delta) ) / m - v_x * omega  # Change in lateral velocity
        
        v_x_next = dv_x   # Next longitudinal velocity
        v_y_next = dv_y  # Next lateral velocity

        v_next = v + ca.sqrt(v_x_next**2 + v_y_next**2)* self.dt
        # v_next = v  + throttle * self.dt
        # Update yaw rate
        domega = (lf * Fyf * ca.cos(delta)  - lr * Fyr ) / Iz  # Change in yaw rate

        theta_next = theta + omega* self.dt  # Next orientation
        #state.v / self.wheelbase * np.tan(state.front_wheel_angle)
        omega_next = omega + domega *self.dt
    # Return the predicted next state
        return ca.vertcat(x_next, y_next, theta_next, v_next, omega_next)
    
        # kinematics
        # x_next = states[0] + states[3] * ca.cos(states[2]) * self.dt
        # y_next = states[1] + states[3] * ca.sin(states[2]) * self.dt
        # theta_next = states[2] + (states[3] / self.wheelbase) * ca.tan(controls[1]) * self.dt
        # v_next = states[3] + controls[0] * self.dt
        
     

        return ca.vertcat(dx, dy, dtheta, dv_x, dv_y, domega)
        # return ca.vertcat(x_next,y_next,theta_next,v_next)

    def get_cost_function(self, states, controls, ref_trajectory,opti):
        # Implement the cost function using CasADi
        cost = 0
        t = ref_trajectory.times[0]
        i = 0
       # print("reeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeef",ref_trajectory)
        while t < ref_trajectory.times[0] + self.horizon:

            x = states[0, i]
            y = states[1, i]
            theta = states[2, i]
            # v_x = states[3, i]
            # v_y = states[4, i]
            v = states[3,i]

            omega = states[4, i]

            throttle = controls[0, i]
            delta = controls[1, i]
            

            # Get the reference states from the trajectory
            ref_x = ref_trajectory.eval(t)[0]
            ref_y = ref_trajectory.eval(t)[1]
            ref_theta = ref_trajectory.eval(t)[2]
            ref_v_x = ref_trajectory.eval_derivative(t)[0]
            ref_v_y = ref_trajectory.eval_derivative(t)[1]
            ref_omega = ref_trajectory.eval_derivative(t)[2]
            ref_v = ca.sqrt(ref_v_x **2 + ref_v_y**2)
            print('ref-v_x',ref_v_x)
            print('ref-v_y',ref_v_y)
            print('ref_theta',ref_theta)
            print('ref-tangx',ref_trajectory.eval_tangent(t)[0])
            print('ref-tangy',ref_trajectory.eval_tangent(t)[1])
            # print('ref-yaw',ref_trajectory.eval_yaw(t))
            e_c = ca.sin(ref_theta) * (x - ref_x) - ca.cos(ref_theta) * (y - ref_y)
            e_l = -ca.cos(ref_theta) * (x - ref_x) - ca.sin(ref_theta) * (y - ref_y)

            # Penalize deviations from the reference states
            cost += self.Q[0]  *(x - ref_x) ** 2
            cost += self.Q[1]  *(y - ref_y) ** 2
            cost += self.Q[2] * (theta - ref_theta) ** 2
            
            # v = ca.sqrt(v_x**2+v_y**2)
            v_ref = ca.sqrt(ref_v_y**2 + ref_v_x**2)
            print('ref-v_',v_ref)
            # cost += self.Q[3] * (v_x - ref_v_x) ** 2
            # cost += self.Q[4] * (v_y - ref_v_y) ** 2

            cost += self.Q[3] * (v - ref_v)**2
            cost += self.Q[4] * (omega - ref_omega) ** 2
            # total speed penalty
            

            # Penalize control effort
            
            # cost += 2 * e_c ** 2
            # cost += 2 * e_l ** 2
            # Penalize large fluctuations in consecutive controls
            if i >= 1:
                cost += self.R[0] * throttle ** 2
                cost += self.R[1] * delta ** 2
                cost += self.R[2] * (throttle - controls[0, i - 1]) ** 2
                cost += self.R[3] * (delta - controls[1, i - 1]) ** 2
                
            t += self.dt
            i += 1
        return cost

    def compute(self, state: VehicleState, component: Component = None):
        assert state.pose.frame != ObjectFrameEnum.CURRENT
        t = state.pose.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last
        # given foward velocity, calculate lateral velocity = forward velocity * tan(steer angle)
        #TODO given yaw, x, y position,and forward vel, calculate , yaw rate of change/ lateral vel
        if state.pose.yaw is not None:

            lateral_v = state.v * ca.sin(state.pose.yaw)
            longitudinal_v = state.v * ca.cos(state.pose.yaw)
        else:
            lateral_v = 0.0
            longitudinal_v = state.v
        # yaw rate of change
        # if self.prev_yaw_angle is None:
            # domega = state.pose.yaw  / self.dt
        # else:
        #     domega = (state.pose.yaw - self.prev_yaw_angle) / self.dt
        if state.pose.yaw is not None and state.front_wheel_angle is not None:
            initial_yaw_rate = state.v / self.wheelbase * ca.tan(state.front_wheel_angle)
        else:
            initial_yaw_rate = 0.0

        domega = (state.v * ca.tan(state.front_wheel_angle))/ self.wheelbase
        # self.front_wheel_angle = state.front_wheel_angle
        print("yaww",state.pose.yaw)
        #current_state = [state.pose.x, state.pose.y, state.pose.yaw if state.pose.yaw is not None else 0.0, state.v]
        current_state = [state.pose.x, state.pose.y, state.pose.yaw if state.pose.yaw is not None else 0.0,state.v, state.heading_rate ]
        
        if self.path is None:
            raise RuntimeError("Behavior without path not implemented")

        if self.path.frame != state.pose.frame:
            print("Transforming path from", self.path.frame.name, "to", state.pose.frame.name)
            self.path = self.path.to_frame(state.pose.frame, current_pose=state.pose)
        if self.trajectory is not None:
            if self.trajectory.frame != state.pose.frame:
                print("Transforming trajectory from", self.trajectory.frame.name, "to", state.pose.frame.name)
                self.trajectory = self.trajectory.to_frame(state.pose.frame, current_pose=state.pose)

        closest_dist,closest_parameter = self.path.closest_point_local((state.pose.x,state.pose.y),[self.current_path_parameter-5.0,self.current_path_parameter+5.0])
        self.current_path_parameter = closest_parameter
        self.current_traj_parameter += dt

        des_parameter = closest_parameter + self.look_ahead + self.look_ahead_scale * state.v
        print("Desired parameter: " + str(des_parameter),"distance to path",closest_dist)
        print(self.path.parameter_to_time(des_parameter))

        # Slice a range of trajectory given the horizon value
        ref_trajectory = self.path.trim(des_parameter, min(des_parameter + self.timesteps, self.path.times[-1]))
        ref_trajectory = compute_headings(ref_trajectory)
        # print("CURRENT STATE: ", current_state)
        # print("REF TRAJECTORY: ", ref_trajectory)
        # print("LEN OF REF: ", len(ref_trajectory.times))

        # Set up the optimization problem
        opti = ca.Opti()
        # kinematics model variables
        # x_vars = opti.variable(4, self.timesteps + 1)  # State variables
        # u_vars = opti.variable(2, self.timesteps) # Control variables
        # dynamics model variables
        x_vars = opti.variable(5, self.timesteps + 1)  # State variables
        u_vars = opti.variable(2, self.timesteps) # Control variables
       
        # Set initial conditions
        opti.subject_to(x_vars[:, 0] == current_state)

        for t in range(self.timesteps):
            # Model equations constraints
            #TODO objective function?
            x_next = self.get_model_equations(x_vars[:, t], u_vars[:, t])
            opti.subject_to(x_vars[:, t + 1] == x_next)

            # State constraints
            # opti.subject_to(x_vars[:, t] < self.max_wheel_angle)
            # opti.subject_to(x_vars[:, t] > self.min_wheel_angle)
            
            # Control input constraints
            #opti.subject_to(opti.bounded(-1, x_vars[4, t], 1))

            opti.subject_to(opti.bounded(-self.max_deceleration, u_vars[0, t], self.max_acceleration))
            opti.subject_to(opti.bounded(self.min_wheel_angle, u_vars[1, t], self.max_wheel_angle))
            
        # Set the objective function
        #TODO two more var need to be consider in cost
        obj = self.get_cost_function(x_vars, u_vars, ref_trajectory,opti)
        
        opti.minimize(obj)
        
        # options = {'ipopt': {
        #     'tol': 1e-8,
        #     'max_iter': 5000,
        #     # 'linear_solver': 'MA57',  # Assuming you have it available
        #     # 'derivative_test': 'none',  # Turn off for performance
        #     # 'nlp_scaling_method': 'none',
        #     # 'constr_viol_tol': 1e-6
        #     # 'warm_start_init_point': 'no',  # Use 'yes' for warm starts
        #     # 'print_level': 1
        # }}
        options = {
        'ipopt': {
        'max_iter': 20000,
        'tol': 1e-6,
        'dual_inf_tol': 1e-6,
        'constr_viol_tol': 1e-6,
        'acceptable_tol': 1e-5,  # Looser tolerance for accepting convergence
        'linear_solver': 'mumps',  # Robust linear solver
        'print_level': 0,  # Provides detailed output about solver progress and issues
        'hessian_approximation': 'limited-memory',  # Good for large-scale problems
        'derivative_test': 'first-order',  # Check derivatives (gradient) correctness63254
    }
}
    #     options = {
    #     'ipopt': {
    #     'max_iter': 10000,
    #     'tol': 1e-10,
    #     'acceptable_tol': 1e-10,
    #     'linear_solver': 'mumps',
    #     'hessian_approximation': 'limited-memory'
    # }

# }
        
        # Set up the solver
        opti.solver('ipopt', options)
        
        # Solve the optimization problem
        try:
            sol = opti.solve()
            
            # Extract the optimal control inputs
            optimal_control = sol.value(u_vars)

            # Extract the optimal steering angle and acceleration
            optimal_acceleration = optimal_control[0, 0]
            optimal_steering = optimal_control[1, 0]

            # Convert the steering angle to the corresponding steering wheel angle
            steering_wheel_angle = optimal_steering

        except:
            optimal_acceleration = 0.0
            steering_wheel_angle = 0.0
        return optimal_acceleration, steering_wheel_angle

class MPCController(Component):
    def __init__(self, vehicle_interface=None, **args):
        self.MPC = MPC(**args)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 5.0

    def state_inputs(self):
        return ['vehicle', 'trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        start_time = time.perf_counter()
        self.MPC.set_path(trajectory)
        acceleration, wheel_angle = self.MPC.compute(vehicle)
        print("curr_acc_pedal",vehicle.accelerator_pedal_position)
        print("curr_brk_pedal",vehicle.brake_pedal_position)

        print("curr_vel",vehicle.v)
        print("acceleration: ", acceleration)
        print("wheel_angle", wheel_angle)
        steering_wheel_angle = np.clip(front2steer(wheel_angle), self.MPC.min_steering_angle, self.MPC.max_steering_angle)
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(acceleration, steering_wheel_angle, vehicle))

        end_time = time.perf_counter()
        execution_time = end_time - start_time
        print(f"Execution time: {execution_time:.4f} seconds")

    def healthy(self):
        return self.MPC.path is not None
    