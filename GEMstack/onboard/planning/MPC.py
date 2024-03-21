import time
import yaml
import numpy as np
import casadi as ca
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.trajectory import Path, Trajectory, compute_headings
from ..component import Component
from ...knowledge.vehicle.geometry import front2steer
from scipy.interpolate import interp1d
from ...utils import settings
from ...mathutils import transforms

# TODO: change this to get the parameter directly from settings, same as in pure_pursuit.py
# class VehicleDynamics():
#     def __init__(self, vehicle_type=None):
#         # Read the vehicle dynamic variables
#         with open('/home/mike/GEMstack/GEMstack/knowledge/vehicle/gem_e2_geometry.yaml', 'r') as file:
#             geometry = yaml.safe_load(file) 
#         with open('/home/mike/GEMstack/GEMstack/knowledge/vehicle/gem_e2_fast_limits.yaml', 'r') as file:
#             dynamics = yaml.safe_load(file) 
        
#         self.geometry = geometry
#         self.dynamics = dynamics
        


class MPC(object):
    def __init__(self, dt, horizon, Q, R):
        self.dt = dt
        self.horizon = horizon
        self.Q = Q
        self.R = R
        # self.vehicle_dynamics = vehicle_dynamics
        self.look_ahead = 4.0
        self.look_ahead_scale = 3.0

        self.path_arg = None
        self.path = None 
        self.trajectory = None

        # Defining geometry and dynamic constraints on the vehicle
        self.front_wheel_angle_scale = 3.0
        self.wheelbase = settings.get('vehicle.geometry.wheelbase')
        self.max_deceleration = settings.get('vehicle.limits.max_deceleration')
        self.max_acceleration = settings.get('vehicle.limits.max_acceleration')
        self.min_steering_angle = settings.get('vehicle.geometry.min_steering_angle')
        self.max_steering_angle = settings.get('vehicle.geometry.max_steering_angle')

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
        # Implement the kinematic bicycle model equations using CasADi
        x_next = states[0] + states[3] * ca.cos(states[2]) * self.dt
        y_next = states[1] + states[3] * ca.sin(states[2]) * self.dt
        theta_next = states[2] + (states[3] / self.wheelbase) * ca.tan(controls[1]) * self.dt
        v_next = states[3] + controls[0] * self.dt
        return ca.vertcat(x_next, y_next, theta_next, v_next)

    def get_cost_function(self, states, controls, ref_trajectory):
        # Implement the cost function using CasADi
        cost = 0

        for t in range(self.horizon):
            v_x = ref_trajectory.eval_derivative(t)[0]
            v_y = ref_trajectory.eval_derivative(t)[1]
            ref_v = np.sqrt(v_x ** 2 + v_y ** 2)
            cost += (ref_trajectory.eval(t)[0] - states[0, t])**2 + \
                (ref_trajectory.eval(t)[1] - states[1, t])**2
            # if t > 0:
            #     delta_x = ref_trajectory.eval(t)[0] - ref_trajectory.eval(t - 1)[0]
            #     delta_y = ref_trajectory.eval(t)[1] - ref_trajectory.eval(t - 1)[1]
            #     ref_theta = np.arctan2(delta_y, delta_x)
            #     cost += (ref_theta - states[2, t])**2
            # else:
            #     cost += states[2, t]**2
            cost += (ref_v - states[3, t])**2
            cost += (ref_trajectory.eval(t)[2] - states[2, t])**2 + \
                (ref_v - states[3, t])**2
            cost += controls[0, t] ** 2 + controls[1, t] ** 2
        # if t > 0:
        #     cost += (controls[0, t] - controls[0, t-1])**2 + \
        #             (controls[1, t] - controls[1, t-1])**2
        # else:
        #     cost += controls[0, t]**2 + controls[1, t]**2
        return cost

    def compute(self, state: VehicleState, component: Component = None):
        assert state.pose.frame != ObjectFrameEnum.CURRENT
        t = state.pose.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        current_state = [state.pose.x, state.pose.y, state.pose.yaw if state.pose.yaw is not None else 0.0, state.v]

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
        #TODO: calculate parameter that is look_ahead distance away from the closest point?
        #(rather than just advancing the parameter)
        des_parameter = closest_parameter + self.look_ahead + self.look_ahead_scale * state.v
        print("Closest parameter: " + str(closest_parameter),"distance to path",closest_dist)
        if closest_dist > 0.1:
            print("Closest point",self.path.eval(closest_parameter),"vs",(state.pose.x,state.pose.y))
        if des_parameter >= self.path.domain()[1]:
            #we're at the end of the path, calculate desired point by extrapolating from the end of the path
            end_pt = self.path.points[-1]
            if len(self.path.points) > 1:
                end_dir = self.path.eval_tangent(self.path.domain()[1])
            else:
                #path is just a single point, just look at current direction
                end_dir = (np.cos(state.pose.yaw),np.sin(state.pose.yaw))
            desired_x,desired_y = transforms.vector_madd(end_pt,end_dir,(des_parameter-self.path.domain()[1]))
        else:
            desired_x,desired_y = self.path.eval(des_parameter)
        desired_yaw = np.arctan2(desired_y-state.pose.y,desired_x-state.pose.x)

        # TODO: looks like we should use the closest point every time for getting the reference trajectory
        # if we only use the curren time t to get the reference trajectory it will be very easy to get
        # large deviations
        ref_trajectory = self.path.trim(des_parameter, min(des_parameter + self.horizon, self.path.times[-1]))

        # Slice a range of trajectory given the horizon value
        # ref_trajectory = self.path.trim(state.pose.t, min(state.pose.t + self.horizon, self.path.times[-1]))
        ref_trajectory = compute_headings(ref_trajectory)
        # print("CURRENT STATE: ", current_state)
        print("REF TRAJECTORY: ", ref_trajectory)

        # Set up the optimization problem
        opti = ca.Opti()
        x_vars = opti.variable(4, self.horizon + 1)  # State variables
        u_vars = opti.variable(2, self.horizon) # Control variables

        # Set initial conditions
        opti.subject_to(x_vars[:, 0] == current_state)

        # TODO: need to set the dynamic constraints here
        # Set up the optimization problem
        for t in range(self.horizon):
            # Model equations constraints
            x_next = self.get_model_equations(x_vars[:, t], u_vars[:, t])
            opti.subject_to(x_vars[:, t + 1] == x_next)

            # Control input constraints
            opti.subject_to(opti.bounded(-self.max_deceleration, u_vars[0, t], self.max_acceleration))
            opti.subject_to(opti.bounded(self.min_steering_angle, u_vars[1, t], self.max_steering_angle))
            
        # Set the objective function
        obj = self.get_cost_function(x_vars, u_vars, ref_trajectory)
        opti.minimize(obj)

        # Set up the solver
        opti.solver('ipopt')

        # Solve the optimization problem
        sol = opti.solve()

        # Extract the optimal control inputs
        optimal_control = sol.value(u_vars)

        # Extract the optimal steering angle and acceleration
        optimal_acceleration = optimal_control[0, 0]
        optimal_steering = optimal_control[1, 0]

        # Convert the steering angle to the corresponding steering wheel angle
        steering_wheel_angle = optimal_steering

        return optimal_acceleration, steering_wheel_angle

class MPCController(Component):
    def __init__(self, vehicle_interface=None, **args):
        # self.vehicle_dynamics = VehicleDynamics()
        self.MPC = MPC(**args)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 1.0

    def state_inputs(self):
        return ['vehicle', 'trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        start_time = time.perf_counter()
        self.MPC.set_path(trajectory)
        acceleration, steering_wheel_angle = self.MPC.compute(vehicle)
        # steering_angle = np.clip(front2steer(steering_wheel_angle), self.MPC.steering_angle_range[0], self.MPC.steering_angle_range[1])
        print("acceleration: ", acceleration)
        print("steering_wheel_angle", steering_wheel_angle)
        # print("steer_angle", steering_angle)
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(acceleration, -steering_wheel_angle, vehicle))

        end_time = time.perf_counter()
        execution_time = end_time - start_time
        print(f"Execution time: {execution_time:.4f} seconds")

    def healthy(self):
        return self.MPC.path is not None
    