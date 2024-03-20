import yaml
import numpy as np
import casadi as ca
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.trajectory import Path, Trajectory, compute_headings
from ..component import Component
from scipy.interpolate import interp1d

# TODO: could have a parameter for defining the vehicle type, i.e., e2 or e4
class VehicleDynamics():
    def __init__(self, vehicle_type=None):
        # Read the vehicle dynamic variables
        with open('/home/mike/GEMstack/GEMstack/knowledge/vehicle/gem_e2_geometry.yaml', 'r') as file:
            geometry = yaml.safe_load(file) 
        with open('/home/mike/GEMstack/GEMstack/knowledge/vehicle/gem_e2_fast_limits.yaml', 'r') as file:
            dynamics = yaml.safe_load(file) 
        
        self.geometry = geometry
        self.dynamics = dynamics


class MPC(object):
    def __init__(self, vehicle_dynamics, dt, horizon, Q, R):
        self.dt = dt
        self.horizon = horizon
        self.Q = Q
        self.R = R
        self.vehicle_dynamics = vehicle_dynamics

        self.path_arg = None
        self.path = None 
        self.trajectory = None

        # Define the optimization variables
        self.delta = ca.SX.sym('delta', self.horizon)  # Steering angles
        self.a = ca.SX.sym('a', self.horizon)  # Accelerations

        # Define the state variables
        self.x = ca.SX.sym('x')
        self.y = ca.SX.sym('y')
        self.theta = ca.SX.sym('theta')
        self.v = ca.SX.sym('v')
        self.states = ca.vertcat(self.x, self.y, self.theta, self.v)

        # Define the reference trajectory variables
        self.x_ref = ca.SX.sym('x_ref', self.horizon)
        self.y_ref = ca.SX.sym('y_ref', self.horizon)
        self.theta_ref = ca.SX.sym('theta_ref', self.horizon)
        self.v_ref = ca.SX.sym('v_ref', self.horizon)

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
        theta_next = states[2] + (states[3] / self.vehicle_dynamics.geometry['wheelbase']) * ca.tan(controls[0]) * self.dt
        v_next = states[3] + controls[1] * self.dt
        return ca.vertcat(x_next, y_next, theta_next, v_next)

    def get_cost_function(self, states, controls, ref_trajectory):
        # Implement the cost function using CasADi
        cost = 0
        print(ref_trajectory)
        for t in range(self.horizon):
            v_x = ref_trajectory.eval_derivative(t)[0]
            v_y = ref_trajectory.eval_derivative(t)[1]
            ref_v = np.sqrt(v_x ** 2 + v_y ** 2)
            cost += (ref_trajectory.eval(t)[0] - states[0, t])**2 + \
                (ref_trajectory.eval(t)[1] - states[1, t])**2
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

        closest_dist, closest_parameter = self.path.closest_point_local((state.pose.x, state.pose.y), [self.current_path_parameter - 5.0, self.current_path_parameter + 5.0])
        self.current_path_parameter = closest_parameter
        self.current_traj_parameter = self.path.parameter_to_time(self.current_path_parameter)

        # Slice a range of trajectory given the horizon value
        ref_trajectory = self.path.trim(self.current_traj_parameter, min(self.current_traj_parameter + self.horizon, self.path.times[-1]))
        ref_trajectory = compute_headings(ref_trajectory)

        # Set up the optimization problem
        opti = ca.Opti()
        x_vars = opti.variable(4, self.horizon + 1)  # State variables
        u_vars = opti.variable(2, self.horizon)  # Control variables

        # Set initial conditions
        opti.subject_to(x_vars[:, 0] == current_state)

        # TODO: need to set the dynamic constraints here
        # Set up the optimization problem
        for t in range(self.horizon):
            # Model equations constraints
            x_next = self.get_model_equations(x_vars[:, t], u_vars[:, t])
            opti.subject_to(x_vars[:, t + 1] == x_next)

            # Control input constraints
            opti.subject_to(opti.bounded(-self.vehicle_dynamics.dynamics['max_deceleration'], u_vars[0, t], self.vehicle_dynamics.dynamics['max_acceleration']))
            opti.subject_to(opti.bounded(-self.vehicle_dynamics.dynamics['max_steering_rate'], u_vars[1, t], self.vehicle_dynamics.dynamics['max_steering_rate']))
            
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
        optimal_acceleration = optimal_control[1, 0]
        optimal_steering = optimal_control[0, 0]

        # Convert the steering angle to the corresponding steering wheel angle
        steering_wheel_angle = optimal_steering

        return optimal_acceleration, steering_wheel_angle

class MPCController(Component):
    def __init__(self, vehicle_interface=None, **args):
        self.vehicle_dynamics = VehicleDynamics()
        self.MPC = MPC(self.vehicle_dynamics, **args)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 10.0

    def state_inputs(self):
        return ['vehicle', 'trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        self.MPC.set_path(trajectory)
        acceleration, steering_wheel_angle = self.MPC.compute(vehicle)
        print("acceleration: ", acceleration)
        print("steering_wheel_angle", steering_wheel_angle)
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(acceleration, steering_wheel_angle, vehicle))

    def healthy(self):
        return self.MPC.path is not None
    