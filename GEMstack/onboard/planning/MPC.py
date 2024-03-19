import numpy as np
import casadi as ca
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.trajectory import Path, Trajectory, compute_headings
from ..component import Component

class MPC(object):
    def __init__(self, dt, horizon, Q, R):
        self.dt = dt
        self.horizon = horizon
        self.Q = Q
        self.R = R

        # Define the vehicle geometries (this can be found in the geometry files in knowledge/vehicle)
        self.L = 1.75

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

        # Define the vehicle model equations
        self.model_equations = self.get_model_equations()

        # Define the cost function
        self.cost_function = self.get_cost_function()

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

    def get_model_equations(self):
        # Implement the kinematic bicycle model equations using CasADi
        x_next = self.x + self.v * ca.cos(self.theta) * self.dt
        y_next = self.y + self.v * ca.sin(self.theta) * self.dt
        theta_next = self.theta + (self.v / self.L) * ca.tan(self.delta) * self.dt
        v_next = self.v + self.a * self.dt
        return ca.vertcat(x_next, y_next, theta_next, v_next)

    def get_cost_function(self):
        # Implement the cost function using CasADi
        cost = 0
        for t in range(self.horizon):
            cost += (self.x_ref[t] - self.x)**2 + (self.y_ref[t] - self.y)**2
            cost += (self.theta_ref[t] - self.theta)**2 + (self.v_ref[t] - self.v)**2
            cost += self.delta[t]**2 + self.a[t]**2
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

        # Set up the optimization problem
        opti = ca.Opti()
        x_vars = opti.variable(4, self.horizon + 1)  # State variables
        u_vars = opti.variable(2, self.horizon)  # Control variables

        # Set initial conditions
        opti.subject_to(x_vars[:, 0] == current_state)

        # Set up the optimization problem
        # TODO: need to put the dynamic constraints here
        for t in range(self.horizon):
            # Model equations constraints
            x_next = self.model_equations(x_vars[:, t], u_vars[:, t])
            opti.subject_to(x_vars[:, t + 1] == x_next)

            # Control input constraints
            opti.subject_to(opti.bounded(-self.max_steering, u_vars[0, t], self.max_steering))
            opti.subject_to(opti.bounded(-self.max_acceleration, u_vars[1, t], self.max_acceleration))

        # Set the objective function
        obj = self.cost_function(x_vars, u_vars)
        opti.minimize(obj)

        # Set up the solver
        opti.solver('ipopt')

        # Set the reference trajectory
        opti.set_value(self.x_ref, ref_trajectory.eval(self.current_traj_parameter + self.dt * np.arange(self.horizon))[0])
        opti.set_value(self.y_ref, ref_trajectory.eval(self.current_traj_parameter + self.dt * np.arange(self.horizon))[1])
        opti.set_value(self.theta_ref, ref_trajectory.eval_derivative(self.current_traj_parameter + self.dt * np.arange(self.horizon))[0])
        opti.set_value(self.v_ref, ref_trajectory.eval_derivative(self.current_traj_parameter + self.dt * np.arange(self.horizon))[1])

        # Solve the optimization problem
        sol = opti.solve()

        # Extract the optimal control inputs
        optimal_control = sol.value(u_vars)

        # Extract the optimal steering angle and acceleration
        optimal_steering = optimal_control[0, 0]
        optimal_acceleration = optimal_control[1, 0]

        # Convert the steering angle to the corresponding steering wheel angle
        steering_wheel_angle = optimal_steering

        return optimal_acceleration, steering_wheel_angle

class MPCController(Component):
    def __init__(self, vehicle_interface=None, **args):
        self.MPC = MPC(**args)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 50.0

    def state_inputs(self):
        return ['vehicle', 'trajectory']

    def state_outputs(self):
        return []

    def update(self, vehicle: VehicleState, trajectory: Trajectory):
        self.MPC.set_path(trajectory)
        acceleration, steering_wheel_angle = self.MPC.compute(vehicle)
        self.vehicle_interface.send_command(acceleration=acceleration,
                                            steering_wheel_angle=steering_wheel_angle)

    def healthy(self):
        return self.MPC.path is not None



