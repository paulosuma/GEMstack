import numpy as np
import casadi as ca
import cvxpy as cp
from ...state.vehicle import VehicleState,ObjectFrameEnum
from ...state.trajectory import Path,Trajectory,compute_headings
from ..component import Component

# class VehicleModel:
#     def __init__(self, dt):
#         self.dt = dt
#         # Define model parameters and state space matrices

#     def predict(self, state, control_input, horizon):
#         # Implement the vehicle model prediction over the specified horizon
#         # Return the predicted states
#         return

# TODO: be mindful when calculating the positions (be careful about object frame vs global frame)
class MPC(object):
    def __init__(self, lookahead = None, lookahead_scale = None, crosstrack_gain = None, desired_speed = None):
        return

    """ 
    Predicts/computes the next state given the control input, i.e., x1 = f(x, u) where the control input u
    consists of the acceleration and the steering angle 
    """
    def predict(self, vehicle : VehicleState, a : float, delta : float):
        x = vehicle.pose.x
        y = vehicle.pose.y
        v = vehicle.v
        steering_wheel_angle = vehicle.steering_wheel_angle
        front_wheel_angle = vehicle.front_wheel_angle

    def set_path(self, path : Path):
        if path == self.path_arg:
            return
        self.path_arg = path
        if len(path.points[0]) > 2:
            path = path.get_dims([0,1])
        if not isinstance(path,Trajectory):
            self.path = path.arc_length_parameterize()
            self.trajectory = None
            self.current_traj_parameter = 0.0
            if self.desired_speed_source in ['path','trajectory']:
                raise ValueError("Can't provide an untimed path to PurePursuit and expect it to use the path velocity. Set control.pure_pursuit.desired_speed to a constant.")
        else:
            self.path = path.arc_length_parameterize()
            self.trajectory = path
            self.current_traj_parameter = self.trajectory.domain()[0]
        self.current_path_parameter = 0.0
    
    def compute(self, state : VehicleState, component : Component = None):
        return 
    


class MPCController(Component):
    def __init__(self, vehicle_interface=None, **args):
        # self.vehicle_interface = vehicle_interface
        # self.dt = dt
        # self.horizon = horizon
        # self.Q = Q  # Cost matrix for state deviation
        # self.R = R  # Cost matrix for control effort
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
        # # Get the current vehicle state
        # current_state = self.get_current_state(vehicle)

        # # Formulate the MPC optimization problem
        # states = cp.Variable((self.model.state_dim, self.horizon + 1))
        # control_inputs = cp.Variable((self.model.control_dim, self.horizon))

        # cost = 0
        # constraints = []

        # for t in range(self.horizon):
        #     # Compute the reference state from the trajectory
        #     ref_state = self.get_reference_state(trajectory, t)

        #     # Define the cost function
        #     cost += cp.quad_form(states[:, t] - ref_state, self.Q)
        #     cost += cp.quad_form(control_inputs[:, t], self.R)

        #     # Define the vehicle model constraints
        #     next_state = self.model.predict(states[:, t], control_inputs[:, t], 1)
        #     constraints += [states[:, t + 1] == next_state]

        #     # Define state and control input constraints
        #     constraints += [states[:, t] <= self.model.state_max]
        #     constraints += [states[:, t] >= self.model.state_min]
        #     constraints += [control_inputs[:, t] <= self.model.control_max]
        #     constraints += [control_inputs[:, t] >= self.model.control_min]

        # # Solve the MPC optimization problem
        # problem = cp.Problem(cp.Minimize(cost), constraints)
        # problem.solve()

        # # Get the optimal control input for the current step
        # optimal_control_input = control_inputs.value[:, 0]

        # # Send the control command to the vehicle interface
        # self.vehicle_interface.send_command(optimal_control_input, vehicle)

        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(0.5, 0.5, vehicle))

    def get_current_state(self, vehicle):
        # Extract the current state from the vehicle state
        # Return the current state vector
        return

    def healthy(self):
        return self.MPC.path is not None

    # def get_reference_state(self, trajectory, t):
    #     # Compute the reference state from the trajectory at time step t
    #     # Return the reference state vector

    # def send_command(self, control_input, vehicle):
    #     # Convert the control input to the appropriate format
    #     # Send the control command to the vehicle interface