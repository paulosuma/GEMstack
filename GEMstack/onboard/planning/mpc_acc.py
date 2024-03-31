from ...utils import settings
from ...mathutils import transforms
from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.agent import AgentState
from ...state.trajectory import Path,Trajectory,compute_headings
from ...knowledge.vehicle.geometry import front2steer
from ..interface.gem import GEMVehicleCommand
from ..component import Component
import numpy as np
import casadi as ca

class ACCMPC(object):
    """Implements a mpc controller on a second-order Dubins vehicle with daptive cruise control."""
    def __init__(self, horizon_steps = None, desired_speed = None, min_leading_dist = None, desired_time_headway = None, sim_dt = None, delay = None):
        self.horizon_steps = horizon_steps if horizon_steps is not None else settings.get('control.mpc_acc.horizon_steps',4.0)
        self.min_leading_dist = min_leading_dist if min_leading_dist is not None else settings.get('control.mpc_acc.min_leading_dist', 7.0) # m
        self.desired_time_headway = desired_time_headway if desired_time_headway is not None else settings.get('control.mpc_acc.desired_time_headway', 1.5) # s
        self.sim_dt = sim_dt if sim_dt is not None else settings.get('control.mpc_acc.sim_dt', 0.2)
        self.delay = delay if delay is not None else settings.get('control.mpc_acc.delay', 0.5)

        self.front_wheel_angle_scale = 3.0
        self.wheelbase  = settings.get('vehicle.geometry.wheelbase')
        self.wheel_angle_range = [settings.get('vehicle.geometry.min_wheel_angle'),settings.get('vehicle.geometry.max_wheel_angle')]
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')]
        
        self.desired_speed = desired_speed if desired_speed is not None else settings.get('control.mpc_acc.desired_speed')
        self.speed_limit = settings.get('vehicle.limits.max_speed')
        self.max_accel = settings.get('vehicle.limits.max_acceleration') # m/s^2
        self.max_decel = settings.get('vehicle.limits.max_deceleration') # m/s^2
        self.min_jerk = settings.get('control.mpc_acc.min_jerk', -2.0) # m/s^3
        self.max_jerk = settings.get('control.mpc_acc.max_jerk', 2.0) # m/s^3

        self.t_last = None
        self.leading_accel = 0
        self.leading_vehicle = None

    def set_leading_vehicle(self, leading_vehicle : AgentState):
        t = leading_vehicle.pose.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last
        
        if self.leading_vehicle:
            self.leading_accel = (leading_vehicle.velocity - self.leading_vehicle.velocity) / dt

        self.leading_vehicle = leading_vehicle
    
    def compute(self, state : VehicleState, component : Component = None):
        assert state.pose.frame != ObjectFrameEnum.CURRENT
  
        curr_x = state.pose.x
        curr_y = state.pose.y
        curr_yaw = state.pose.yaw if state.pose.yaw is not None else 0.0
        speed = state.v

        leading_x = self.leading_vehicle.pose.x
        leading_y = self.leading_vehicle.pose.y
        leading_speed = self.leading_vehicle.velocity

        # State
        # The state X is made up of the distance between the vehicle and the leading vehicle,
        # the velocity of the vehicle, the relative velocity (leading - our vehicle), 
        # acceleration, and jerk
        # The control is the acceleration (for now, TODO add lane-keeping)
        n_x = 5
        n_u = 1
        x = ca.MX.sym('x', n_x, self.horizon_steps+1)
        u = ca.MX.sym('u', n_u, self.horizon_steps)

        # Initial constraints (assume 0 acceleration at start; TODO is this true?)
        g = []
        initial_dist = np.sqrt((curr_x - leading_x)**2 + (curr_y - leading_y)**2)
        g.append(x[:,0] - np.array([initial_dist, speed, leading_speed - speed, 0, 0]))

        # Objective function
        obj = 0

        # Dynamics constraints
        A = np.array([[1.0, 0.0, self.sim_dt, -0.5 * self.sim_dt**2, 0.0],
                      [0.0, 1.0, 0.0, self.sim_dt, 0.0],
                      [0.0, 0.0, 1.0, -1.0 * self.sim_dt, 0.0],
                      [0.0, 0.0, 0.0, 1.0 - self.sim_dt / self.delay, 0.0],
                      [0.0, 0.0, 0.0, -1.0 / self.delay, 0.0]])
        B = np.array([0.0, 0.0, 0.0, self.sim_dt / self.delay, 1.0 / self.delay]).T
        G = np.array([0.5 * self.sim_dt**2, 0.0, self.sim_dt, 0.0, 0.0]).T
        for k in range(self.horizon_steps):
            x_next = ca.mtimes(A, x[:, k]) + ca.mtimes(B, u[:, k] + G * [self.leading_accel])
            g.append(x[:,k+1] - x_next)
        
        # TODO objective function


class ACCTrajectoryPlanner(Component):
    def __init__(self, vehicle_interface=None, **args):
        self.acc_mpc = ACCMPC(**args)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 5.0

    def state_inputs(self):
        return ['vehicle']

    def state_outputs(self):
        return []

    def update(self, vehicle : VehicleState, leading_vehicle : AgentState):
        self.acc_mpc.set_leading_vehicle(leading_vehicle)
        accel,wheel_angle = self.acc_mpc.compute(vehicle, self)
        #print("Desired wheel angle",wheel_angle)
        steering_angle = np.clip(front2steer(wheel_angle), self.pure_pursuit.steering_angle_range[0], self.pure_pursuit.steering_angle_range[1])
        #print("Desired steering angle",steering_angle)
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))
    
    def healthy(self):
        return self.pure_pursuit.path is not None