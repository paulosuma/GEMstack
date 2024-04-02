from ...utils import settings
from ...mathutils import transforms
from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
from ...state.all import AllState
from ...state.vehicle import VehicleState, ObjectFrameEnum
from ...state.agent import AgentState, AgentEnum
from ...state.trajectory import Path,Trajectory,compute_headings
from ...knowledge.vehicle.geometry import front2steer
from ..interface.gem import GEMVehicleCommand
from ..component import Component
import numpy as np
import casadi as ca

class ACCMPC(object):
    """Implements a mpc controller on a second-order Dubins vehicle with daptive cruise control."""
    def __init__(self, horizon_steps = None, desired_speed = None, min_leading_dist = None, desired_time_headway = None, sim_dt = None, delay = None):
        self.horizon_steps = horizon_steps if horizon_steps is not None else settings.get('control.mpc_acc.horizon_steps',10)
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

        leading_x = self.leading_vehicle.pose.x if self.leading_vehicle else np.inf
        leading_y = self.leading_vehicle.pose.y if self.leading_vehicle else np.inf
        leading_speed = self.leading_vehicle.velocity if self.leading_vehicle else np.inf

        # Make the optimization problem
        opti = ca.Opti()

        # State
        # The state X is made up of the distance between the vehicle and the leading vehicle,
        # the velocity of the vehicle, the relative velocity (leading - our vehicle), 
        # acceleration, and jerk
        # The control is the acceleration (for now, TODO add lane-keeping)
        n_x = 5
        n_u = 1
        x = opti.variable(n_x, self.horizon_steps+1)
        u = opti.variable(n_u, self.horizon_steps)

        # Initial constraints (assume 0 acceleration at start; TODO is this true?)
        initial_dist = np.sqrt((curr_x - leading_x)**2 + (curr_y - leading_y)**2)
        opti.subject_to(x[:,0] == [initial_dist, speed, leading_speed - speed, 0, 0])

        # Objective function
        obj = 0

        # Dynamics constraints per timestep
        for k in range(self.horizon_steps):
            dist_next = x[0,k] + self.sim_dt * x[2,k] - 0.5 * (self.sim_dt**2) * x[3,k] + 0.5 * (self.sim_dt**2) * self.leading_accel
            v_next = x[1,k] + self.sim_dt * x[3,k]
            rel_v_next = x[2,k] - self.sim_dt * x[2,k] + self.sim_dt * self.leading_accel
            a_next = (1.0 - self.sim_dt / self.delay) * x[3,k] + (self.sim_dt / self.delay) * u[0,k]
            j_next = (-1.0 / self.delay) * x[4,k] + (1.0 / self.delay) * u[0,k]

            opti.subject_to(x[0,k+1] == dist_next)
            opti.subject_to(x[1,k+1] == v_next)
            opti.subject_to(x[2,k+1] == rel_v_next)
            opti.subject_to(x[3,k+1] == a_next)
            opti.subject_to(x[4,k+1] == j_next)
        
            # Objective function
            # Minimize the difference between the actual and desired distance between cars,
            # the relative velocity of the cars, and the magnitude of acceleration and jerk
            desired_dist = self.min_leading_dist + self.desired_time_headway * x[1,k]
            dist_error = x[0, k] - desired_dist
            obj += ca.sumsqr(dist_error) + ca.sumsqr(x[2,k]) + ca.sumsqr(x[3, k]) + ca.sumsqr(x[4, k])

        # Hard constraints
        # Constrain the velocity, acceleration, deceleration, and jerk
        opti.subject_to(opti.bounded(0, x[1,:], self.desired_speed)) # 0 <= velocity <= max speed
        opti.subject_to(opti.bounded(self.max_decel, x[3,:], self.max_accel)) # a_min <= accelereration <= a_max
        opti.subject_to(opti.bounded(self.min_jerk, x[4,:], self.max_jerk)) # j_min <= jerk <= j_max

        # Solve the NLP
        opti.minimize(obj)
        opti.solver('ipopt')
        sol = opti.solve()

        # Extract optimal control and state trajectories
        optimal_u = sol.value(u[0,:])
        optimal_x = sol.value(x)

        return optimal_u, optimal_x

class ACCTrajectoryPlanner(Component):
    def __init__(self, vehicle_interface=None, **args):
        self.acc_mpc = ACCMPC(**args)
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 5.0

    def state_inputs(self):
        return ['all']

    def state_outputs(self):
        return []

    def update(self, state : AllState):
        vehicle = state.vehicle
        agents = state.agents
        leading = None # TODO figure out the best way to get the leading car
        for k, a in agents.items():
            if a.type == AgentEnum.CAR:
                leading = a

        if leading is not None:
            self.acc_mpc.set_leading_vehicle(leading)
        
        accel, _ = self.acc_mpc.compute(vehicle, self)
        wheel_angle = 0 # TODO
        #print("Desired wheel angle",wheel_angle)
        steering_angle = np.clip(front2steer(wheel_angle), self.acc_mpc.steering_angle_range[0], self.acc_mpc.steering_angle_range[1])
        #print("Desired steering angle",steering_angle)
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel,steering_angle, vehicle))
    
    def healthy(self):
        return self.acc_mpc.leading_vehicle is not None # TODO figure out what this should be