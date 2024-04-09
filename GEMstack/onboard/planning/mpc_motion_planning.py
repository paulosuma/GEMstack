from ...mathutils.control import PID
from ...utils import settings
from ...mathutils import transforms
from ...knowledge.vehicle.dynamics import acceleration_to_pedal_positions
from ...state import AllState,VehicleState,Route,ObjectFrameEnum,Roadmap,Roadgraph
from ...state.vehicle import VehicleState,ObjectFrameEnum
from ...state.trajectory import Path,Trajectory,compute_headings
from ...state.agent import AgentEnum
from ...knowledge.vehicle.geometry import front2steer
from ..interface.gem import GEMVehicleCommand
from ..component import Component
import numpy as np
import casadi as ca

class MPC(object):
    def __init__(self):
        self.horizon_steps = settings.get('model_predictive_controller.horizon_steps', 10)
        self.dt = settings.get('model_predictive_controller.dt', 0.1) # s

        self.wheelbase  = settings.get('vehicle.geometry.wheelbase') # m
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')] # radians

        self.max_speed = settings.get('vehicle.limits.max_speed') # m/s
        self.max_reverse_speed = settings.get('vehicle.limits.max_reverse_speed') # m/s
        self.max_accel = settings.get('vehicle.limits.max_acceleration') # m/s^2
        self.max_decel = settings.get('vehicle.limits.max_deceleration') # m/s^2

        self.min_dist = settings.get('model_predictive_controller.min_dist', 10.0) # meters
        self.time_headway = settings.get('model_predictive_controller.time_headway', 5.0) # seconds
        self.front_degree_range = settings.get('model_predictive_controller.front_degree_range', 15.0) # degrees

    def compute(self, x0, y0, theta0, v0, x_goal, y_goal, theta_goal, v_goal, obstacles):
        """
        Setup and solve the MPC problem.
        Returns the first control inputs (v, delta) from the optimized trajectory.
        """

        v0 = np.clip(v0, -1 * self.max_reverse_speed, self.max_speed) # TODO: clip to avoid constrain issues, should be handled more carefully

        opti = ca.Opti()  # Create an optimization problem

        # State
        # TODO: Add second order dynamics for steering angle
        X = opti.variable(4, self.horizon_steps+1)  # [x, y, theta, v]
        # X = opti.variable(5, N+1)  # [x, y, theta, v, delta]
        U = opti.variable(2, self.horizon_steps)    # [a, delta/omega]

        # Initial constraints
        opti.subject_to(X[:,0] == [x0, y0, theta0, v0])
        # opti.subject_to(X[:,0] == [x0, y0, theta0, v0, delta0])

        # Dynamics constraints
        obstacle_penalty = 0
        for k in range(self.horizon_steps):
            x_next = X[0,k] + X[3,k] * ca.cos(X[2,k]) * self.dt
            y_next = X[1,k] + X[3,k] * ca.sin(X[2,k]) * self.dt
            v_next = X[3,k] + U[0,k] * self.dt
            theta_next = X[2,k] + X[3,k]/self.wheelbase * ca.tan(U[1,k]) * self.dt
            # theta_next = X[2,k] + X[3,k]/L*ca.tan(X[4,k])*dt
            # delta_next = X[4,k] + U[1,k]*dt
            
            opti.subject_to(X[0,k+1] == x_next)
            opti.subject_to(X[1,k+1] == y_next)
            opti.subject_to(X[2,k+1] == theta_next)
            opti.subject_to(X[3,k+1] == v_next)
            # opti.subject_to(X[4,k+1] == delta_next)

            # Obstacle constraints
            # TODO: Add soft constraints
            penalty_scale = 2000
            for obs in obstacles:
                obs_type, obs_x, obs_y, obs_vx, obs_vy, obs_w, obs_l, obs_h = obs
                obs_x = obs_x + (obs_vx * self.dt * k)
                obs_y = obs_y + (obs_vy * self.dt * k)

                # Get the displacement between us and them
                disp_x = obs_x - X[0, k]
                disp_y = obs_y - X[1, k]
                disp_angle = ca.atan2(disp_y, disp_x) * 180.0 / ca.pi

                # Calculate distance
                distance_squared = disp_x**2 + disp_y**2

                # Check if the obstacle is a car
                is_car = (obs_type == AgentEnum.CAR)

                # Check if they are in front of us (within range)
                in_front = ca.fabs(disp_angle - X[2, k]) <= self.front_degree_range

                # Condition to apply car penalty
                car_in_front = ca.logic_and(is_car, in_front)

                # Keep a constant time headway distance based on current speed
                desired_dist = self.min_dist + self.time_headway * X[3, k]
                real_dist = ca.sqrt(distance_squared)

                # Calculate penalty for car in front within desired distance
                car_penalty = ca.sumsqr(desired_dist - real_dist)
                
                # Calculate default penalty for other cases
                default_penalty = penalty_scale / (distance_squared + 1)

                # Apply car penalty if condition is met, otherwise apply default penalty
                penalty_to_apply = ca.if_else(car_in_front, car_penalty, default_penalty)

                # Add the appropriate penalty
                obstacle_penalty += penalty_to_apply


        # Control costraints
        opti.subject_to(opti.bounded(-1 * self.max_decel, U[0,:], self.max_accel))
        opti.subject_to(opti.bounded(self.steering_angle_range[0], U[1,:], self.steering_angle_range[1]))
        opti.subject_to(opti.bounded(-1 * self.max_reverse_speed, X[3,:], self.max_speed))
        # opti.subject_to(opti.bounded(omega_min, U[1,:], omega_max))
        # opti.subject_to(opti.bounded(delta_min, X[4,:], delta_max))
        

        # objective
        # objective = ca.sumsqr(X[0:4,-1] - [x_goal, y_goal, theta_goal, v_goal])
        objective = ca.sumsqr(X[0:4,-1] - [x_goal, y_goal, theta_goal, v_goal]) + obstacle_penalty
        opti.minimize(objective)

        # Solver
        opts = {"ipopt.print_level": 0, "print_time": 0}
        opti.solver("ipopt", opts)

        sol = opti.solve()
        x_sol = sol.value(X[0,:])
        y_sol = sol.value(X[1,:])
        theta_sol = sol.value(X[2,:])
        v_sol = sol.value(X[3,:])[0]
        delta_sol = sol.value(U[1,:])[0]
        acc_sol = sol.value(U[0,:])[0]
        # delta_sol = sol.value(X[4,:])[0]
        # omega_sol = sol.value(U[1,:])[0]

        return delta_sol, acc_sol

class MPCTrajectoryPlanner(Component):
    def __init__(self,vehicle_interface=None, **args):
        self.mpc = MPC()
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return 10.0

    def state_inputs(self):
        return ['all']

    def state_outputs(self):
        return ['trajectory']

    def update(self, state : AllState):
        agents = state.agents
        vehicle = state.vehicle
        route = state.route
        x_start, y_start, theta_start, v_start = vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, vehicle.v
        x_goal, y_goal, theta_goal, v_goal = *route.points[-1], 0., 0.

        agents = [a.to_frame(ObjectFrameEnum.START, start_pose_abs=state.start_vehicle_pose) for a in agents.values()]
        agents = [[a.type, a.pose.x, a.pose.y, a.velocity[0], a.velocity[1], *a.dimensions] for a in agents]

        wheel_angle, accel = self.mpc.compute(x_start, y_start, theta_start, v_start, \
                                              x_goal, y_goal, theta_goal, v_goal, agents)
        print(f"Wheel angle: {wheel_angle}, Acceleration: {accel}")

        if np.sqrt((x_start - x_goal)**2 + (y_start - y_goal)**2) <= 0.1: # threshold for reaching the goal
            wheel_angle = 0
            accel = 0
        steering_angle = np.clip(front2steer(wheel_angle), self.mpc.steering_angle_range[0], self.mpc.steering_angle_range[1])
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel, steering_angle, vehicle))


    def healthy(self):
        return True