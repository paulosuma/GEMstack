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
        self.horizon_steps = settings.get('model_predictive_controller.horizon_steps', 2)
        self.dt = settings.get('model_predictive_controller.dt', 0.1) # s
        self.rate = settings.get('model_predictive_control.rate', 10.0) # Hz

        self.wheelbase  = settings.get('vehicle.geometry.wheelbase') # m
        self.steering_angle_range = [settings.get('vehicle.geometry.min_steering_angle'),settings.get('vehicle.geometry.max_steering_angle')] # radians

        self.max_speed = settings.get('vehicle.limits.max_speed') # m/s
        self.max_reverse_speed = settings.get('vehicle.limits.max_reverse_speed') # m/s
        self.max_accel = settings.get('vehicle.limits.max_acceleration') # m/s^2
        self.max_decel = settings.get('vehicle.limits.max_deceleration') # m/s^2

        self.min_dist = settings.get('model_predictive_controller.min_dist', 5.0) # meters
        self.min_obst_dist = settings.get('model_predictive_controller.min_obst_dist', 3.0) # meters
        self.time_headway = settings.get('model_predictive_controller.time_headway', 5.0) # seconds
        self.front_degree_range = settings.get('model_predictive_controller.front_degree_range', 10.0) # degrees
        self.front_relevance_dist = settings.get('model_predictive_controller.front_relevance_dist', 50.0) # meters

        self.lane_penalty_constant = settings.get('model_predictive_controller.lane_penalty_constant', 1000)
        self.lane_centerline = None # list of tuples or None

        self.healthy = True # Track whether a solution was found

    def compute(self, x0, y0, theta0, v0, obstacles):
        """
        Setup and solve the MPC problem.
        Returns the first control inputs (v, delta) from the optimized trajectory.

        Lane keeping: get the closest lane by centerline. Then, at each point in the horizon,
        get the segment closest to the current point and try to aim the yaw toward that line
        segment.
        """

        v0 = np.clip(v0, -1 * self.max_reverse_speed, self.max_speed) # TODO: clip to avoid constrain issues, should be handled more carefully

        opti = ca.Opti()  # Create an optimization problem

        # State
        X = opti.variable(4, self.horizon_steps+1)  # [x, y, theta, v]
        U = opti.variable(2, self.horizon_steps)    # [a, delta/omega]

        # Initial constraints
        opti.subject_to(X[:,0] == [x0, y0, theta0, v0])

        # Dynamics constraints
        obstacle_penalty = 0
        lane_penalty = 0
        for k in range(self.horizon_steps):
            x_next = X[0,k] + X[3,k] * ca.cos(X[2,k]) * self.dt
            y_next = X[1,k] + X[3,k] * ca.sin(X[2,k]) * self.dt
            v_next = X[3,k] + U[0,k] * self.dt
            theta_next = X[2,k] + X[3,k]/self.wheelbase * ca.tan(U[1,k]) * self.dt
            theta_next = self.wrap_to_pi(theta_next)
            
            opti.subject_to(X[0,k+1] == x_next)
            opti.subject_to(X[1,k+1] == y_next)
            opti.subject_to(X[2,k+1] == theta_next)
            opti.subject_to(X[3,k+1] == v_next)

            # Lane keeping
            if self.lane_centerline:
                min_dist = ca.inf
                best_yaw = 0

                px, py = X[0, k], X[1, k]
                for i in range(len(self.lane_centerline)-1):
                    ax, ay, _ = self.lane_centerline[i]
                    bx, by, _ = self.lane_centerline[i+1]

                    dist, point = self.segment_distance(px, py, ax, ay, bx, by)
                    segment_yaw = ca.atan2(by - ay, bx - ax)

                    update_cond = dist < min_dist
                    min_dist = ca.if_else(update_cond, dist, min_dist)
                    best_yaw = ca.if_else(update_cond, segment_yaw, best_yaw)
                
                lane_penalty += ca.sumsqr(X[2,k] - best_yaw)
                lane_penalty += ca.sumsqr(px - point[0]) + ca.sumsqr(py - point[1])


            # Obstacle constraints
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
                is_car = (obs_type == AgentEnum.CAR or obs_type == AgentEnum.LARGE_TRUCK or obs_type == AgentEnum.MEDIUM_TRUCK)

                # Check if they are in front of us (within range)
                in_front = ca.logic_and(ca.fabs(disp_angle - self.wrap_to_pi(X[2, k])) <= self.front_degree_range,
                                        distance_squared <= self.front_relevance_dist**2)

                # Condition to apply car penalty
                car_in_front = ca.logic_and(is_car, in_front)

                # Keep a constant time headway distance based on current speed
                desired_dist = self.min_dist + self.time_headway * X[3, k]
                real_dist = ca.sqrt(distance_squared)

                # Calculate penalty for car in front within desired distance
                car_penalty = ca.sumsqr(desired_dist - real_dist)

                # Apply car penalty if condition is met, otherwise apply default penalty
                penalty_to_apply = ca.if_else(car_in_front, car_penalty, 0)

                # Add the appropriate penalty
                obstacle_penalty += penalty_to_apply

                opti.subject_to(distance_squared >= self.min_obst_dist**2)


        # Control costraints
        opti.subject_to(opti.bounded(-1 * self.max_decel, U[0,:], self.max_accel))
        opti.subject_to(opti.bounded(self.steering_angle_range[0], U[1,:], self.steering_angle_range[1]))
        opti.subject_to(opti.bounded(-1 * self.max_reverse_speed, X[3,:], self.max_speed))

        # Objective
        objective = obstacle_penalty + lane_penalty
        opti.minimize(objective)

        # Solver
        opts = {"ipopt.print_level": 0, "print_time": 0}
        opti.solver("ipopt", opts)

        try:
            sol = opti.solve()
        except:
            self.healthy = False
            return 0, 0

        print("X", sol.value(X[0,:]))
        print("Y", sol.value(X[1,:]))
        print("T", sol.value(X[2,:]))
        print("V", sol.value(X[3,:]))
        delta_sol = sol.value(U[1,:])[0]
        acc_sol = sol.value(U[0,:])[0]

        return delta_sol, acc_sol

    def wrap_to_pi(self, angle):
        """Wrap angle in radians to [-pi, pi]"""
        return ca.fmod(angle + ca.pi, 2 * ca.pi) - ca.pi
    
    def segment_distance(self, px, py, ax, ay, bx, by):
        # Create CasADi variables for input
        p = ca.vertcat(px, py)
        a = ca.vertcat(ax, ay)
        b = ca.vertcat(bx, by)
        
        # Calculate vectors
        ab = b - a
        ap = p - a
        
        # Project point onto line segment
        ab_mag = ca.dot(ab, ab)
        proj_scalar = ca.dot(ap, ab) / ab_mag
        proj_point = a + ca.fmax(ca.fmin(proj_scalar, 1), 0) * ab
        
        # Distance to the closest point on the segment
        distance = ca.norm_2(p - proj_point)
        
        return distance, proj_point

class MPCTrajectoryPlanner(Component):
    def __init__(self,vehicle_interface=None, **args):
        self.mpc = MPC()
        self.vehicle_interface = vehicle_interface

    def rate(self):
        return self.mpc.rate

    def state_inputs(self):
        return ['all']

    def state_outputs(self):
        return ['trajectory']

    def update(self, state : AllState):
        agents = state.agents
        vehicle = state.vehicle.to_frame(ObjectFrameEnum.ABSOLUTE_CARTESIAN, start_pose_abs=state.start_vehicle_pose)
        x_start, y_start, theta_start, v_start = vehicle.pose.x, vehicle.pose.y, vehicle.pose.yaw, vehicle.v

        agents = [a.to_frame(ObjectFrameEnum.ABSOLUTE_CARTESIAN, start_pose_abs=state.start_vehicle_pose) for a in agents.values()]
        agents = [[a.type, a.pose.x, a.pose.y, a.velocity[0], a.velocity[1], *a.dimensions] for a in agents]

        curr_lane = state.roadgraph.get_current_lane(state.vehicle)
        if curr_lane:
            # Get the current segment
            min_dist = float('inf')
            seg_idx = 0
            for idx, seg in enumerate(state.roadgraph.lanes[curr_lane].center.segments):
                for i in range(len(seg)-1):
                    dist, _ = transforms.point_segment_distance((x_start, y_start, 0), seg[i], seg[i+1])
                    if dist < min_dist:
                        min_dist = dist
                        seg_idx = idx

            self.mpc.lane_centerline = state.roadgraph.lanes[curr_lane].center.segments[seg_idx]

        wheel_angle, accel = self.mpc.compute(x_start, y_start, theta_start, v_start, agents)
        print(f"Wheel angle: {wheel_angle}, Acceleration: {accel}")

        steering_angle = np.clip(front2steer(wheel_angle), self.mpc.steering_angle_range[0], self.mpc.steering_angle_range[1])
        self.vehicle_interface.send_command(self.vehicle_interface.simple_command(accel, steering_angle, vehicle))


    def healthy(self):
        return self.mpc.healthy