from typing import List
from ..component import Component
from ...state import AllState, VehicleState, EntityRelation, EntityRelationEnum, Path, Trajectory, Route, ObjectFrameEnum
from ...utils import serialization
from ...mathutils.transforms import vector_madd, vector_dist
import math

# Three helper functions to get plaaning time (Should be moved to mathutils?)
def solve_quadratic(a, b, c):
    # Calculate the discriminant
    discriminant = b**2 - 4*a*c

    # Check if the discriminant is positive, negative, or zero
    if discriminant >= 0:
        # Two real and distinct roots
        root1 = (-b + math.sqrt(discriminant)) / (2*a)
        root2 = (-b - math.sqrt(discriminant)) / (2*a)
        return root1, root2
    else:
        raise ValueError('Discriminant should not be negative')


def get_time_by_formula(acceleration, speed, dist):
    # formula: 1/2at^2 + vt - dist = 0
    time, _ = solve_quadratic(0.5*acceleration, speed, -dist)
    return time

def get_dist_by_time(acceleration, speed, time):
    # formula: 1/2at^2 + vt - dist = 0
    return speed * time + 0.5 * acceleration * time ** 2
    
def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def longitudinal_plan(path: Path, acceleration: float, deceleration: float, max_speed: float, current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for a path with a
    trapezoidal velocity profile.

    1. accelerates from current speed toward max speed
    2. travel along max speed
    3. if at any point you can't brake before hitting the end of the path,
       decelerate with accel = -deceleration until velocity goes to 0.
    """
    path_normalized = path.arc_length_parameterize()
    # TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    # cont_points = [p[0] + (p[1] - p[0]) / 100.0 * i for i in range(100)]
    times = [t for t in path_normalized.times]

    print('(longitudinal_plan) input current_speed:', current_speed)
    # print("(longitudinal_plan) before_points:", points)
    # print("(longitudinal_plan) before_times:", times)

    # current_speed = min(current_speed, max_speed)

    # Condition 1: Hitting the end of the path,
    #              decelerate with accel = -deceleration until velocity goes to 0.
    if len(points) < 3: # less than 3 waypoints, begin to decelerate (design by ourselves)
        traj = longitudinal_brake(path, deceleration, current_speed)


    # time to deceleration
    time_to_stop = (current_speed) / deceleration
    
    dt = 0.05
    time_left = times[-1]
    current_point = points[0][0]
    current_time = times[0]
    end_point = points[-1][0]

    update_points = [points[0]]
    update_times = [current_time]
    while current_point <= end_point:
        current_time += dt

        # move to deceleration
        distance_left = end_point - current_point
        safety_dist = (current_speed**2)/(2*deceleration)
        
        if distance_left <= safety_dist:
            #update
            # current_point += get_dist_by_time(deceleration, current_speed, dt)
            # current_speed += dt * acceleration
            current_point += dt * current_speed
            current_speed -= deceleration * dt
        else:
            #check max speed
            if current_speed < max_speed :
                current_speed += dt * acceleration
                current_point += dt * current_speed
            
            elif current_speed > max_speed :
                current_point += dt * current_speed
                current_speed -= deceleration * dt
            
            else:
                current_point += dt * current_speed

        update_points.append((current_point,0))
        update_times.append(current_time)
    
    while current_speed > 0:
        current_time += dt
        current_point += dt * current_speed
        current_speed -= deceleration * dt
        update_points.append((current_point,0))
        update_times.append(current_time)


    traj = Trajectory(path.frame, update_points, update_times)

    # print("(longitudinal_plan) update_points:", traj.points)
    # print("(longitudinal_plan) update_times:", traj.times)

    return traj


def longitudinal_brake(path: Path, deceleration: float, current_speed: float) -> Trajectory:
    """Generates a longitudinal trajectory for braking along a path."""
    path_normalized = path.arc_length_parameterize()
    # TODO: actually do something to points and times
    points = [p for p in path_normalized.points]
    times = [t for t in path_normalized.times]

    # bug in GemStack code? (len(points) != len(times))
    N = min(len(points), len(times))
    print('(longitudinal_break) input current_speed:', current_speed)
    dt = 0.05
    end_time = times[-1]
    current_point = points[0][0]
    current_time = times[0]
    end_point = points[-1][0]

    update_points = [points[0]]
    update_times = [current_time]
    while current_point <= end_point and current_time < end_time:
        current_time += dt
        if current_speed >= 0.02:
            current_speed -= deceleration * dt
            current_speed = current_speed
        elif current_speed < 0.02:
            current_speed = 0
               
        current_time += dt
        current_point += dt * current_speed
        current_speed = max(current_speed, 0)
        update_points.append((current_point,0))
        update_times.append(current_time)

    while current_speed > 0:
        current_time += dt
        current_point += dt * current_speed
        current_speed -= deceleration * dt
        current_speed = max(current_speed, 0)
        update_points.append((current_point,0))
        update_times.append(current_time)

    
    
    return Trajectory(path.frame, update_points, update_times)


class YieldTrajectoryPlanner(Component):
    """Follows the given route.  Brakes if you have to yield or
    you are at the end of the route, otherwise accelerates to
    the desired speed.
    """

    def __init__(self):
        self.route_progress = None
        self.t_last = None
        self.acceleration = 0.5
        self.desired_speed = 1.0
        self.deceleration = 2.0

    def state_inputs(self):
        return ['all']

    def state_outputs(self) -> List[str]:
        return ['trajectory']

    def rate(self):
        return 10.0

    def update(self, state: AllState):
        vehicle = state.vehicle  # type: VehicleState
        route = state.route   # type: Route
        t = state.t

        if self.t_last is None:
            self.t_last = t
        dt = t - self.t_last

        curr_x = vehicle.pose.x
        curr_y = vehicle.pose.y
        curr_v = vehicle.v

        # figure out where we are on the route
        if self.route_progress is None:
            self.route_progress = 0.0
        closest_dist, closest_parameter = state.route.closest_point_local(
            (curr_x, curr_y), [self.route_progress-5.0, self.route_progress+5.0])
        self.route_progress = closest_parameter

        # extract out a 10m segment of the route
        route_with_lookahead = route.trim(
            closest_parameter, closest_parameter+10.0)

        # parse the relations indicated
        should_brake = False
        for r in state.relations:
            if r.type == EntityRelationEnum.YIELDING and r.obj1 == '':
                # yielding to something, brake
                should_brake = True
        should_accelerate = (not should_brake and curr_v < self.desired_speed)

        # choose whether to accelerate, brake, or keep at current velocity
        if should_accelerate:
            traj = longitudinal_plan(
                route_with_lookahead, self.acceleration, self.deceleration, self.desired_speed, curr_v)
        elif should_brake:
            traj = longitudinal_brake(
                route_with_lookahead, self.deceleration, curr_v)
        else:
            traj = longitudinal_plan(
                route_with_lookahead, 0.0, self.deceleration, self.desired_speed, curr_v)

        return traj