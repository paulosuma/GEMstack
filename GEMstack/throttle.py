import rospy
import sys
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import (
    PositionWithSpeed,
    PacmodCmd,
    SystemRptInt,
    SystemRptFloat,
    VehicleSpeedRpt,
)


# For message format, see
# https://github.com/astuff/astuff_sensor_msgs/blob/3.3.0/pacmod_msgs/msg/PacmodCmd.msg
class ThrottleInput:
    """Your control loop code should go here.  You will use ROS and Pacmod messages
    to communicate with drive-by-wire system to control the vehicle's turn signals.
    """

    def __init__(self, throttle_level):
        # Throttle Pedal Publisher
        self.accel_pub = rospy.Publisher(
            "/pacmod/as_rx/accel_cmd", PacmodCmd, queue_size=1
        )
        # Brake Pedal Publisher
        self.brake_pub = rospy.Publisher(
            "/pacmod/as_rx/brake_cmd", PacmodCmd, queue_size=1
        )
        # Steering Column Publisher
        self.steer_pub = rospy.Publisher(
            "/pacmod/as_rx/steer_cmd", PositionWithSpeed, queue_size=1
        )
        # Speed Subscriber
        self.speed_sub = rospy.Subcriber(
            "/pacmod/parsed_tx/vehicle_speed_rpt",
            VehicleSpeedRpt,
            self.print_speed_to_file,
        )
        # Steering Column Subscriber
        self.steer_sub = rospy.Subcriber(
            "/pacmod/parsed_tx/steer_rpt", SystemRptFloat, self.get_steering_position
        )

        # Member Variables
        self.throttle_level = throttle_level
        self.file = open(f"/logs/accel_log_{throttle_level}.csv", "w")
        self.vehicle_speed = 0
        self.steering_position = -1

    def print_speed_to_file(self, data):
        """Print Vehicle Speed"""
        timestamp = data.header.stamp
        speed = data.vehicle_speed
        valid = data.valid
        if valid:
            self.vehicle_speed = speed
            self.file.write(f"{timestamp.secs*(10**9) + timestamp.nsecs},{speed}\n")

    def get_steering_position(self, data):
        self.steering_position = max(
            abs(data.manual_input), abs(data.command), abs(data.output)
        )

    def rate(self):
        """Requested update frequency, in Hz"""
        return 3

    def initialize(self):
        """Run first"""
        pass

    def cleanup(self):
        """Run last
        Sets throttle pedal position to 0 and brake pedal position to 0.7 for
        5 seconds."""
        rate = rospy.Rate(rate)
        max_calls = 15
        num_calls = 0
        while self.vehicle_speed > 0 and num_calls < max_calls:
            # Set Throttle Pedal Position to 0
            accel_cmd = PacmodCmd()
            accel_cmd.enable = True
            accel_cmd.f64_cmd = 0
            self.accel_pub.publish(accel_cmd)
            # Set Brake Pedal Position to 0.7
            brake_cmd = PacmodCmd()
            brake_cmd.enable = True
            brake_cmd.f64_cmd = 0.7
            # Set Steering Column to Straight
            steer_cmd = PositionWithSpeed()
            steer_cmd.angular_position = 0.0
            steer_cmd.angular_velocity_limit = 1
            self.steer_pub.publish(steer_cmd)
            num_calls += 1
            rate.sleep()

        self.file.close()

    def update(self):
        """Run in a loop"""
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            # Set steering column to center before moving
            while not self.steering_position == 0:
                steer_cmd = PositionWithSpeed()
                steer_cmd.angular_position = 0.0
                steer_cmd.angular_velocity_limit = 1
                self.steer_pub.publish(steer_cmd)
                rate.sleep()

            # Once steering column is centered - move
            accel_cmd = PacmodCmd()
            accel_cmd.enable = True
            accel_cmd.f64_cmd = self.throttle_level 
            self.accel_pub.publish(accel_cmd)  # Set Throttle
            brake_cmd = PacmodCmd()
            brake_cmd.enable = True
            brake_cmd.f64_cmd = 0
            self.brake_pub.publish(brake_cmd)  # Set Brake
            steer_cmd = PositionWithSpeed()
            steer_cmd.angular_position = 0.0
            steer_cmd.angular_velocity_limit = 1
            self.steer_pub.publish(steer_cmd)  # Set Steering
            rate.sleep()

    def healthy(self):
        """Returns True if the element is in a stable state."""
        return True

    def done(self):
        """Return True if you want to exit."""
        return False


def run_ros_loop(node):
    """Executes the event loop of a node using ROS.  `node` should support
    rate(), initialize(), cleanup(), update(), and done().  You should not modify
    this code.
    """
    # intializes the node. We use disable_signals so that the end() function can be called when Ctrl+C is issued.
    # See http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
    rospy.init_node(node.__class__.__name__, disable_signals=True)

    node.initialize()
    rate = rospy.Rate(node.rate())
    termination_reason = "undetermined"
    try:
        while not rospy.is_shutdown() and not node.done() and node.healthy():
            node.update()
            rate.sleep()
        if node.done():
            termination_reason = "Node done"
        if not node.healthy():
            termination_reason = "Node in unhealthy state"
    except KeyboardInterrupt:
        termination_reason = "Killed by user"
    except Exception as e:
        termination_reason = "Exception " + str(e)
        raise
    finally:
        node.cleanup()
        rospy.signal_shutdown(termination_reason)


if __name__ == "__main__":
    throttle_level = float(sys.argv[1])
    run_ros_loop(ThrottleInput(throttle_level))
