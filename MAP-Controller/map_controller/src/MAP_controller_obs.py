#!/usr/bin/env python3

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from f110_msgs.msg import WpntArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from steering_lookup.lookup_steer_angle import LookupSteerAngle
import tf


class Controller:
    def __init__(self):
        rospy.init_node('control_node', anonymous=True)

        # Get parameters for the MAP controller
        self.param_q_map = rospy.get_param('/control_node/q_map')
        self.param_m_map = rospy.get_param('/control_node/m_map')
        self.param_t_clip_min = rospy.get_param('/control_node/t_clip_min')
        self.param_t_clip_max = rospy.get_param('/control_node/t_clip_max')

        # Load lookup table to calculate steering angle
        LUT_name = rospy.get_param('/control_node/LU_table')
        self.steer_lookup = LookupSteerAngle(LUT_name)

        # Set loop rate in hertz
        self.loop_rate = 40

        # Initialize variables
        self.position = None  # current position in map frame [x, y, theta]
        self.waypoints = None  # waypoints in map frame [x, y, speed]
        self.avoidance_waypoints = None  # avoidance waypoints
        self.ros_time = rospy.Time()

        # Publisher to publish lookahead point and steering angle
        self.lookahead_pub = rospy.Publisher('lookahead_point', Marker, queue_size=10)

        # Publisher for steering and speed command
        self.drive_pub = rospy.Publisher('/drive',
                                         AckermannDriveStamped,
                                         queue_size=10)

        # Subscribers to get waypoints and the position of the car
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/global_path/optimal_trajectory_wpnt', WpntArray, self.waypoints_cb)
        rospy.Subscriber('/planner/avoidance/otwpnts', WpntArray, self.avoidance_waypoints_cb)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_speed = msg.twist.twist.linear.x
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        _, _, self.yaw = tf.transformations.euler_from_quaternion(orientation_list)
        self.position = np.array([self.current_pose.position.x, self.current_pose.position.y, self.yaw])

    def waypoints_cb(self, data):
        """
        The callback function for the '/global_waypoints' subscriber.

        Args:
            data (WpntArray): The message containing the global waypoints.
        """
        self.waypoints = np.empty((len(data.wpnts), 3), dtype=np.float32)
        for i, waypoint in enumerate(data.wpnts):
            speed = waypoint.vx_mps
            self.waypoints[i] = [waypoint.x_m, waypoint.y_m, speed]

    def avoidance_waypoints_cb(self, data):
        """
        Callback for the '/planner/avoidance/otwpnts' topic.

        Args:
            data (WpntArray): The message containing avoidance waypoints.
        """
        self.avoidance_waypoints = np.empty((len(data.wpnts), 3), dtype=np.float32)
        for i, waypoint in enumerate(data.wpnts):
            speed = waypoint.vx_mps
            self.avoidance_waypoints[i] = [waypoint.x_m, waypoint.y_m, speed]

    def control_loop(self):
        """
        Control loop for the MAP controller.
        """
        rate = rospy.Rate(self.loop_rate)

        while not rospy.is_shutdown():
            # Wait for position and waypoints
            if self.position is None or (self.waypoints is None and self.avoidance_waypoints is None):
                continue

            # Determine which waypoints to use
            if self.avoidance_waypoints is not None and len(self.avoidance_waypoints) > 0:
                # Find the nearest avoidance waypoint and calculate its distance
                idx_nearest_avoidance = self.nearest_waypoint(self.position[:2], self.avoidance_waypoints[:, :2])
                avoidance_distance = self.distance(self.position[:2], self.avoidance_waypoints[idx_nearest_avoidance, :2])

                # Check if near the end of avoidance waypoints
                if idx_nearest_avoidance >= len(self.avoidance_waypoints) - 2:
                    rospy.loginfo_throttle(1, "[Controller]: Near end of avoidance waypoints, switching to global waypoints.")
                    # Switch to global waypoints
                    waypoints = self.waypoints
                else:
                    waypoints = self.avoidance_waypoints
                    rospy.loginfo_throttle(1, "[Controller]: Following avoidance waypoints.")
            else:
                waypoints = self.waypoints
                rospy.loginfo_throttle(1, "[Controller]: Following global waypoints.")


            # Send speed and steering commands to mux
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = self.ros_time.now()
            ack_msg.header.frame_id = 'base_link'

            if waypoints is not None and waypoints.shape[0] > 2:
                idx_nearest_waypoint = self.nearest_waypoint(self.position[:2], waypoints[:, :2])

                # Desired speed at waypoint closest to car
                target_speed = waypoints[idx_nearest_waypoint, 2]

                # Calculate lookahead_distance
                lookahead_distance = self.param_q_map + target_speed * self.param_m_map
                lookahead_distance = np.clip(lookahead_distance, self.param_t_clip_min, self.param_t_clip_max)
                lookahead_point = self.waypoint_at_distance_infront_car(
                    lookahead_distance, waypoints[:, :2], idx_nearest_waypoint)

                if lookahead_point.any() is not None:
                    # Vector from the current position to the point at lookahead distance
                    position_la_vector = np.array([lookahead_point[0] - self.position[0], lookahead_point[1] - self.position[1]])
                    yaw = self.position[2]
                    eta = np.arcsin(np.dot([-np.sin(yaw), np.cos(yaw)], position_la_vector) / np.linalg.norm(position_la_vector))
                    lat_acc = 2 * target_speed**2 / lookahead_distance * np.sin(eta)

                    steering_angle = self.steer_lookup.lookup_steer_angle(lat_acc, target_speed)
                    ack_msg.drive.steering_angle = steering_angle
                    ack_msg.drive.speed = np.max(target_speed, 0)  # no negative speed

                    self.visualize_lookahead(lookahead_point)
                    self.visualize_steering(steering_angle)

            # If there are no waypoints, publish zero speed and steer to STOP
            else:
                ack_msg.drive.speed = 0
                ack_msg.drive.steering_angle = 0
                rospy.logerr('[MAP Controller]: No waypoints received. Stopping!')

            # Always publish ackermann msg
            self.drive_pub.publish(ack_msg)
            rate.sleep()

    @staticmethod
    def distance(point1, point2):
        """
        Calculates the Euclidean distance between two points.

        Args:
            point1 (tuple): A tuple containing the x and y coordinates of the first point.
            point2 (tuple): A tuple containing the x and y coordinates of the second point.

        Returns:
            float: The Euclidean distance between the two points.
        """
        return (((point2[0] - point1[0]) ** 2) + ((point2[1] - point1[1]) ** 2))**0.5

    @staticmethod
    def nearest_waypoint(position, waypoints):
        """
        Finds the index of the nearest waypoint to a given position.

        Args:
            position (tuple): A tuple containing the x and y coordinates of the position.
            waypoints (numpy.ndarray): An array of tuples representing the x and y coordinates of waypoints.

        Returns:
            int: The index of the nearest waypoint to the position.
        """
        position_array = np.array([position]*len(waypoints))
        distances_to_position = np.linalg.norm(abs(position_array - waypoints), axis=1)
        return np.argmin(distances_to_position)

    def waypoint_at_distance_infront_car(self, distance, waypoints, idx_waypoint_behind_car):
        """
        Finds the waypoint a given distance in front of a given waypoint.

        Args:
            distance (float): The distance to travel from the given waypoint.
            waypoints (numpy.ndarray): An array of tuples representing the x and y coordinates of waypoints.
            idx_waypoint_behind_car (int): The index of the waypoint behind the car.

        Returns:
            numpy.ndarray: A tuple containing the x and y coordinates of the waypoint a given distance in front of the given waypoint.
        """
        dist = 0
        i = idx_waypoint_behind_car

        while dist < distance:
            i = (i + 1) % len(waypoints)
            dist = self.distance(waypoints[idx_waypoint_behind_car], waypoints[i])

        return np.array(waypoints[i])

    def visualize_steering(self, theta):
        """
        Publishes a visualization of the steering direction as an arrow.

        Args:
            theta (float): The steering angle in radians.
        """
        quaternions = quaternion_from_euler(0, 0, theta)

        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "base_link"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = Marker.ARROW
        lookahead_marker.id = 2
        lookahead_marker.scale.x = 0.6
        lookahead_marker.scale.y = 0.05
        lookahead_marker.scale.z = 0
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.lifetime = rospy.Duration()
        lookahead_marker.pose.position.x = 0
        lookahead_marker.pose.position.y = 0
        lookahead_marker.pose.position.z = 0
        lookahead_marker.pose.orientation.x = quaternions[0]
        lookahead_marker.pose.orientation.y = quaternions[1]
        lookahead_marker.pose.orientation.z = quaternions[2]
        lookahead_marker.pose.orientation.w = quaternions[3]
        self.lookahead_pub.publish(lookahead_marker)

    def visualize_lookahead(self, lookahead_point):
        """
        Publishes a marker indicating the lookahead point on the map.

        Args:
            lookahead_point (tuple): A tuple of two floats representing the x and y coordinates of the lookahead point.
        """
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = 2
        lookahead_marker.id = 1
        lookahead_marker.scale.x = 0.15
        lookahead_marker.scale.y = 0.15
        lookahead_marker.scale.z = 0.15
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = lookahead_point[0]
        lookahead_marker.pose.position.y = lookahead_point[1]
        lookahead_marker.pose.position.z = 0
        lookahead_marker.pose.orientation.x = 0
        lookahead_marker.pose.orientation.y = 0
        lookahead_marker.pose.orientation.z = 0
        lookahead_marker.pose.orientation.w = 1
        self.lookahead_pub.publish(lookahead_marker)


if __name__ == "__main__":
    controller = Controller()
    controller.control_loop()

            
