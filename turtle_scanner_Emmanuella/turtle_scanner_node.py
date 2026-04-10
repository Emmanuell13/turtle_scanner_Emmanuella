#!/usr/bin/env python3
 
import math

import rclpy

from rclpy.node import Node

from turtlesim.msg import Pose

from geometry_msgs.msg import Twist

from std_msgs.msg import Bool
 
 
class TurtleScanner(Node):

    def __init__(self):

        super().__init__('turtle_scanner_node')
 
        # Positions des tortues

        self.pose_scanner = None

        self.pose_target = None
 
        # Subscribers

        self.sub_scanner = self.create_subscription(

            Pose,

            '/turtle1/pose',

            self.scanner_callback,

            10

        )
 
        self.sub_target = self.create_subscription(

            Pose,

            '/turtle_target/pose',

            self.target_callback,

            10

        )
 
        # Publisher pour faire bouger turtle1

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
 
        # Publisher pour la détection

        self.detect_pub = self.create_publisher(Bool, '/target_detected', 10)
 
        # Paramètres du serpentin

        self.nb_lignes = 5

        self.y_start = 1.0

        self.y_step = 2.0

        self.x_min = 1.0

        self.x_max = 10.0
 
        # Paramètres de contrôle

        self.waypoint_tolerance = 0.3

        self.Kp_ang = 5.0

        self.Kp_lin = 1.0

        self.linear_speed_max = 2.0
 
        # Paramètres de détection

        self.detection_radius = 1.5

        self.target_found = False
 
        # Waypoints

        self.waypoints = self.generate_waypoints()

        self.current_waypoint_index = 0

        self.scan_finished = False
 
        self.get_logger().info(f"Waypoints générés : {self.waypoints}")
 
        # Timer

        self.timer = self.create_timer(0.05, self.scan_step)
 
    def scanner_callback(self, msg):

        self.pose_scanner = msg
 
    def target_callback(self, msg):

        self.pose_target = msg
 
    def generate_waypoints(self):

        waypoints = []
 
        for i in range(self.nb_lignes):

            y = self.y_start + i * self.y_step
 
            if i % 2 == 0:

                x = self.x_max

            else:

                x = self.x_min
 
            waypoints.append((x, y))
 
        return waypoints
 
    def compute_angle(self, A, B):

        xA, yA = A

        xB, yB = B

        return math.atan2(yB - yA, xB - xA)
 
    def compute_distance(self, A, B):

        xA, yA = A

        xB, yB = B

        return math.sqrt((xB - xA) ** 2 + (yB - yA) ** 2)
 
    def normalize_angle_error(self, angle_error):

        return math.atan(math.tan(angle_error / 2.0))
 
    def stop_turtle(self):

        msg = Twist()

        msg.linear.x = 0.0

        msg.angular.z = 0.0

        self.cmd_pub.publish(msg)
 
    def publish_detection(self, detected):

        msg = Bool()

        msg.data = detected

        self.detect_pub.publish(msg)
 
    def scan_step(self):

        # Attendre les poses

        if self.pose_scanner is None or self.pose_target is None:

            return
 
        # Si la cible est déjà trouvée, ne plus bouger

        if self.target_found:

            self.stop_turtle()

            self.publish_detection(True)

            return
 
         

        

        distance_to_target = self.compute_distance(

            (self.pose_scanner.x, self.pose_scanner.y),

            (self.pose_target.x, self.pose_target.y)

        )
 
        if distance_to_target < self.detection_radius:

            self.stop_turtle()

            self.target_found = True

            self.publish_detection(True)
 
            self.get_logger().info(

                f"Cible détectée à ({self.pose_target.x:.2f}, {self.pose_target.y:.2f}) !"

            )

            return

        else:

            self.publish_detection(False)
 
        # -----------------------------

        # PARTIE 3 : serpentin

        # -----------------------------

        if self.scan_finished:

            return
 
        if self.current_waypoint_index >= len(self.waypoints):

            self.stop_turtle()

            self.scan_finished = True

            self.get_logger().info("Balayage terminé")

            return
 
        target_x, target_y = self.waypoints[self.current_waypoint_index]
 
        current_x = self.pose_scanner.x

        current_y = self.pose_scanner.y

        current_theta = self.pose_scanner.theta
 
        distance = self.compute_distance((current_x, current_y), (target_x, target_y))
 
        if distance < self.waypoint_tolerance:

            self.get_logger().info(

                f"Waypoint {self.current_waypoint_index + 1} atteint : ({target_x:.2f}, {target_y:.2f})"

            )

            self.current_waypoint_index += 1

            return
 
        theta_desired = self.compute_angle((current_x, current_y), (target_x, target_y))
 
        angle_error = theta_desired - current_theta

        angle_error = self.normalize_angle_error(angle_error)
 
        angular_z = self.Kp_ang * angle_error

        linear_x = self.Kp_lin * distance
 
        if linear_x > self.linear_speed_max:

            linear_x = self.linear_speed_max
 
        if abs(angle_error) > 0.5:

            linear_x = 0.2
 
        cmd = Twist()

        cmd.linear.x = linear_x

        cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)
 
 
def main(args=None):

    rclpy.init(args=args)

    node = TurtleScanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
 
 
if __name__ == '__main__':

    main()
 
