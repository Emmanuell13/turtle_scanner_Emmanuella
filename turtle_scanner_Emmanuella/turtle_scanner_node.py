#!/usr/bin/env python3

 

import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

 

 

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

 

        # Liste des waypoints
        self.waypoints = self.generate_waypoints()
        self.current_waypoint_index = 0
        self.scan_finished = False

 

        self.get_logger().info(f"Waypoints générés : {self.waypoints}")

 

        # Timer pour appeler scan_step régulièrement
        self.timer = self.create_timer(0.05, self.scan_step)

 

        self.get_logger().info("Node scanner démarré")

 

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

 

    def scan_step(self):
        # Attendre la pose de turtle1
        if self.pose_scanner is None:
            return

 

        # Si c'est fini, ne rien faire
        if self.scan_finished:
            return

 

        # Tous les waypoints sont terminés
        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_turtle()
            self.scan_finished = True
            self.get_logger().info("Balayage terminé")
            return

 

        # Waypoint courant
        target_x, target_y = self.waypoints[self.current_waypoint_index]

 

        # Position actuelle du scanner
        current_x = self.pose_scanner.x
        current_y = self.pose_scanner.y
        current_theta = self.pose_scanner.theta

 

        # Distance vers le waypoint
        distance = self.compute_distance((current_x, current_y), (target_x, target_y))

 

        # Si on est assez proche, passer au suivant
        if distance < self.waypoint_tolerance:
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_index + 1} atteint : ({target_x:.2f}, {target_y:.2f})"
            )
            self.current_waypoint_index += 1
            return

 

        # Calcul angle désiré
        theta_desired = self.compute_angle((current_x, current_y), (target_x, target_y))

 

        # Erreur angulaire
        angle_error = theta_desired - current_theta
        angle_error = self.normalize_angle_error(angle_error)

 

        # Contrôle proportionnel
        angular_z = self.Kp_ang * angle_error
        linear_x = self.Kp_lin * distance

 

        # Limiter la vitesse linéaire
        if linear_x > self.linear_speed_max:
            linear_x = self.linear_speed_max

 

        # Si la tortue doit beaucoup tourner, on ralentit l'avance
        if abs(angle_error) > 0.5:
            linear_x = 0.2

 

        # Publier la commande
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
