import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class ThymioController(Node):
    def __init__(self):
        super().__init__('thymio_controller')

        self.center_wall_detected = -1.0   # -1.0 = rien détecté
        self.right_wall_detected = -1.0

        self.declare_parameter('linear_speed', 0.14)      # Vitesse linéaire maximale (m/s)
        self.declare_parameter('angular_speed', 0.0)

        self.center_wall_detected = -1.0   # -1.0 = rien détecté
        self.right_wall_detected = -1.0
        self.turning_left = False  

        # Initialisation du publisher pour les commandes de vitesse et des subscribers pour les capteurs de proximité
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Range, '/proximity/center', self.center_sensor_callback, 10)
        self.create_subscription(Range, '/proximity/right', self.right_sensor_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def center_sensor_callback(self, msg: Range):
        self.center_wall_detected = msg.range
        

    def right_sensor_callback(self, msg: Range):
        self.right_wall_detected = msg.range

    def control_loop(self):
        right_dist = self.right_wall_detected
        center_dist = self.center_wall_detected
        max_speed = self.get_parameter('linear_speed').value   # Lecture seule
        angular = self.get_parameter('angular_speed').value

        # ---------- Facteur frontal (0 à 1) ----------
        front_slow = 0.10    # Commence à ralentir à partir d'ici
        front_stop = 0.06    # S'arrête ici
        
        if center_dist < 0.0:                       # -1.0 = rien détecté
            front_factor = 1.0
        else:
            front_factor = (center_dist - front_stop) / (front_slow - front_stop)
            front_factor = max(0.0, min(1.0, front_factor))

        # ---------- Facteur latéral (side_min à 1) ----------
        side_slow = 0.05     # Égal à la distance cible par rapport au mur
        side_near = 0.03     # En deçà, utilise la vitesse minimale
        side_min = 0.3       # Ne descend jamais en dessous de 30% à cause du côté
        
        if right_dist <= 0.0:         # Rien détecté sur le côté, la vitesse vers l'avant doit être nulle
            side_factor = 0.0
        else:
            side_factor = (right_dist - side_near) / (side_slow - side_near)
            side_factor = max(side_min, min(1.0, side_factor))

        # ---------- Vitesse linéaire : applique la contrainte la plus restrictive ----------
        v = max_speed * min(front_factor, side_factor)

        # ---------- Vitesse angulaire (inchangée) ----------
        if right_dist > 0.0 and right_dist < 0.10:
            angular = (right_dist - 0.05) * -10
            self.set_parameters([Parameter('angular_speed', value=angular)])

        else:
            angular = -0.5

        # ---------- Virage à gauche : mur en the face + mur à droite ----------
        front_release = 0.12     # Devant dégagé au-dessus de cette valeur (ou pas de détection)
        left_turn_speed = 1.0    # rad/s, positif = gauche

        front_blocked = 0.0 <= center_dist < front_slow
        right_wall = 0.0 < right_dist < 0.10
        front_clear = center_dist < 0.0 or center_dist > front_release

        if not self.turning_left and front_blocked and right_wall:
            self.turning_left = True          # Entre dans le virage
        elif self.turning_left and front_clear:
            self.turning_left = False         # Devant dégagé : reprend le suivi de mur

        if self.turning_left:
            angular = left_turn_speed
            v = 0.0                           # Tourne sur place ; utiliser v * 0.3 pour tourner en avançant

        # Création et publication du message Twist
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)   


def main():
    rclpy.init()
    rclpy.spin(ThymioController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()