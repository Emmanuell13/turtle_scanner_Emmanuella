import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random

class SpawnTarget(Node):

   def __init__(self):
       super().__init__('spawn_target')

       # Création du client pour appeler le service /spawn
       self.client = self.create_client(Spawn, '/spawn')

       # Attendre que le service soit disponible
       while not self.client.wait_for_service(timeout_sec=1.0):
           self.get_logger().info('Service /spawn non disponible, attente...')

       # Générer coordonnées aléatoires
       self.x = random.uniform(1.0, 10.0)
       self.y = random.uniform(1.5, 10.2)

       # Créer la requête
       self.req = Spawn.Request()
       self.req.x = self.x
       self.req.y = self.y
       self.req.theta = 0.0
       self.req.name = 'turtle_target'

       # Appeler le service
       self.future = self.client.call_async(self.req)
       self.future.add_done_callback(self.response_callback)

   def response_callback(self, future):
       try:
           response = future.result()
           self.get_logger().info(
               f'Tortue cible spawnée à ({self.x:.2f}, {self.y:.2f})'
           )
       except Exception as e:
           self.get_logger().error(f'Erreur lors du spawn : {e}')


def main(args=None):
   rclpy.init(args=args)
   node = SpawnTarget()
   rclpy.spin(node)
   rclpy.shutdown()

if __name__ == '__main__':
   main()
