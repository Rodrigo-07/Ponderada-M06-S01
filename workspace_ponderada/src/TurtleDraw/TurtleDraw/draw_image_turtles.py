import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

import random
import string

from teste import process_image

class DrawTurtleImage(Node):

    # Montar construtor da classe
    def __init__(self):
        # Inicializar o Node
        super().__init__('draw_turtle_image')

        # Criar um publisher
        self.publisher_ = self.create_publisher(msg_type=Twist, topic='/turtle1/cmd_vel', qos_profile=10)

        # CRiar um cliente para o serviço de spawn
        self.spawn_client = self.create_client(Spawn, '/spawn')

        self.spawn_turtle()

        self.image_contours = self.process_image('star.jpeg')

        self.turtle_number = 0

        print(self.image_contours)

    def process_image(self, image_path):
        
        image_points = process_image(image_path)

        return image_points
    
    def spawn_turtle(self):

        spawn_info = Spawn.Request()
        spawn_info.x = 5.0
        spawn_info.y = 5.0
        spawn_info.theta = 0.0

        spawn_info.name = f'turtle_{self.turtle_number}'
        self.turtle_number += 1

        future = self.spawn_client.call_async(spawn_info)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Erro ao espalnar: %r' % (e,))


def main(args=None):
    # Inicializar o RCLPY
    rclpy.init(args=args)

    # Inicializar a classe MoveTurtle
    draw = DrawTurtleImage()

    # O spin do TurtleControler serve para manter o programa rodando até que seja finalizado
    rclpy.spin_once(draw)

    # Limpar o nó
    draw.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()