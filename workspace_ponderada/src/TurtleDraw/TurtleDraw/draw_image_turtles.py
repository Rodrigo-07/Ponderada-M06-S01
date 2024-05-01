import rclpy
import cv2
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

import random
import string

class DrawTurtleImage(Node):

    # Montar construtor da classe
    def __init__(self):
        # Inicializar o Node
        super().__init__('draw_turtle_image')

        # Criar um publisher
        self.publisher_ = self.create_publisher(msg_type=Twist, topic='/turtle1/cmd_vel', qos_profile=10)

        # CRiar um cliente para o serviço de spawn
        self.spawn_client = self.create_client(Spawn, '/spawn')

        self.turtle_number = 0

        self.draw_image()

    def process_image(self, image_path):
        # Carregar imagem em escala de cinza
        print(image_path)
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

        if image is None:
            raise Exception(f"Não foi possível carregar a imagem em {image_path}. Verifique o caminho e as permissões do arquivo.")

        image = cv2.resize(image, (500, 500))
        
        # Aplicar limiarização para torná-la binária
        _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        scaled_contours = []

        # Loop through the list of contours
        for contour in contours:
            # Convert contour array to numpy array for easy slicing
            # and separate x and y coordinates
            contour = np.array(contour)
            xs = contour[:, 0, 0]
            ys = contour[:, 0, 1]

            # Normalize the coordinates to a 0-10 scale
            xs = 10 * (xs - min(xs)) / (max(xs) - min(xs))
            ys = 10 * (ys - min(ys)) / (max(ys) - min(ys))

            # Append the scaled coordinates to the list
            scaled_contours.append(np.column_stack((xs, ys)))

        print(scaled_contours)

        return scaled_contours
    
    def spawn_turtle(self, x, y, theta, name):

        spawn_info = Spawn.Request()
        spawn_info.x = x
        spawn_info.y = y
        spawn_info.theta = theta

        spawn_info.name = name

        future = self.spawn_client.call_async(spawn_info)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Erro ao espalnar: %r' % (e,))

    def draw_image(self):

        self.image_contours = self.process_image('/home/rodrigo-07/Github/Ponderada-M06-S01/workspace_ponderada/src/TurtleDraw/TurtleDraw/star.jpeg')

        for contour in self.image_contours:
            for point in contour:
                x, y = point
                print(f'x: {x}, y: {y}')
                try:
                    self.spawn_turtle(x, y, 0.0, f'turtle_{self.turtle_number}')

                    print(f'turtle_{self.turtle_number} spawned at x: {x}, y: {y}')
                except Exception as e:
                    self.get_logger().error('Erro ao espalnar: %r' % (e,))

                self.turtle_number += 1
            


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