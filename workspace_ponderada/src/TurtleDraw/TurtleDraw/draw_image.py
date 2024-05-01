import rclpy
import cv2
import numpy as np
import matplotlib.pyplot as plt

from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen
from turtlesim.msg import Pose

import random
import string

class DrawTurtleImage(Node):

    # Montar construtor da classe
    def __init__(self):
        # Inicializar o Node
        super().__init__('draw_turtle_image')

        # Carregar os contornos da imagem
        self.image_contours = self.process_image('/home/rodrigo-07/Github/Ponderada-M06-S01/images_test/star.jpeg')

        # Criar um cliente para o serviço de spawn
        self.spawn_client = self.create_client(Spawn, '/spawn')

        # Spawnar uma tartaruga no primeiro ponto dos contornos da imagem
        self.spawn_turtle(self.image_contours[0][0][0], self.image_contours[0][0][1], 0.0, 'turtleDesigner')

        # Criar um publisher
        self.position_publisher= self.create_publisher(msg_type=Twist, topic='/turtleDesigner/cmd_vel', qos_profile=10)

        # Criar um cliente para o serviço de kill
        self.kill_client = self.create_client(Kill, '/kill')

        # Matar a tartaruga padrão
        self.kill_turtle('turtle1')

        # Subscrever no tópico de pegar a posição da tartaruga
        self.pose_subscription = self.create_subscription(Pose, '/turtleDesigner/pose', self.pose_callback, 10)
        self.pose_subscription
        
        # Inicializar a variavel de posição da tartaruga
        self.turtle_position = Pose()

        self.position_received = False
        
    # Callback para pegar a posição da tartaruga quando o Pose() for publicado
    def pose_callback(self, msg):
        self.turtle_position = msg
        self.position_received = True
        # print(f'Posição da tartaruga: x: {msg.x}, y: {msg.y}, theta: {msg.theta}')
        # self.get_logger().info(f'Posição da tartaruga: x: {msg.x}, y: {msg.y}, theta: {msg.theta}')

    def draw_image(self):

        # Loop para cada contorno
        for contour in self.image_contours:
            for point in contour:
                x, y = point
                self.get_logger().info(f'Movendo a tartaruga para x: {x}, y: {y}')
                try:
                    self.go_to_position(x, y)
                except Exception as e:
                    self.get_logger().error(f'Erro ao mover a tartaruga: {e}')


    def go_to_position(self, x, y):
        # Preciso pegar a posição atual da tarturuga
        # Calcular o angulo entre a posição atual e a posição desejada
        # Girar ela na direção correta
        # Mover ela para a posição desejada

        # Esperar a posição inicial da tartaruga
        while not self.position_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info('Waiting for position...')
        
        # Subscrever no tópico de pegar a posição da tartaruga
        self.pose_subscription

        # Calcular o angulo entre a posição atual e a posição desejada
        angle = np.arctan2(y - self.turtle_position.y, x - self.turtle_position.x)
        # print(f'Posição atual: x: {self.turtle_position.x}, y: {self.turtle_position.y}')

        # Girar ela na direção correta
        while abs(angle - self.turtle_position.theta) > 0.1:
            msg = Twist()
            # print(angle - self.turtle_position.theta)
            msg.angular.z = 0.5 if angle > self.turtle_position.theta else -0.5
            self.position_publisher.publish(msg)
            rclpy.spin_once(self)
        
        # Mover ela para a posição desejada
        msg = Twist()
        msg.angular.z = 0.0 # Parar de girar
        while abs(x - self.turtle_position.x) > 0.1 or abs(y - self.turtle_position.y) > 0.1:
            msg.linear.x = 0.5
            self.position_publisher.publish(msg)
            rclpy.spin_once(self)

        # Parar a tartaruga
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.position_publisher.publish(msg)


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
            self.get_logger().error('Erro a dar spaw na tartaruga {name}: %r' % (e,))
    
    def kill_turtle(self, name):
        kill_info = Kill.Request()
        kill_info.name = name

        future = self.kill_client.call_async(kill_info)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Erro ao matar a tartaruga {name}: %r' % (e,))
            

def main(args=None):
    # Inicializar o RCLPY
    rclpy.init(args=args)

    # Inicializar a classe MoveTurtle
    draw = DrawTurtleImage()

    draw.draw_image()

    # O spin do TurtleControler serve para manter o programa rodando até que seja finalizado
    rclpy.spin(draw)

    # Limpar o nó
    draw.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()