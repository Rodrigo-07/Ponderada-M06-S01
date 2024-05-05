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

        self.declare_parameter('image_path', '-')

        # Obter o valor do parâmetro
        image_path_param = self.get_parameter('image_path').get_parameter_value().string_value
        
        print(image_path_param)

        # Carregar os contornos da imagem
        self.image_contours = self.process_image(image_path_param)

        # Criar um cliente para o serviço de spawn
        self.spawn_client = self.create_client(Spawn, '/spawn')

        # Spawnar uma tartaruga no primeiro ponto dos contornos da imagem
        self.spawn_turtle(self.image_contours[0][0][0], self.image_contours[0][0][1], 0.0, 'turtleDesigner')

        # Criar um publisher
        self.position_publisher= self.create_publisher(msg_type=Twist, topic='/turtleDesigner/cmd_vel', qos_profile=10)

        # Criar um cliente para o serviço de kill
        self.kill_client = self.create_client(Kill, '/kill')

        # Criar um cliente para o serviço de set_pen
        self.set_pen_client = self.create_client(SetPen, '/turtleDesigner/set_pen')

        # Setar a cor da caneta da tartaruga
        self.set_pen_turtle(0, 0, 0, 2, 0)

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
            msg.angular.z = 1.0 if angle > self.turtle_position.theta else -1.0
            self.position_publisher.publish(msg)
            rclpy.spin_once(self)
        
        # Mover ela para a posição desejada
        msg = Twist()
        msg.angular.z = 0.0 # Parar de girar
        while abs(x - self.turtle_position.x) > 0.1 or abs(y - self.turtle_position.y) > 0.1:
            msg.linear.x = 1.0
            self.position_publisher.publish(msg)
            rclpy.spin_once(self)

        # Parar a tartaruga
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.position_publisher.publish(msg)


    def process_image(self, image_path):
        # Carregar imagem
        img = cv2.imread(image_path)
        if img is None:
            raise Exception(f"Não foi possível carregar a imagem em {image_path}. Verifique o caminho e as permissões do arquivo.")
        
        img = cv2.resize(img, (500, 500))
        
        # Converter para escala de cinza
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Aplicar suavização Gaussiana
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detecção de bordas Canny
        edges = cv2.Canny(blurred, 50, 150)

        # Aplicar fechamento morfológico que vai servir para fechar os contornos e buracos
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Encontrar contornos
        contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        scaled_contours = []

        # Filtrar e desenhar contornos baseados em seu perímetro
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filtrar contornos pequenos
                perimeter = cv2.arcLength(contour, True)

                # contour = np.array(contour)
                # xs = contour[:, 0, 0]
                # ys = contour[:, 0, 1]

                # # Normalize the coordinates to a 0-10 scale
                # xs = 10 * (xs - min(xs)) / (max(xs) - min(xs))
                # ys = 10 * (ys - min(ys)) / (max(ys) - min(ys))
                
                # Transformar o contorno gerado pelo openCV em um array 2D
                contour = np.squeeze(contour)

                # Normalizar as coordenadas de 0 a 1 (dividindo por 500) e depois de 0 a 10 para se adequar ao espaço do turtlesim
                xs = contour[:, 0] * (10 / 500)
                ys = contour[:, 1] * (10 / 500)

                # Inverter o eixo Y 
                ys = 11 - ys

                # Colocar as coordenadas em um array 2D
                scaled_contours.append(np.column_stack((xs, ys)))


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

    def set_pen_turtle(self, r, g, b, width, off):
        set_pen_info = SetPen.Request()
        set_pen_info.r = r
        set_pen_info.g = g
        set_pen_info.b = b
        set_pen_info.width = width
        set_pen_info.off = off

        future = self.set_pen_client.call_async(set_pen_info)
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Erro ao setar as configurações da caneta da tartaruga: %r' % (e,))
            

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