import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

from random import uniform

class DrawTurtle(Node):

    # Montar construtor da classe
    def __init__(self):
        # Inicializar o Node
        super().__init__('draw_turtle')

        # Criar um publisher
        self.publisher_ = self.create_publisher(msg_type=Twist, topic='/turtle1/cmd_vel', qos_profile=10)

        # Criar um timer para chamar a função de callback
        self.timer_ = self.create_timer(1, self.timer_callback) # Tempo em segundos
        self.i = 0

    def timer_callback(self):
        # Criar uma mensagem do tipo Twist
        msg = Twist()

        # Montar minha mensagem
        msg.angular.x = uniform(-5, 5)
        msg.angular.y = uniform(-5, 5)
        msg.angular.z = uniform(-5, 5)

        msg.linear.x = uniform(-5, 5)
        msg.linear.y = uniform(-5, 5)
        msg.linear.z = uniform(-5, 5)

        # Publicar a mensagem que criamos no publisher
        self.publisher_.publish(msg)

        # Gerar um log da publicação da mensagem
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1

def main(args=None):
    # Inicializar o RCLPY
    rclpy.init(args=args)

    # Inicializar a classe MoveTurtle
    draw = DrawTurtle()

    # O spin do TurtleControler serve para manter o programa rodando até que seja finalizado
    rclpy.spin(draw)

    # Limpar o nó
    draw.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()