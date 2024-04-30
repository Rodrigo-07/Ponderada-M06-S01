import rclpy

from rclpy.node import Node

from geometry_msgs.msg._pose import Pose

class MoveTurtle(Node):

    # Montar construtor da classe
    def __init__(self):
        super().__init__('move_turtle')
        
        # Criar um publisher
        self.publisher_ = self.create_publisher(msg_type=Pose, topic='/turtle1/pose', qos_profile=10)

        # Criar um timer
        self.timer_ = self.create_timer(0.5, callback=self.timer_callback)
        self.i = 0

    # Montar função de callback da nossa classe
    def timer_callback(self):

        # Criar uma mensagem do tipo Pose
        msg = Pose()

        msg.x = 3.542
        msg.y = 3.542
        msg.theta = 0.0

        # Publicar a mensagem que criamos no publisher
        self.publisher_.publish(msg)

        # Gerar um log da publicação da mensagem
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1

def main(args=None):
    # Inicializar o RCLPY
    rclpy.init(args=args)

    # Inicializar a classe MoveTurtle
    move = MoveTurtle()

    # O spin do TurtleControler serve para manter o programa rodando até que seja finalizado
    rclpy.spin(move)

    # LImpar
    move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





