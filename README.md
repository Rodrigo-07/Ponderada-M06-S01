# Ponderada-M06-S01

## Introdução

Atividade da Engenharia da Computação da disciplina de Ponderada de Programação 06, sobre a Semana 01. Foi criado um projeto em ROS que interaje com a tartaruga do turtlesim de modo que, ao final da execução do script, a tela do turtlesim apresente um desenho. Para realizar o desenho, eu criar uma função que consegue analisar uma imagem e transformar em um vetores de pontos para a tartaruga percorrer e desenhar.

## Requisitos

- ROS2 Humble Hawksbill
- Python 3.10
- OpenCV
- Numpy
- Turtlesim
  
Além disso, o projeto foi desenvolvido e testado em um ambiente Linux, Ubuntu 22.04 LTS, o que não garante o funcionamento em outros sistemas operacionais.

## Instalação

Para instalar o ROS2 Humble Hawksbill, siga as instruções do site oficial do ROS2: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

Para instalar o Python 3.10, siga as instruções do site oficial do Python: [https://www.python.org/downloads/](https://www.python.org/downloads/)

Para instalar o OpenCV e o Numpy, execute o seguinte comando:

```bash
pip install opencv-python numpy
```

Agora que você já tem o ROS2, o Python 3.10, o OpenCV e o Numpy instalados, clone o repositório do projeto:

```bash
git clone https://github.com/Rodrigo-07/Ponderada-M06-S01
```

## Execução


Para executar o projeto, abra um terminal e execute o seguinte comando, para iniciar o nó do turtlesim e visualizar a tartaruga:

```bash
ros2 run turtlesim turtlesim_node
```

Em seguida, abra outro terminal e execute o seguinte comando, para fazer o build do package e do setup do pacote:

```bash
cd workspace_ponderada/src/TurtleDraw

colcon build
. install/setup.bash
```

Por fim, execute o seguinte comando para rodar o script que desenha a imagem na tela do turtlesim com o parametro image_path sendo o caminho, **absuluto**, da imagem que deseja desenhar:

```bash

ros2 run TurtleDraw draw_image --ros-args -p image_path:="caminho_da_imagem"
```
Exemplo:

```bash
ros2 run TurtleDraw draw_image --ros-args -p image_path:="/home/rodrigo-07/Github/Ponderada-M06-S01/images_test/twitter.jpeg"
ros2 run TurtleDraw draw_image --ros-args -p image_path:="/home/rodrigo-07/Github/Ponderada-M06-S01/images_test/star.jpeg"
```

## Resultado

[Link de demonstração do projeto](https://youtube.com/)

## Sistema de mapeamento da imagem

Para a tartaruga conseguir desenhar a imagem, foi necessário criar um sistema de pré-processeamento na imagem para que seja pego os contornos da imagem e transformado em um vetor de pontos. Para realizar isso foi criado a função `process_image` que recebe a imagem e retorna um vetor de pontos. Nela é realizado o seguinte processo:

1. Transformar a imagem em escala de cinza e redimensionar a imagem para o tamanho da tela do turtlesim (500 x 500 pixels).
2. Aplicar uma suavização gaussiana na imagem para reduzir o ruído.
3. Aplicar o método de Canny para detectar as bordas da imagem.
4. Aplicar fechamento morfológico para fechar as bordas da imagem e preencher os buracos.
5. Achar os contornos da imagem.
6. Após achar os contornos da imagem, é feito um loop para pegar os pontos dos contornos e transformar em um vetor de pontos.

Foi utilziado a biblioteca OpenCV para realizar o pré-processamento da imagem.

Existe algumas considerações e limitações para o sistema de mapeamento da imagem, como imagem com muitos detalhes e complexas, com muitos contornos ou com muitos ruídos podem não ser desenhadas corretamente. Para melhorar o sistema de mapeamento da imagem, é necessário realizar um tratamento de imagem mais avançado e complexo, como por exemplo, a utilização de redes neurais para identificar os contornos da imagem.

## Arquivos do pacote

Dentro do pacote TurtleDraw, temos os seguintimos arquivos executáveis com o ros2:

- `draw_image`: Script que desenha a imagem na tela do turtlesim, com o próprio traço da tartaruga, a partir de uma imagem que é passada como parametro.
- `draw_image_turtles.py`: Script que desenha a imagem na tela do turtlesim, com o spaw de várias tartarugas, a partir de uma imagem que é passada como parametro. 
- `draw_random.py`: Script que desenha traços aleatórios na tela do turtlesim.

Também tem o arquivo `image_processing.py` que foi utilizado como teste para do sistema de mapeamento da imagem.
