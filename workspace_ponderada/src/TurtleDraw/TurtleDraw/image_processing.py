import cv2
import numpy as np
import matplotlib.pyplot as plt

def process_image(image_path):
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

    # Aplicar fechamento morfológico
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    # Encontrar contornos
    contours, _ = cv2.findContours(closed_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Preparar a imagem para desenhar os contornos
    result = img.copy()

    scaled_contours = []

    # Filtrar e desenhar contornos baseados em seu perímetro
    for contour in contours:
        if cv2.contourArea(contour) > 100:  # Filtrar contornos
            perimeter = cv2.arcLength(contour, True)

            contour = np.array(contour)
            xs = contour[:, 0, 0]
            ys = contour[:, 0, 1]

            # Normalizar os contornos para o range certo
            xs = 11 * (xs - min(xs)) / (max(xs) - min(xs))
            ys = 11 * (ys - min(ys)) / (max(ys) - min(ys))

            scaled_contours.append(np.column_stack((xs, ys)))

            if perimeter > 500:
                cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)

    # Salvar e mostrar os resultados
    cv2.imwrite("processed_image.jpg", result)
    cv2.imshow("Original", img)
    cv2.imshow("Edges", edges)
    cv2.imshow("Result with Contours", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return scaled_contours

# Ajuste o caminho para sua imagem
contours = process_image('/home/rodrigo-07/Github/Ponderada-M06-S01/images_test/charmander.png')

print(contours)

def plot_contours(contours):
    # Criar a figura e o eixo
    fig, ax = plt.subplots()

    for contour in contours:
        # Transformar o contorno em um array numpy
        contour = np.array(contour)
        xs = contour[:, 0]
        ys = contour[:, 1]

        # Plotar os contornos
        ax.plot(xs, ys, marker='o') 

    # Definir limites para os eixos
    ax.set_xlim([0, 11])
    ax.set_ylim([0, 11])
    ax.set_aspect('equal', 'box')

    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')

    ax.grid(True)

    plt.show()

plot_contours(contours)

