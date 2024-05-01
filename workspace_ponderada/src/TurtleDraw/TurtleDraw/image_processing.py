import cv2
import numpy as np
import matplotlib.pyplot as plt


def process_image(image_path):
    # Carregar imagem em escala de cinza
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    image = cv2.resize(image, (500, 500))
    
    if image is None:
        raise Exception(f"Não foi possível carregar a imagem em {image_path}. Verifique o caminho e as permissões do arquivo.")
    
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

    return scaled_contours


# Ajuste o caminho para sua imagem
contours = process_image('star.jpeg')

print(contours)

def plot_contours(contours):
    # Create a figure and a set of subplots
    fig, ax = plt.subplots()

    # Loop through the list of contours
    for contour in contours:
        # Convert contour array to numpy array for easy slicing
        # and separate x and y coordinates
        contour = np.array(contour)
        xs = contour[:, 0]
        ys = contour[:, 1]

        # Plot each contour
        ax.plot(xs, ys, marker='o')  # 'o' can be replaced with other marker types if desired

    # Set plot limits
    ax.set_xlim([0, 11])
    ax.set_ylim([0, 11])
    ax.set_aspect('equal', 'box')

    # Label the axes
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')

    # Show grid
    ax.grid(True)

    # Show the plot
    plt.show()

# Assuming 'scaled_contours' is your list of contours from the image processing part
plot_contours(contours)

