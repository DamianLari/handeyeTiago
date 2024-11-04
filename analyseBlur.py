import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

def analyze_gradients(image_path,id):
    image = cv2.imread(image_path)

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #gray_image = image

    grad_x = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=1)  
    grad_y = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=1)

    gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)

    plt.hist(gradient_magnitude.ravel(), bins=256, range=(0, 255), color='blue', alpha=0.7)
    plt.title("Histogramme des gradients")
    plt.xlabel("Magnitude du gradient")
    plt.ylabel("Nombre de pixels")
    plt.savefig("gradient_histogram.png")
    print("Histogramme des gradients sauvegardé sous 'gradient_histogram.png'")

    seuil_bas = 10  
    seuil_haut = 150  

    total_pixels = gray_image.size
    flou_pixels = np.sum(gradient_magnitude < seuil_bas)
    net_pixels = np.sum(gradient_magnitude > seuil_haut)
    interm_pixels = total_pixels - (flou_pixels + net_pixels)  

    proportion_flou = flou_pixels / total_pixels * 100
    proportion_net = net_pixels / total_pixels * 100
    proportion_interm = interm_pixels / total_pixels * 100

    print(f"Nombre total de pixels : {total_pixels}")
    print(f"Pixels flous (faible gradient) : {flou_pixels} ({proportion_flou:.2f}%)")
    print(f"Pixels nets (fort gradient) : {net_pixels} ({proportion_net:.2f}%)")
    print(f"Pixels intermédiaires : {interm_pixels} ({proportion_interm:.2f}%)")

    plt.figure(figsize=(10, 5))
    plt.subplot(1, 3, 1)
    plt.imshow(gray_image, cmap='gray')
    plt.title('Image en niveaux de gris')

    plt.subplot(1, 3, 2)
    plt.imshow(gradient_magnitude, cmap='gray')
    plt.title('Magnitude du gradient')

    plt.subplot(1, 3, 3)
    result_image = np.zeros_like(gray_image)
    result_image[gradient_magnitude < seuil_bas] = 50  
    result_image[gradient_magnitude > seuil_haut] = 200  
    plt.imshow(result_image, cmap='gray')
    plt.title('Pixels flous et nets')

    plt.tight_layout()
    plt.savefig(f"blur/image{id}/histogram{id}.png")

    cv2.imwrite(f'blur/image{id}/gradient_magnitude.png',gradient_magnitude )
    cv2.imwrite(f'blur/image{id}/result_image.png',result_image )

    print("Analyse des gradients sauvegardée sous 'gradient_analysis.png'")
    print("====================================================================================")

def analyze_aruco_image(image_path,id):
    image = cv2.imread(image_path)

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    enhanced_image = cv2.equalizeHist(gray_image)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    enhanced_image_clahe = clahe.apply(gray_image)

    kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]]) 
    sharp_image = cv2.filter2D(enhanced_image_clahe, -1, kernel)

    plt.figure(figsize=(10, 6))
    plt.subplot(1, 3, 1)
    plt.imshow(gray_image, cmap='gray')
    plt.title('Image en niveaux de gris')

    plt.subplot(1, 3, 2)
    plt.imshow(enhanced_image_clahe, cmap='gray')
    plt.title('Contraste amélioré (CLAHE)')

    plt.subplot(1, 3, 3)
    plt.imshow(sharp_image, cmap='gray')
    plt.title('Image avec filtre de netteté')
    plt.savefig(f'blur/image{id}/analyse{id}.png')
    cv2.imwrite(f'blur/image{id}/image.png',image )
    cv2.imwrite(f'blur/image{id}/gray_image.png',gray_image )
    cv2.imwrite(f'blur/image{id}/enhanced_image_clahe.png',enhanced_image_clahe )
    cv2.imwrite(f'blur/image{id}/sharp_image.png',sharp_image )

analyze_aruco_image('images/image_0000.png',0)
analyze_gradients('images/image_0000.png',0)  
analyze_aruco_image('images/image_0001.png',1)
analyze_gradients('images/image_0001.png',1)  
analyze_aruco_image('images/image_0002.png',2)
analyze_gradients('images/image_0002.png',2)  
analyze_aruco_image('images/image_0006.png',6)
analyze_gradients('images/image_0006.png',6)  

