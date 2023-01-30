import cv2
import numpy as np


class tttDetection:
    
    def __init__(self):
        #self.capture = cv2.VideoCapture(0)
        self.frame = cv2.imread("./ttt1.jpeg")
        
        
    def get_mask(self, hsv , lower_color , upper_color):
        # Monta a mascara com os ranges selecionados
        lower = np.array(lower_color)
        upper = np.array(upper_color) 
        mask = cv2.inRange(hsv , lower, upper)
        return mask

    def get_square_area(self,img):
        # Retorna uma lista com os quadrados que tem area maior ou igual a min_area
        squareDetected = 0
        min_area = 6000 # Area minima para um quadrado ser contabilizado
    
        contours,junk=cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # Se quiser ver os contornos:
        #cv2.drawContours(self.frame,contours,-1,(255,0,0),3)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area:

                x,y,w,h=cv2.boundingRect(contour)
                squareDetected = [x,y,w,h]
                # Desenha um retangulo em torno do quadrado detectado (opcional)
                #cv2.rectangle(self.frame,(x,y),(x+w,y+h),(0,0,255),3)

        return squareDetected

    def getSquares (self, image):
        
        # Mascara azul
        # Calibrar antes de usar

        lower_blue = [ 79, 97, 173]
        upper_blue = [ 153, 218, 255]

        # Criando as máscaras
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        blue_mask = self.get_mask(hsv, lower_blue, upper_blue)


        # Os passos abaixo servem só para melhorar a visualizacao
        # da mascara para o usuario, nao interferem na deteccao
        blue_result = cv2.bitwise_and(image , image , mask= blue_mask)

        erode_size = 5
        dilate_size = 5

        erode_kernel = np.ones((erode_size, erode_size), np.float32)
        dilate_kernel = np.ones((dilate_size, dilate_size), np.float32)
        
        blue_result = cv2.dilate(blue_result, dilate_kernel)
        blue_result = cv2.erode(blue_result, erode_kernel)


        # Retorna a contagem de quadrados verdes e vermelhos e as imagens para visualizacao das mascaras
        return self.get_square_area(blue_mask), blue_result
    
    def detect_board(self):

        # success, self.frame = self.capture.read(0)
        # if success == False:
        #     raise ConnectionError

        board, blue_image = self.getSquares(self.frame)

        # Para ver as imagens das mascaras aplicadas e da camera usada

        x,y,w,h = board

        board_squaresX = [ 
            [x + w * (1/6), x + w * (3/6), x + w * (5/6)],
            [x + w * (1/6), x + w * (3/6), x + w * (5/6)],
            [x + w * (1/6), x + w * (3/6), x + w * (5/6)]
        ]

        board_squaresY = [ 
            [y + h * (3/6), y + h * (3/6), y + h * (3/6)],
            [y + h * (5/6), y + h * (5/6), y + h * (5/6)],
            [y + h * (1/6), y + h * (1/6), y + h * (1/6)]
        ]

        for i in range(3):
            for j in range(3):
                cv2.circle(self.frame,(round(board_squaresX[i][j]),round(board_squaresY[i][j])), 3, (0,0,255), -1)

        cv2.imshow('blue', blue_image)
        cv2.imshow('blue2', self.frame)
        if cv2.waitKey(20): key = cv2.waitKey(20)  
        return board_squaresX, board_squaresY


# Enquanto não chegar ao fim do tubo, ir lendo os sensores da tela
if __name__ == "__main__":
    detecting = tttDetection()
    while True:
        detecting.detect_board()         
            
    detecting.capture.release()
    cv2.destroyAllWindows()

    cv2.waitKey(0)
    