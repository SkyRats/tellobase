import cv2
import numpy as np


class tttDetection:
    
    def __init__(self):
        self.capture = cv2.VideoCapture(0)
        #self.frame = cv2.imread("./ttt1.jpeg")

        player1 = cv2.imread("player1.png")
        player1_gray = 255 - cv2.cvtColor(player1, cv2.COLOR_BGR2GRAY)
        
        _, thresh1 = cv2.threshold(player1_gray, 127, 255, 0)
        contours, _ = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.player1_cnt = contours[0]

        player2 = cv2.imread("player2.png")
        player2_gray = 255 - cv2.cvtColor(player2, cv2.COLOR_BGR2GRAY)

        _, thresh2 = cv2.threshold(player2_gray, 127, 255, 0)
        contours, _ = cv2.findContours(thresh2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.player2_cnt = contours[0]

    def filter_small(self, contours, min_area):
        
        filtered_contours = []

        for contour in contours:
            if cv2.contourArea(contour) >= min_area:
                filtered_contours.append(contour)

        return filtered_contours

    def match_shape(self, square_cnts, max_tolerance):
        for contour in square_cnts:
                        
            if cv2.matchShapes(contour, self.player1_cnt, 1, 0.0) < max_tolerance:
                print("player1")
                return "player1"

            elif cv2.matchShapes(contour, self.player2_cnt, 1, 0.0) < max_tolerance:
                print("player2")
                return "player2"

        print("not played")
        return "not played"
    
    def most_frequent(self, list):

        counter = 0
        result = list[0]
     
        for i in list:
            frequency = list.count(i)
            if(frequency > counter):
                counter = frequency
                result = i

        return result

    def get_common(self, boards):
        board_common = [[], [], []]
        for i in range(3):
            for j in range(3):
                play = []
                for board in boards:
                    play.append(board[i][j])
                #play = np.array(play)
                board_common[i].append(self.most_frequent(play))
        
        return board_common

      
    def get_mask(self, hsv , lower_color , upper_color):
        # Monta a mascara com os ranges selecionados
        lower = np.array(lower_color)
        upper = np.array(upper_color) 
        mask = cv2.inRange(hsv , lower, upper)
        return mask

    def get_square_area(self,img):
        # Retorna uma lista com os quadrados que tem area maior ou igual a min_area
        square_detected = 0

        min_area = 6000 # Area minima para um quadrado ser contabilizado
    
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Se quiser ver os contornos:
        #cv2.drawContours(self.frame,contours,-1,(255,0,0),3)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area:

                x,y,w,h=cv2.boundingRect(contour)
                square_detected = [x,y,w,h]

                # Desenha um retangulo em torno do quadrado detectado (opcional)
                #cv2.rectangle(self.frame,(x,y),(x+w,y+h),(0,0,255),3)

        return square_detected

    def get_squares (self, image):
        
        # Mascara azul
        # Calibrar antes de usar
        lower_blue = [ 96, 85, 0]
        upper_blue = [ 145, 230, 255]

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
    
    def detect_board(self, tries = 1, max_tolerance = .6, wait_time = 50):
        boards = []
        for _ in range(tries):
            success, self.frame = self.capture.read(0)
            if success == False:
                raise ConnectionError

            board, blue_image = self.get_squares(self.frame)

            if board != 0:

                # Para ver as imagens das mascaras aplicadas e da camera usada
                x,y,w,h = board

                """
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
                """
                
                board_simple = [[], [], []]
                frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
                _, frame_thresh = cv2.threshold(frame_gray, 127, 255,0)

                cv2.imshow("frame threshold", frame_thresh)

                for i in range(3):
                    for j in range(3):
                        
                        square = frame_thresh[(y + h * (j)//3) : (y + h * (j + 1)//3), (x + w * (i)//3) : (x + w * (i + 1)//3)]
                        square_cnts, _ = cv2.findContours(square, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                        square_cnts = self.filter_small(square_cnts, 25)

                        print("row: ", j + 1, "collum: ", i + 1)

                        board_simple[i].append(self.match_shape(square_cnts, max_tolerance))

                        print("------------------")
                        square = cv2.cvtColor(square, cv2.COLOR_GRAY2BGR)
                        cv2.imshow("", square)

                cv2.waitKey(wait_time)
                cv2.imshow('blue', blue_image)
                print("#----------------#")
                boards.append(board_simple)
            
        board_simple = self.get_common(boards)
        print("result after", tries, "tries:")
        print(board_simple)
        return board_simple

# Enquanto não chegar ao fim do tubo, ir lendo os sensores da tela
if __name__ == "__main__":
    
    #adjust the board
    capture = cv2.VideoCapture(0)
    while(True):

        
        success, frame = capture.read(0)
        if cv2.waitKey(1) == ord(" "):
            break
        cv2.imshow("line up board", frame)
        
    capture.release()

    detecting = tttDetection()
    while True:
        board = detecting.detect_board(25)
        cv2.waitKey(0)

    detecting.capture.release()
    cv2.destroyAllWindows()

    cv2.waitKey(0)
    