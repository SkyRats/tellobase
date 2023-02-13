import cv2
import numpy as np
import time
from tttAI import IA

'''
Class tttDetection
--------------------------------------------------------------------------------------
__init__()            # Inicia a captura de tela e o contorno dos players
filter_small()        # Filtra contornos pequenos que possam atrapalhar a detecção
match_shape ()        # Detecta a jogada em um quadrado específico do tabuleiro
most_frequent()       # Dado uma lista, retorna a string mais recorrente
get_common()          # Dado um conjunto de tabuleiros, seleciona as jogadas mais recorrentes
get_mask()            # Gera a máscara de cores no range informado
get_squares()         # Identifica uma região com área mínima na cor desejada (azul)
detect_board()        # Identifica o tabuleiro na imagem e retorna suas dimensões e coordenadas
read_board()          # Lê o tabuleiro e retorna seu estado atual de jogadas
print_board()         # Imprime o estado do tabuleiro no terminal
play_ttt()            # Conduz o jogo, esperando e lendo as jogadas
--------------------------------------------------------------------------------------


'''

class tttDetection:
    
    def __init__(self):
        #self.capture = cv2.VideoCapture(0)
        self.ia = IA()
        self.frame = cv2.imread("./ttt1.jpeg")

        # Contornos do player1 (X)
        player1 = cv2.imread("./player1.png")
        player1_gray = 255 - cv2.cvtColor(player1, cv2.COLOR_BGR2GRAY)
        
        _, thresh1 = cv2.threshold(player1_gray, 127, 255, 0)
        contours, _ = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        self.player1_cnt = contours[0]

        # Contornos do player2 (Check mark)
        player2 = cv2.imread("./player2.png")
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
                # print("player1")
                return 1

            elif cv2.matchShapes(contour, self.player2_cnt, 1, 0.0) < max_tolerance:
                # print("player2")
                return -1

        # print("not played")
        return 0
    
    def most_frequent(self, list):

        mostFrequent = 0
        result = list[0]
     
        for i in list:
            frequency = list.count(i)
            if(frequency > mostFrequent):
                mostFrequent = frequency
                result = i

        return result

    def get_common(self, boards):

        board_common = [[], [], []]
        for i in range(3):
            for j in range(3):
                play = []
                for board in boards:
                    play.append(board[i][j])

                board_common[i].append(self.most_frequent(play))
        
        return board_common
      
    def get_mask(self, hsv , lower_color , upper_color):
        # Monta a mascara com os ranges selecionados
        lower = np.array(lower_color)
        upper = np.array(upper_color) 
        mask = cv2.inRange(hsv , lower, upper)
        return mask

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

        # Retorna uma lista com os quadrados que tem area maior ou igual a min_area
        square_detected = 0

        min_area = 6000 # Area minima para um quadrado ser contabilizado
    
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Se quiser ver os contornos:
        #cv2.drawContours(self.frame,contours,-1,(255,0,0),3)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area:

                x,y,w,h=cv2.boundingRect(contour)
                square_detected = [x,y,w,h]

                # Desenha um retangulo em torno do quadrado detectado (opcional)
                #cv2.rectangle(self.frame,(x,y),(x+w,y+h),(0,0,255),3)

        return square_detected, blue_result
    
    def detect_board(self, tries, wait_time = 0.025, max_tolerance = .6):
        boards = []
        print("Quando encontrar o tabuleiro, aperte espaço com a janela selecionada.")
        while(True):

            #_, self.frame = self.capture.read(0)
            self.frame = cv2.imread("./ttt1.jpeg")
            frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            _, frame_thresh = cv2.threshold(frame_gray, 127, 255,0)
            cv2.imshow("frame threshold", frame_thresh)

            if(cv2.waitKey(1) == ord(" ")):
                break

        for _ in range(tries):
            #_, self.frame = self.capture.read(0)
            self.frame = cv2.imread("./ttt1.jpeg")

            board, _ = self.get_squares(self.frame)
            if board != 0:

                #cv2.imshow("frame threshold", frame_thresh)
                boards.append(self.read_board(board, max_tolerance))

            time.sleep(wait_time)

        return self.get_common(boards)
    
    def read_board(self, board, max_tolerance = .6):

        x,y,w,h = board
        board_simple = [[], [], []]

        frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        _, frame_thresh = cv2.threshold(frame_gray, 127, 255, 0)

        cv2.imshow("frame threshold", frame_thresh)

        for i in range(3):
            for j in range(3):
                
                square = frame_thresh[(y + h * (j)//3) : (y + h * (j + 1)//3), (x + w * (i)//3) : (x + w * (i + 1)//3)]
                square_cnts, _ = cv2.findContours(square, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                square_cnts = self.filter_small(square_cnts, 25)

                board_simple[j].append(self.match_shape(square_cnts, max_tolerance))
        
        return board_simple

    def print_board(self, board):
        print(f" {board[0][0]} | {board[0][1]} | {board[0][2]}")
        print("-----------")
        print(f" {board[1][0]} | {board[1][1]} | {board[1][2]}")
        print("-----------")
        print(f" {board[2][0]} | {board[2][1]} | {board[2][2]}")
        print()
        print()
        print("###########################################")

    def play_ttt(self):

        board_state = []
        start_detetection = False

        c_choice = "X"
        h_choice = "V"

        print("Começando o jogo da velha!!")
        print("-------------------------------")

        first = input("Você que começar? [S/N] ")

        # Rodando o loop enquanto o tabuleiro não estiver preenchido nem ganho        
        while len(self.ia.empty_cells(self.ia.board)) > 0 and not self.ia.game_over(self.ia.board):
            
            if first in'Nn':
                self.ia.ai_turn(c_choice, h_choice)
                # drone indica a jogada
                first = ''

            start_detetection = input("Digite qualquer coisa para iniciar uma detecção  \n")

            board_state = self.detect_board(tries = 25)
            print("Tabuleiro encontrado!\n")
            print("\nSituação atual do jogo:")
            self.ia.board = board_state

            self.ia.ai_turn(c_choice, h_choice)
            self.print_board(board_state)
            # drone indica a jogada

if __name__ == "__main__":

    detecting = tttDetection()
    detecting.play_ttt()

    detecting.capture.release()
    cv2.destroyAllWindows()

    
