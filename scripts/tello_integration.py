import mediapipe as mp
import cv2

from datetime import datetime, date, timedelta

import pygame as pg

import time
import random
from os import mkdir
import os

from djitellopy import Tello


'''
Classe Drone
-----------------------------------------------------------------------------------------
__init__()              # inicia mediapipe e variáveis úteis
tello_startup()         # inicia objeto tello e se conecta com o drone
get_tello_battery()     # atualiza o valor da bateria e variável 'tricks' do drone
tello_no()              # informa o usuário no caso de falta de bateria para truques
return_to_pos()         # retorna ao ponto inicial após realizar um flip
verify_commands()       # verifica vetor dedos e indica ações a serem tomadas pelo drone
hand_keyboard_control() # loop central de controle com mediapipe hands e input do teclado
follow_hands()          # se a mão estiver aberta, drone segue o centro da mão
follow_camera_game()    # nova função com jogo de seguir a câmera
keep_tello_alive()      # função que manda sinal para evitar pouso automático
-----------------------------------------------------------------------------------------

Classe Interface
-----------------------------------------------------------------------------------------
__init__()              # inicia o PyGame e objeto drone
create_window()         # cria janela do PyGame com instruções de uso
print_commands()        # printa comandos possíveis ao final de cada interação
interface_loop()        # loop principal da interface com os modos de controle
-----------------------------------------------------------------------------------------
'''

# Comandos aceitos pelo mediapipe por enquanto:
# - Mão aberta -> segue a mão
# - Polegar -> flips laterais
# - Indicador -> flips frontais
# - Hang-loose e rock -> tiram foto
# - dedo do meio e L -> pousar
# - indicador, V, 3 e 4 -> sem função definida ainda

# Comandos aceitos pelo teclado:
# - a, w, s, d, shift, space -> movimentação
# - setas -> flips
# - l -> land
# - t -> take-off

# For webcam simulation ------ #
SIMULATION = False
BATTERY = 65
# ---------------------------- #

INTERFACE_FACTOR = 1    # Increase for better resolution monitor

class Drone:
    def __init__(self):
        if not SIMULATION:
            self.tello_startup()
            self.state = self.tello.get_current_state()
            print(self.state)

            if self.tello.get_height() < 5:
                self.height = 0
                self.takeoff = False
            else:
                self.height = self.tello.get_height()
                self.takeoff = True
        else:
            self.cap = cv2.VideoCapture(0)
            self.takeoff = False
        
        self.get_tello_battery()

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands

        # inicializando variáveis
        self.prev_vector = [0, 0, 0, 0, 0]
        self.repeat = 0
        self.repeat2 = 0
        self.num_repeat = 30
        self.num_repeat2 = 15
        

    # Inicialização do Tello
    def tello_startup(self):
        # For Tello input:
        self.tello = Tello()  # Starts the tello object
        self.tello.connect()  # Connects to the drone
        self.tello.streamon()

    # Atualizar valor da bateria
    def get_tello_battery(self):
        if not SIMULATION:
            self.battery = self.tello.get_battery()
        else:
            self.battery = BATTERY

        if self.battery <= 50:
            self.tricks = False
        else:
            self.tricks = True

    # Negar truques por bateria insuficiente
    def tello_no(self):
        print(f"Bateria insuficiente! {self.battery}")
        if not SIMULATION:
            self.tello.rotate_clockwise(30)
            self.tello.rotate_counter_clockwise(60)
            self.tello.rotate_clockwise(30)

    # Voltar para a posicao que estava antes do flip
    def return_to_pos(self, orientation):

        if orientation == 'foward':
            print("moving back...")
            if not SIMULATION:
                self.tello.move_back(40)
            
        elif orientation == 'back':
            print("moving foward...")
            if not SIMULATION:
                self.tello.move_forward(40)

        elif orientation == 'right':
            print("moving left...")
            if not SIMULATION:
                self.tello.move_left(40)

        elif orientation == 'left':
            print("moving right...")
            if not SIMULATION:
                self.tello.move_right(40)


    # definição dos comandos válidos
    def verify_commands(self, vector):

        # Seguir a mão
        if vector == [1, 1, 1, 1, 1]:
            pixel = (self.marks[9].x, self.marks[9].y)
            self.follow_hand(pixel, self.marks[12].y, self.marks[0].y)

        # Flips frontais
        elif vector == [0, 1, 0, 0, 0]:

            if self.tricks:

                if self.orientation_y == 'foward':
                    print("Flip foward!")
                    if not SIMULATION:
                        self.tello.flip_forward()

                else:
                    print("Flip back!")
                    if not SIMULATION:
                        self.tello.flip_back()
                
                self.return_to_pos(self.orientation_y)
            else:
                self.tello_no()

        # Flips laterais
        elif vector == [1, 0, 0, 0, 0]:

            if self.tricks:
            
                if self.orientation_x == 'right':
                    print("Flip right!")
                    if not SIMULATION:
                        self.tello.flip_right()

                elif self.orientation_x == 'left':
                    print("Flip left!")
                    if not SIMULATION:
                        self.tello.flip_left()
                
                self.return_to_pos(self.orientation_x)
            else:
                self.tello_no()
        
        # Pouso com L de land
        elif vector == [1, 1, 0, 0, 0]:
            print(f"Landing...")
            if not SIMULATION:
                self.tello.land()

        # Comando 2
        elif vector == [0, 1, 1, 0, 0]: # Da uma rodadinha
            print("Trick 2...")
            if not SIMULATION:
                self.tello.rotate_clockwise(360)

        # Comando 3
        elif vector == [0, 1, 1, 1, 0]: # Se distancia 20cm e dá uma rodadinha
            print("Trick 3...")
            if not SIMULATION:
                self.tello.move_back(20)
                self.tello.rotate_clockwise(360)
                self.tello.move_forward(20)

        # Comando 4
        elif vector == [0, 1, 1, 1, 1]: 
            print("Trick 4...")
            if not SIMULATION:
                self.tello.move_down(20)

        # Tirar foto
        elif vector == [1, 0, 0, 0, 1] or vector == [0, 1, 0, 0, 1]: # Hang-Loose e Rock
            print("Capture and save foto...")
            now = datetime.now()
            day = date.today()

            current_time = now.strftime("%H-%M-%S")
            current_day = day.strftime("%b-%d-%Y")

            if not os.path.isdir("fotos"):
                os.makedirs("fotos")

            save = f'fotos/{current_day}--{current_time}.jpg'
            cv2.imwrite(save, self.foto)


    # controle com as mãos e teclado
    def hand_keyboard_control(self):
        # Takeoff
        if self.takeoff == False:
            print("Takeoff...")
            if not SIMULATION:
                self.tello.takeoff()
                time.sleep(3)
                self.tello.move_up(40)
                time.sleep(1)
            self.takeoff = True

        # inicializando mediapipe hands
        with self.mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.75,
            min_tracking_confidence=0.75) as hands:

            self.control_loop = True
            self.start = datetime.now()

            while self.control_loop:

                if datetime.now() - self.start > timedelta(seconds=10):
                    self.keep_tello_alive()
                    self.start = datetime.now()
                
                if not SIMULATION:
                    self.image = self.tello.get_frame_read().frame  # Stores the current streamed frame

                    
                else:
                    success, self.image = self.cap.read()
                    if not success:
                        print("Ignoring empty camera frame.")
                        # If loading a video, use 'break' instead of 'continue'.
                        continue

                self.foto = self.image
                
                self.abs_area = self.foto.shape[0] * self.foto.shape[1] * 10**(-8)

                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                self.image.flags.writeable = False
                self.image = cv2.flip(self.image, 1)
                self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

                results = hands.process(self.image)

                # Draw the hand annotations on the image.
                self.image.flags.writeable = True
                self.image = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)

                # se detectar mãos na câmera
                if results.multi_hand_landmarks:
                    true_hand = None
                    larger = 0.3 # Para detectar mãos menores, diminuir essa variável

                    # para cada mão detectada, verificar qual é a maior (e se não é muito pequena)
                    for potential_hand in results.multi_hand_landmarks:
                        # coordenadas dos pontos 0, 5 e 17
                        lm0 = potential_hand.landmark[0]
                        lm5 = potential_hand.landmark[5]
                        lm17 = potential_hand.landmark[17]

                        # área do triângulo central
                        area = 1/2 * abs(lm0.x * (lm5.y - lm17.y) + lm5.x * (lm17.y - lm0.y) + lm17.x * (lm0.y - lm5.y)) / self.abs_area
                        if area > larger:
                            true_hand = potential_hand
                            larger = area
                    
                    # se existir uma mão única perto o suficiente da câmera
                    if true_hand:
                        hand_landmarks = true_hand

                        # define o vetor dedos 
                        finger_vector = [0, 0, 0, 0, 0]
                        marks = hand_landmarks.landmark
                        dist0 = ( (marks[9].x - marks[0].x)**2 + (marks[9].y - marks[0].y)**2 )**(1/2)    # distância entre pontos 0 e 9
                        dist1 = ( (marks[5].x - marks[17].x)**2 + (marks[5].y - marks[17].y)**2 )**(1/2)  # distância entre pontos 5 e 17

                        # tolerâncias (Quanto menores, mais conservadora é a detecção)
                        TOL = 1.3   # Geral
                        TOL_2 = 1.5 # Polegar

                        # compara distâncias entre pontos para decidir quais dedos estão abertos ou fechados:

                        # polegar
                        finger_dist0 = ( (marks[4].x - marks[17].x)**2 + (marks[4].y - marks[17].y)**2 )**(1/2)
                        if finger_dist0 < TOL_2*dist1:
                            finger_vector[0] = 0
                        else:
                            finger_vector[0] = 1

                        # orientação do polegar no eixo X
                        if marks[4].x - marks[17].x < 0:
                            self.orientation_x = 'right'
                        else:
                            self.orientation_x = 'left'
                        
                        # orientação do indicador no eixo Y
                        if marks[8].y - marks[5].y < 0:
                            self.orientation_y = 'foward'
                        else:
                            self.orientation_y = 'back'
                        
                        # indicador
                        finger_dist1 = ( (marks[8].x - marks[0].x)**2 + (marks[8].y - marks[0].y)**2 )**(1/2)
                        if finger_dist1 < TOL*dist0:
                            finger_vector[1] = 0
                        else:
                            finger_vector[1] = 1

                        # médio
                        finger_dist2 = ( (marks[12].x - marks[0].x)**2 + (marks[12].y - marks[0].y)**2 )**(1/2)
                        if finger_dist2 < TOL*dist0:
                            finger_vector[2] = 0
                        else:
                            finger_vector[2] = 1
                        
                        # anelar
                        finger_dist3 = ( (marks[16].x - marks[0].x)**2 + (marks[16].y - marks[0].y)**2 )**(1/2)
                        if finger_dist3 < TOL*dist0:
                            finger_vector[3] = 0
                        else:
                            finger_vector[3] = 1
                        
                        # mindinho
                        finger_dist4 = ( (marks[20].x - marks[0].x)**2 + (marks[20].y - marks[0].y)**2 )**(1/2)
                        if finger_dist4 < TOL*dist0:
                            finger_vector[4] = 0
                        else:
                            finger_vector[4] = 1

                        # desenha o modelo de mão na imagem
                        self.mp_drawing.draw_landmarks(
                            self.image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())

                        self.marks = marks

                        # Se detectar uma mão aberta
                        if finger_vector == [1, 1, 1, 1, 1]:

                            if self.repeat2 > self.num_repeat2: # Se tiver detectado essa mão aberta várias vezes
                                self.get_tello_battery()
                                self.command = self.verify_commands(finger_vector)
                                
                                # Reiniciar variáveis
                                self.prev_vector = [0, 0, 0, 0, 0]
                                self.repeat2 = 0
                            
                            # Se ainda não tiver o número mínimo de repetições para seguir a mão
                            else: 
                                # mesmo vetor encontrado
                                if finger_vector == self.prev_vector:
                                    self.repeat2 += 1
                                # se encontrar um vetor diferente
                                else:
                                    self.repeat2 = 0
                                # vetor atual é o vetor anterior da pŕoxima iteração
                                self.prev_vector = finger_vector

                        # Se detectar algum vetor específico várias vezes
                        elif self.repeat > self.num_repeat:
                            # verifica e realiza o comando correspondente
                            self.get_tello_battery()
                            self.command = self.verify_commands(finger_vector)

                            # Reiniciar variáveis
                            self.repeat = 0
                            self.prev_vector = [0, 0, 0, 0, 0]

                        # se ainda não tiver o número mínimo de repetições para realizar ação
                        else:
                            # mesmo vetor encontrado
                            if finger_vector == self.prev_vector:
                                self.repeat += 1
                            # se encontrar um vetor diferente
                            else:
                                self.repeat = 0
                            # vetor atual é o vetor anterior da pŕoxima iteração
                            self.prev_vector = finger_vector

                    # else:
                    #     # Zerar velocidade do drone
                    #     if not SIMULATION:
                    #         self.tello.send_rc_control(0, 0, 0, 0)

                # mostra a imagem com o modelo de mão encontrado
                cv2.imshow('MediaPipe Hands', self.image)
                if cv2.waitKey(5) & 0xFF == ord('q'):
                    self.control_loop = False

                # Se detectar alguma tecla do teclado sendo pressionada
                for event in pg.event.get():
                    self.get_tello_battery()
                    if event.type == pg.KEYDOWN:
                        if event.key == pg.K_w:
                            print("Going foward...")
                            if not SIMULATION:
                                self.tello.move_forward(20)

                        if event.key == pg.K_s:
                            print("Going back...")
                            if not SIMULATION:
                                self.tello.move_back(20)

                        if event.key == pg.K_a:
                            print("Going left...")
                            if not SIMULATION:
                                self.tello.move_left(20)

                        if event.key == pg.K_d:
                            print("Going right...")
                            if not SIMULATION:
                                self.tello.move_right(20)

                        if event.key == pg.K_SPACE:
                            print("Going up...")
                            if not SIMULATION:
                                self.tello.move_up(20)

                        if event.key == pg.K_LSHIFT:
                            print("Going down...")
                            if not SIMULATION:
                                self.tello.move_down(20)

                        if event.key == pg.K_t:
                            print("Takeoff...")
                            if not SIMULATION:
                                self.tello.takeoff()

                        if event.key == pg.K_l:
                            print("Landing...")
                            if not SIMULATION:
                                self.tello.land()

                        if event.key == pg.K_b:
                            if not SIMULATION:
                                print(f"Bateria: {self.tello.get_battery()}")
                            else:
                                print(f"Bateria: ?")

                        if event.key == pg.K_LEFT:
                            if self.tricks == True:
                                print("Flip left!")
                                if not SIMULATION:
                                    self.tello.flip_left()
                                self.return_to_pos('left')
                            else: 
                                self.tello_no()

                        if event.key == pg.K_RIGHT :
                            if self.tricks == True:
                                print("Flip right!")
                                if not SIMULATION:
                                    self.tello.flip_right()
                                self.return_to_pos('right')
                            else: 
                                self.tello_no()

                        if event.key == pg.K_UP:
                            if self.tricks == True:
                                print("Flip foward!")
                                if not SIMULATION:
                                    self.tello.flip_forward()
                                self.return_to_pos('foward')
                            else: 
                                self.tello_no()

                        if event.key == pg.K_DOWN:
                            if self.tricks == True:
                                print("Flip back!")
                                if not SIMULATION:
                                    self.tello.flip_back()
                                self.return_to_pos('back')
                            else: 
                                self.tello_no()

                        if event.key == pg.K_e:
                            print("Sending zero velocity...")
                            if not SIMULATION:
                                self.tello.send_rc_control(0, 0, 0, 0)

                        # EMERGÊNCIA - PARA TODOS OS MOTORES
                        if event.key == pg.K_BACKSPACE:
                            print("STOP ALL MOTORS!!!")
                            if not SIMULATION:
                                self.tello.emergency() # se o drone estiver com velocidade setada

                        if event.key == pg.K_k:
                            print("Checking tello conection...")
                            if not SIMULATION:
                                self.keep_tello_alive()

                        if event.key == pg.K_ESCAPE:
                            self.control_loop = False

        cv2.destroyAllWindows()
        cv2.waitKey(1)
    
    def keep_tello_alive(self):
        # Manda sinal para o tello nao pousar. Criamos uma funcao, pois chamaremos em outra classe
        if not SIMULATION:
            self.tello.send_control_command("command")

    # função de centralização no landmark 9 (central)
    def follow_hand(self, pixel, ext_up, ext_down):

        P_tol = 0.075 # Quanto maior -> menos responsivo
        z_error = abs(ext_up - ext_down)
        x_error = (pixel[0] - 0.5)
        y_error = (pixel[1] - 0.5)
        if abs(x_error) < P_tol:
            x_error = 0
        if abs(y_error) < P_tol:
            y_error = 0

        if not SIMULATION:
            self.height = self.tello.get_height()
            if self.height > 200:
                print("Too high, mooving down...")
                self.tello.move_down(40)
                print(self.height)

        # O ponto (0,0) eh o canto superior esquerdo e o ponto (1,1) eh o canto inferior direito.
        # Se x_error for negativo, então a mão está a esquerda do centro. O drone precisa ir para a esquerda . Vice-versa
        # Se y_error for negativo, a mão está acima do centro. O drone precisa ir para cima (cima dele mesmo).

        # Se estiver muito perto
        if z_error > 0.8:
            print("Too close! Moving backwards...")
        else:
            z_error = 0

        # Cálculo das velocidades
        x_vel = -int(100*x_error)
        y_vel = -int(50*z_error)
        # z_vel = -int(100*y_error)

        z_vel = 0

        if x_vel != 0 or y_vel != 0 or z_vel != 0:
            if not SIMULATION:
                self.tello.send_rc_control(x_vel, y_vel, z_vel, 0)
                time.sleep(0.6)
                # Note: “x”, “y”, and “z” values can’t be set between -20 – 20 simultaneously.
                self.tello.send_rc_control(0, 0, 0, 0)
            else:
                print(f"Send RC control: {x_vel}, {y_vel} ,{z_vel}, 0")

        # print(f"{round(x_error, 3)}, {round(y_error, 3)}")


    # Jogo das fotos
    def follow_camera_game(self):

        if self.takeoff == False:
            print("Takeoff...")
            if not SIMULATION:
                self.tello.takeoff()
                time.sleep(5)
                self.tello.move_up(40)
            self.takeoff = True

        # creating the folder where the pictures will be stored and creating VideoWriter object
        if not SIMULATION:
            frame_read = self.tello.get_frame_read()
            image = frame_read.frame
        else:
            # frame_read = self.cap.read()
            # image2 = frame_read
            getHeight = 100
        
        # success, self.image = self.tello.get_frame_read()
        # dimensions = image.shape

        date = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        mkdir(date)
        # video_out = cv2.VideoWriter(f"{date}/pictures.avi", cv2.VideoWriter_fourcc('M','J','P','G'), 1,(dimensions[0], dimensions[1]))

        #game variables. max and min height dictate how far vertically the drone moves.
        #min_time dictates the least amount of time the drone waits before taking a picture.
        #(note: min_time is affected by the difficulty)
        difficulty = None
        choices = ["left", "right", "up", "down"]
        deg_choices = [30, 40, 50, 60, 70, 80]
        dist_choices = [20, 30, 40]

        num_pictures = -1
        maxHeight = 170
        minHeight = 20

        # Inicializando jogo
        while difficulty != "1" and difficulty != "2" and difficulty != "3":
            difficulty = input("Escolha sua dificuldade \n(1 - 2 - 3): ")

        while True:
            try:
                num_pictures = int(input("Quantas fotos serão tiradas? "))
                if num_pictures > 0:
                    break
                print("Valor inválido")
            except:
                print("Valor inválido.")

        self.keep_tello_alive()
        for i in range(1, 4):
            print("Começando em", 4 - i)
            time.sleep(1)
            
        print("Começou!")

        # Start Game
        for i in range(num_pictures):

            if not SIMULATION:
                getHeight = self.tello.get_height()

            print("\nAltura: ", getHeight)

            choice = random.choice(choices)

            if choice == "left":
                degrees = deg_choices[random.randint(0, 5)]
                print("Girando", degrees, "° para a esquerda")
                if not SIMULATION:
                    self.tello.rotate_counter_clockwise(degrees)

            elif choice == "right":
                degrees = deg_choices[random.randint(0, 5)]
                print("Girando", degrees, "° para a direita")
                if not SIMULATION:
                    self.tello.rotate_clockwise(degrees)

            elif choice == "up": 
                distance = dist_choices[random.randint(0, 2)]
                if getHeight + distance < maxHeight:
                    print("Subindo", distance, "cm")
                    if not SIMULATION:
                        self.tello.move_up(distance)
                        getHeight += distance

                else: #if it is higher than the max height, move down instead
                    print("Descendo", distance, "cm")
                    if not SIMULATION:
                        self.tello.move_down(distance)
                        getHeight -= distance

            elif choice == "down": 
                distance = dist_choices[random.randint(0, 2)]
                if getHeight - distance > minHeight:
                    print("Descendo", distance, "cm")
                    if not SIMULATION:
                        self.tello.move_down(distance)
                    getHeight -= distance
                    
                else: #if it is lower than the min height, move up instead
                    print("Subindo", distance, "cm")
                    if not SIMULATION:
                        self.tello.move_up(distance)
                    getHeight += distance

            time.sleep(5-int(difficulty))

            if not SIMULATION:
                frame_read = self.tello.get_frame_read()  # Stores the current streamed frame
                self.frame = frame_read.frame
            else:
                self.cap.release()
                self.cap = cv2.VideoCapture(0)
                success, self.frame = self.cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    # If loading a video, use 'break' instead of 'continue'.
                    continue

            print("Tirando foto!")
            cv2.imwrite(f"{date}/{i}.png", self.frame)
            # video_out.write(image)

            time.sleep(0.5)

        print("\nObrigado por jogar! Verifique a pasta", date, "para ver as fotos :)")
        # self.cap.release()
        cv2.destroyAllWindows()
        cv2.waitKey(1)

class Interface:
    def __init__(self):
        print("\nBem vindo!")
        self.tello = Drone()

        self.print_commands()
        self.create_window()

    def create_window(self):
        self.f = INTERFACE_FACTOR
        self.dim = (500*self.f, 480*self.f)
        pg.init()
        self.win = pg.display.set_mode((self.dim[0], self.dim[1]))
        pg.display.set_caption("Pygame")
        font1 = pg.font.Font('freesansbold.ttf', int(16+self.f*16))
        font2 = pg.font.Font('freesansbold.ttf', int(10+self.f*16))
        white = (255, 255, 255)
        black = (0, 0, 0)
        gray = (200, 200, 200)
        self.text1 = font1.render('SkyRats - Tello', True, white, black)
        self.text2 = font2.render('Select this window to control', True, gray, black)
        self.text3 = font2.render('(0) Exit', True, gray, black)
        self.text4 = font2.render('(1) Hands and Keyboard', True, gray, black)
        self.text5 = font2.render('(2) Tic Tac Toe', True, gray, black)
        self.text6 = font2.render('(3) Camera Game', True, gray, black)
        self.text7 = font2.render('(Esc) To exit game mode', True, gray, black)

    def print_commands(self):
        print("\n(0) Sair")
        print("(1) Controlar com as mãos e teclado")
        print("(2) Jogo da velha")
        print("(3) Jogo 'Siga a câmera'")

    def interface_loop(self):

        run = True
        self.tello.keep_tello_alive() # Manda sinal para o drone não pousar
        self.start = datetime.now()
        d0 = 30*self.f
        d1 = 40*self.f
        d2 = 50*self.f
        while run:
            if datetime.now() - self.start > timedelta(seconds=10):
                self.tello.keep_tello_alive()
                self.start = datetime.now()
            # Adicionar textos na janela do PyGame
            self.win.blit(self.text1, (d0, d0))
            self.win.blit(self.text2, (d0, (d0+d2)))
            self.win.blit(self.text3, (d0, (d0+3*d2)))
            self.win.blit(self.text4, (d0, (d0+3*d2+d1)))
            self.win.blit(self.text5, (d0, (d0+3*d2+2*d1)))
            self.win.blit(self.text6, (d0, (d0+3*d2+3*d1)))
            self.win.blit(self.text7, (d0, (d0+5*d2+3*d1)))

            for event in pg.event.get():
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_0:
                        print("\nSaindo...")
                        run = False

                    if event.key == pg.K_1:
                        print("\nIniciando controle por mãos e teclado...")
                        # self.tello.init_camera()
                        self.tello.hand_keyboard_control()
                        self.print_commands()

                    if event.key == pg.K_2:
                        print("\nIniciando jogo da velha...")
                        # self.tello.init_camera()
                        self.print_commands()

                    if event.key == pg.K_3:
                        print("\nIniciando jogo da câmera...")
                        # self.tello.init_camera()
                        self.tello.follow_camera_game()
                        self.print_commands()

            pg.display.update()

main = Interface()
main.interface_loop()
