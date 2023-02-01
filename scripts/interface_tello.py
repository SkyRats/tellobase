import mediapipe as mp
import cv2

from datetime import datetime, date
import pygame as pg

import time
import random
from os import mkdir

from djitellopy import Tello

'''
Classe Drone
--------------------------------------------------------------------------------------
__init__()            # inicia mediapipe e variáveis úteis
init_camera()         # inicia videocapture
tello_startup         # inicia objeto tello e se conecta com o drone
verify_commands()     # verifica vetor dedos e indica ações a serem tomadas pelo drone
follow_hands()        # se a mão estiver aberta, drone segue o centro da mão
hand_control()        # loop central de controle com mediapipe hands
key_control()         # loop de controle com teclado e PyGame
follow_camera_game()  # nova função com jogo de seguir a câmera
--------------------------------------------------------------------------------------

Classe Interface
--------------------------------------------------------------------------------------
__init__()            # inicia o PyGame e objeto drone
create_window()       # cria janela do PyGame com instruções de uso
print_commands()      # printa comandos possíveis ao final de cada interação
interface_loop()      # loop principal da interface com os modos de controle
--------------------------------------------------------------------------------------

'''

# Esse é apenas um simulador, ou seja, serve para teste das funções sem o drone real

# Comandos aceitos pelo mediapipe:
# - Mão aberta -> segue a mão
# - Polegar -> flips laterais
# - Hang-loose e rock -> tiram foto
# - dedo do meio -> pousar
# - indicador, V, 3 e 4 -> sem função ainda

# Comandos aceitos pelo teclado:
# - a, w, s, d -> movimentação
# - setas -> flips
# - l -> land
# - t -> take-off

# Selecionando a janela do opencv, precione 'q' para sair


class Drone:
  def __init__(self):

    self.tello_startup()
    self.mp_drawing = mp.solutions.drawing_utils
    self.mp_drawing_styles = mp.solutions.drawing_styles
    self.mp_hands = mp.solutions.hands
    self.tricks = True

    # inicializando variáveis
    self.prev_vector = [0, 0, 0, 0, 0]
    self.repeat = 0
    self.num_repeat = 30

  def tello_startup(self):
        # For Tello input:
        self.tello = Tello()  # Starts the tello object
        self.tello.connect()  # Connects to the drone

  # def init_camera(self):

  #   self.cap = cv2.VideoCapture(0)
  #   print(self.cap)

  # definição dos comandos válidos

  def verify_commands(self, vector):

    if self.tricks == False:
        if vector == [1, 0, 0, 0, 0] or vector == [0, 1, 0, 0, 0]:
            print("bateria baixa para fazer o truque!")
            self.tello.rotate_clockwise(90)
            self.tello.rotate_counter_clockwise(90) #tentativa de fazer "não"

            return "no"

    elif vector == [0, 1, 0, 0, 0]: #Flips frontais
      print(f"flip {self.orientation_y}")
      if self.orientation_y == 'foward':
        self.tello.flip_forward
      else:
        self.tello.flip_back

      return 'flip frontal'

    elif vector == [0, 1, 1, 0, 0]:
      print("Comando 2")
      return '2'

    elif vector == [0, 1, 1, 1, 0]:
      print("Comando 3")
      return '3'

    elif vector == [0, 1, 1, 1, 1]:
      print("Comando 4")
      return '4'

    elif vector == [1, 0, 0, 0, 0]: # Flips laterais
      print(f"flip {self.orientation_x}")

      if self.orientation_x == 'right':
        self.tello.flip_right
      elif self.orientation_x == 'left':
        self.tello.flip_left

      return 'flip-lateral'

    elif vector == [0, 0, 1, 0, 0] or vector == [1, 1, 0, 0, 0]: # Dedo do meio e L de land
      print(f"pousar :(")
      self.tello.land
      return 'pouso'
    
    elif vector == [0, 0, 0, 0, 0]: #Se aproxima 20cm e dá uma rodadinha
      self.tello.move_forward(20)
      self.tello.rotate_clockwise(360)
      print("Foward")
      return 'foward'
    
    elif vector == [1, 0, 0, 0, 1] or vector == [0, 1, 0, 0, 1]: # Hang-Loose e Rock
      print("Tirar e salvar foto")
      now = datetime.now()
      day = date.today()

      current_time = now.strftime("%H-%M-%S")
      current_day = day.strftime("%b-%d-%Y")

      save = f'fotos/{current_day}--{current_time}.jpg'

      cv2.imwrite(save, self.foto)
      return 'hang-loose'

    else:
      return 0

  # função de centralização no landmark 9 (central)
  def follow_hand(self, pixel):
    P_tol = 0.05
    x_error = (pixel[0] - 0.5)
    y_error = (pixel[1] - 0.5)
    if abs(x_error) < P_tol:
      x_error = 0
    if abs(y_error) < P_tol:
      y_error = 0

    #O ponto (0,0) eh o canto superior esquerdo e o ponto (1,1) eh o canto inferior direito.
    #Se x_error for negativo, então a mão está a esquerda do centro. O drone precisa ir para a esquerda. Vice-versa
    #Se y_error for negativo, a mão está acima do centro. O drone precisa ir para cima.
    #No tello: velocidade X negativa é para a esquerda e velocidade Y negativa é para cima. 
    # Manteremos, então, o sinal das variáveis na velocidade.

    self.tello.send_rc_control(10*x_error, 0, 10*y_error, 0)
    time.sleep(0.5)
    self.tello.send_rc_control(0, 0, 0, 0)

    print(f"{round(x_error, 3)}, {round(y_error, 3)}")

  # controle com as mãos
  def hand_control(self):
    # inicializando mediapipe hands
    with self.mp_hands.Hands(
        model_complexity=0,
        min_detection_confidence=0.75,
        min_tracking_confidence=0.75) as hands:

      self.tello.streamoff()  # Ends the current stream, in case it's still opened
      self.tello.streamon()  # Starts a new stream

      #loop de controle

      while True:

        success, self.image = self.tello.get_frame_read()  # Stores the current streamed frame
        self.foto= self.image
        self.abs_area = self.foto.shape[0] * self.foto.shape[1] * 10**(-8)

        if not success:
          print("Ignoring empty camera frame.")
          # If loading a video, use 'break' instead of 'continue'.
          continue

        #p ega a bateria do drone para checar se eh possivel fazer os truques

        self.battery = self.tello.get_battery()
        if self.battery <= 50:

          self.tricks = False

        else:
            
          self.tricks = True

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
          larger = 1.75

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

            # tolerâncias
            TOL = 1.3
            TOL_2 = 1.5

            # compara distâncias entre pontos para decidir quais dedos estão abertos ou fechados:

            # polegar
            finger_dist0 = ( (marks[4].x - marks[17].x)**2 + (marks[4].y - marks[17].y)**2 )**(1/2)
            if finger_dist0 < TOL_2*dist1:
              finger_vector[0] = 0
            else:
              finger_vector[0] = 1

            # orientação do polegar no eixo X
            if marks[4].x - marks[17].x > 0:
              self.orientation_x = 'right'
            else:
              self.orientation_x = 'left'
            
           #orientação do indicador no eixo Y
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

            # se o vetor dedos tiver sido detectado várias vezes seguidas
            if self.repeat > self.num_repeat:
              # verifica e realiza o comando correspondente
              self.command = self.verify_commands(finger_vector)

              # realiza o comando

            

              # ação é tomada e variáveis são reiniciadas
              self.repeat = 0
              self.prev_vector = [0, 0, 0, 0, 0]
            
            # se ainda não tiver o número mínimo de repetições para realizar ação
            else:
              # seguir se detectar mão aberta
              if finger_vector == [1, 1, 1, 1, 1]:
                pixel = (marks[9].x, marks[9].y)
                self.follow_hand(pixel)

              # senão, atualizar vetor e repetir detecção
              else:
                # se for o mesmo do anterior
                if finger_vector == self.prev_vector:
                  self.repeat += 1
                # se encontrar um vetor diferente
                else:
                  self.repeat = 0
                # vetor atual é o vetor anterior da pŕoxima iteração
                self.prev_vector = finger_vector

        # mostra a imagem com o modelo de mão encontrado
        cv2.imshow('MediaPipe Hands', self.image)
        if cv2.waitKey(5) & 0xFF == ord('q'):
          break

    self.cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)
  
  # Controle pelo teclado
  def key_control(self):

    self.tello.streamoff()  # Ends the current stream, in case it's still opened
    self.tello.streamon()  # Starts a new stream

    while True:

      success, self.image = self.tello.get_frame_read()  # Stores the current streamed frame

      if not success:
        print("Ignoring empty camera frame.")
        # If loading a video, use 'break' instead of 'continue'.
        continue

      for event in pg.event.get():
        if event.type == pg.KEYDOWN:
          if event.key == pg.K_w:
            self.tello.move_forward(20)
            # self.tello.send_rc_control(0,-10,0,0) --> velocidade constante para frente
            print("foward")
          if event.key == pg.K_z:
            self.tello.move_back(20)
             # self.tello.send_rc_control(0,10,0,0) --> velocidade constante para trás
            print("back")
            self.tello.move_back(20)
          if event.key == pg.K_a:
            self.tello.move_left(20)
             # self.tello.send_rc_control(-10,0,0,0) --> velocidade constante para esquerda
            print("left")
          if event.key == pg.K_d:
            self.tello.move_right(20)
             # self.tello.send_rc_control(0,10,0,0) --> velocidade constante para direita
            print("right")
          if event.key == pg.K_q:
            self.tello.move_up(20)
            # self.tello.send_rc_control(0,0,-10,0) --> velocidade constante para cima
            print("up")
          if event.key == pg.K_x:
            self.tello.move_down(20)
            # self.tello.send_rc_control(0,0,10,0) --> velocidade constante para baixo
            print("down")
          if event.key == pg.K_t:
            self.tello.takeoff()
            print("takeoff")
          if event.key == pg.K_l:
            self.tello.land()
            print("land")
          if event.key == pg.K_LEFT:
            self.tello.flip_left
            print("flip left")
          if event.key == pg.K_RIGHT:
            self.tello.flip_right
            print("flip right")
          if event.key == pg.K_UP:
            self.tello.flip_forward
            print("flip foward")
          if event.key == pg.K_DOWN:
            self.tello.flip_back
            print("flip back")
          if event.key == pg.K_s:
            self.tello.send_rc_control(0, 0, 0, 0) #se o drone estiver com velocidade setada
            print("stop")

      cv2.imshow('Key control', self.image)
      if cv2.waitKey(5) & 0xFF == ord('q'):
        break

    self.cap.release()
    cv2.destroyAllWindows()
    cv2.waitKey(1)

  def follow_camera_game(self):
    #creating the folder where the pictures will be stored and creating VideoWriter object
    frame_read = self.tello.get_frame_read()
    image = frame_read.frame
    # success, self.image = self.tello.get_frame_read()
    dimensions = image.shape

    date = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
    mkdir(date)
    video_out = cv2.VideoWriter(f"{date}/pictures.avi",cv2.VideoWriter_fourcc('M','J','P','G'), 1,(dimensions[0], dimensions[1]))

    #game variables. max and min height dictate how far vertically the drone moves.
    #min_time dictates the least amount of time the drone waits before taking a picture.
    #(note: min_time is affected by the difficulty)
    difficulty = None
    choices = ["left", "right", "up", "down"] 
    num_pictures = -1
    min_time = 1
    maxHeight = 170
    minHeight = 20
    #getHeight = 20

    while difficulty != "facil" and difficulty != "medio" and difficulty != "dificil":
        difficulty = input("Escolha sua dificuldade \n(facil - medio - dificil): ")
    
    if difficulty == "facil":
        dif_factor = 1

    elif difficulty == "medio":
        dif_factor = 0.85

    else:
        dif_factor = 0.7

    while(True):
        try:
            num_pictures = int(input("Quantas fotos serão tiradas? "))
            if num_pictures > 0:
                break
            print("Valor invalido")
        except:
            print("Valor invalido.")

    for i in range(1, 6):
        print("Começando em", 6 - i)
        time.sleep(1)
        
    print("Começou!\n")

    for i in range(num_pictures):

        #image = video.read()[1]
        time.sleep(0.2)
        # getHeight = self.tello.get_height()
        getHeight = 100
        print("Altura: ", getHeight)

        choice = random.choice(choices)

        if choice == "left":
            degrees = random.randint(20, 90)
            print("Girando", degrees, "° para a esquerda \n")
            # self.tello.rotate_counter_clockwise(degrees)
            time.sleep(dif_factor*(min_time + degrees * 1.75/200))

        elif choice == "right":
            degrees = random.randint(20, 90)
            print("Girando", degrees, "° para a direita \n")
            # self.tello.rotate_clockwise(degrees)
            time.sleep(dif_factor*(min_time + degrees * 1.75/200))

        elif choice == "up": 
            distance = random.randint(20, 360)
            if getHeight + distance < maxHeight:
                print("Subindo", distance, "cm \n")
                # self.tello.move_up(distance)
                #getHeight += distance

            else: #if it is higher than the max height, move down instead
                print("Descendo", distance, "cm \n")
                # self.tello.move_down(distance)
                #getHeight -= distance

            time.sleep(dif_factor*(min_time + distance/20))

        elif choice == "down": 
            
            distance = random.randint(20, 60)

            if getHeight - distance > minHeight:
                print("Descendo", distance, "cm \n")
                # self.tello.move_down(distance)
                #getHeight -= distance
                
            else: #if it is lower than the min height, move up instead
                print("Subindo", distance, "cm \n")
                # self.tello.move_up(distance)
                #getHeight += distance

            time.sleep(dif_factor*(min_time + distance/20))

        frame_read = self.tello.get_frame_read()  # Stores the current streamed frame
        image = frame_read.frame
        # success, self.image = self.cap.read()

        print("Tirando foto!")
        cv2.imwrite(f"{date}/{i}.jpg", image)
        video_out.write(image)

    print("Obrigado por jogar! Verifique a pasta", date, "para ver as fotos e o vídeo :)")


class Interface:
  def __init__(self):
    print("Bem vindo!")
    self.tello = Drone()

    self.print_commands()
    self.create_window()
    
    

  def create_window(self):
    self.dim = (500, 400)
    pg.init()
    self.win = pg.display.set_mode((self.dim[0], self.dim[1]))
    pg.display.set_caption("Pygame")
    font1 = pg.font.Font('freesansbold.ttf', 32)
    font2 = pg.font.Font('freesansbold.ttf', 26)
    white = (255, 255, 255)
    black = (0, 0, 0)
    gray = (200, 200, 200)
    self.text1 = font1.render('SkyRats - Tello', True, white, black)
    self.text2 = font2.render('Select the window to control', True, gray, black)
    self.text3 = font2.render('(0) Exit', True, gray, black)
    self.text4 = font2.render('(1) Hands', True, gray, black)
    self.text5 = font2.render('(2) Keyboard', True, gray, black)
    self.text6 = font2.render('(3) Game', True, gray, black)

  def print_commands(self):
    print("\n(0) Sair")
    print("(1) Controlar com as mãos")
    print("(2) Controlar com o teclado")
    print("(3) Jogo 'Siga a câmera'")

  def interface_loop(self):

    run = True
    while run:
      # Adicionar textos na janela do PyGame
      self.win.blit(self.text1, (30, 30))
      self.win.blit(self.text2, (30, 80))
      self.win.blit(self.text3, (30, 180))
      self.win.blit(self.text4, (30, 220))
      self.win.blit(self.text5, (30, 260))
      self.win.blit(self.text6, (30, 300))

      for event in pg.event.get():
        if event.type == pg.KEYDOWN:
          if event.key == pg.K_1:
            print("\nIniciando controle por mãos...")
            self.tello.init_camera()
            self.tello.hand_control()
            self.print_commands()

          if event.key == pg.K_2:
            print("\nIniciando controle por teclado...")
            self.tello.init_camera()
            self.tello.key_control()
            self.print_commands()

          if event.key == pg.K_3:
            print("\nIniciando jogo...")
            self.tello.init_camera()
            self.tello.follow_camera_game()
            self.print_commands()

          if event.key == pg.K_0:
            print("\nSaindo...")
            run = False

      pg.display.update()

main = Interface()
main.interface_loop()
