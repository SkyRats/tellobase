import mediapipe as mp
import cv2
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# For webcam input:
cap = cv2.VideoCapture(0)

# inicializando variáveis
vetor_dedos = [0, 0, 0, 0, 0] # dedão ao mindinho
vetor_anterior = [0, 0, 0, 0, 0]
repeticao = 0
num_repeticoes = 30

# definição dos comandos válidos
def verifica_comandos(vetor):
  if vetor == [0, 1, 0, 0, 0]:
    return '1'
  elif vetor == [0, 1, 1, 0, 0]:
    return '2'
  elif vetor == [0, 1, 1, 1, 0]:
    return '3'
  elif vetor == [0, 1, 1, 1, 1]:
    return '4'
  elif vetor == [1, 1, 1, 1, 1]:
    return 'mão aberta'
  elif vetor == [1, 0, 0, 0, 0]: # Flips laterais
    return 'flip-lateral'
  elif vetor == [1, 0, 0, 0, 1]: # Hang-Loose (foto)
    return 'hang-loose'
  elif vetor == [0, 1, 0, 0, 1]: # Rock (foto)
    return 'rock'
  elif vetor == [0, 0, 1, 0, 0]: # Dedo do meio
    return 'dedo do meio'
  else:
    return 0

# inicializando mediapipe hands
with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.75,
    min_tracking_confidence=0.75) as hands:
  
  # loop de controle
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.flip(image, 1)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    results = hands.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    # se detectar mãos na câmera
    if results.multi_hand_landmarks:
      # para cada mão detectada
      for hand_landmarks in results.multi_hand_landmarks:

        # inicializa o vetor dedos 
        vetor_dedos = [0, 0, 0, 0, 0]
        
        marks = hand_landmarks.landmark
        dist0 = ( (marks[9].x - marks[0].x)**2 + (marks[9].y - marks[0].y)**2 )**(1/2)    # distância entre pontos 0 e 9
        dist1 = ( (marks[5].x - marks[17].x)**2 + (marks[5].y - marks[17].y)**2 )**(1/2)  # distância entre pontos 5 e 17

        # tolerâncias
        TOL = 1.3
        TOL_dedao = 1.5

        # compara distâncias entre pontos para decidir quais dedos estão abertos ou fechados:

        # dedão
        dist_dedo0 = ( (marks[4].x - marks[17].x)**2 + (marks[4].y - marks[17].y)**2 )**(1/2)
        if dist_dedo0 < TOL_dedao*dist1:
          vetor_dedos[0] = 0
        else:
          vetor_dedos[0] = 1

        # indicador
        dist_dedo1 = ( (marks[8].x - marks[0].x)**2 + (marks[8].y - marks[0].y)**2 )**(1/2)
        if dist_dedo1 < TOL*dist0:
          vetor_dedos[1] = 0
        else:
          vetor_dedos[1] = 1

        # médio
        dist_dedo2 = ( (marks[12].x - marks[0].x)**2 + (marks[12].y - marks[0].y)**2 )**(1/2)
        if dist_dedo2 < TOL*dist0:
          vetor_dedos[2] = 0
        else:
          vetor_dedos[2] = 1
        
        # anelar
        dist_dedo3 = ( (marks[16].x - marks[0].x)**2 + (marks[16].y - marks[0].y)**2 )**(1/2)
        if dist_dedo3 < TOL*dist0:
          vetor_dedos[3] = 0
        else:
          vetor_dedos[3] = 1
        
        # mindinho
        dist_dedo4 = ( (marks[20].x - marks[0].x)**2 + (marks[20].y - marks[0].y)**2 )**(1/2)
        if dist_dedo4 < TOL*dist0:
          vetor_dedos[4] = 0
        else:
          vetor_dedos[4] = 1

        # desenha o modelo de mão na imagem
        mp_drawing.draw_landmarks(
          image,
          hand_landmarks,
          mp_hands.HAND_CONNECTIONS,
          mp_drawing_styles.get_default_hand_landmarks_style(),
          mp_drawing_styles.get_default_hand_connections_style())

      # se o vetor dedos tiver sido detectado várias vezes seguidas
      if repeticao > num_repeticoes:
        # encontra o comando correspondente
        codigo = verifica_comandos(vetor_dedos)
        if codigo != 0:
          print(f'Comando {codigo} detectado!')
        # ação é tomada e variáveis são reiniciadas
        repeticao = 0
        vetor_anterior = [0, 0, 0, 0, 0]
      
      # se ainda não tiver o número mínimo de repetições
      else:
        # se for o mesmo do anterior
        if vetor_dedos == vetor_anterior:
          repeticao += 1
        # se encontrar um vetor diferente
        else:
          repeticao = 0
        # vetor atual é o vetor anterior da pŕoxima iteração
        vetor_anterior = vetor_dedos

    # mostra a imagem com o modelo de mão encontrado
    cv2.imshow('MediaPipe Hands', image)
    if cv2.waitKey(5) & 0xFF == ord('q'):
      break

cap.release()
