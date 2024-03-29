from djitellopy import Tello
import cv2
import mediapipe as mp
from math import sqrt

#inicializando a IA
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_face_mesh = mp.solutions.face_mesh

tellinho = Tello()
tellinho.connect()
tellinho.streamon()

tellinho.takeoff()
tellinho.move_up(80)

#funcoes utilizadas
def distance(x1, y1, x0, y0):
  return sqrt((x1-x0)**2 + (y1-y0)**2)

def pisca(v1, d1, v2, d2, img):
    global piscando

    olhoDireito = v1 / d1
    olhoEsquerdo = v2 / d2
    if olhoDireito < 0.05 or olhoEsquerdo < 0.05:
        # Piscando os dois olhos
        if olhoDireito < 0.05 and olhoEsquerdo < 0.05 and piscando == False:
            piscando = True
            print('PISCOU')
            if(tellinho.get_battery()>50):
              tellinho.flip('b')
              tellinho.move_forward(60)
            else:
               tellinho.land()         
          
        # Piscando o olho direito
        if olhoDireito < 0.05 and olhoEsquerdo >= 0.09 and piscando == False:
            piscando = True
            print('Piscou Direito ;)')
            if(tellinho.get_battery()>50):
              tellinho.flip('l')
              tellinho.move_right(50)
            else:
               tellinho.land()            

        # Piscando o olho esquerdo
        if olhoEsquerdo < 0.05 and olhoDireito >= 0.09 and piscando == False:
            piscando = True
            print('Piscou Esquerdo ;)')
            if(tellinho.get_battery()>50):
              tellinho.flip('r')
              tellinho.move_left(50)
            else:
               tellinho.land()

    elif olhoEsquerdo >= 0.35 and olhoDireito >= 0.35 and piscando == False:
        print("Pousaaaar") 
        tellinho.land()

    else:
        piscando = False

    return img

def faceTrack(img):
  height, width, _ = img.shape
  #acha centro da imagem
  Xm = round(width/2)
  Ym = round(height/2)
  #Marca o centro da camera do tello
  cv2.circle(img, (Xm, Ym), radius=1, color=(255, 0, 0), thickness=5)
  #Desenha o quadrado de deteccao
  cv2.rectangle(img, (Xm-100, Ym+100), (Xm+100, Ym-100), color=(0, 0, 255), thickness=5)
  #acha o centro do rosto
  Xrosto = round((positionsx["243"]+positionsx["463"])/2)
  Yrosto = positionsy["243"]
  cv2.circle(img, (Xrosto, Yrosto), radius=1, color=(255, 0, 0), thickness=5)
  #checa se o rosto esta no centro
  if Xm-100<=Xrosto<=Xm+100 and Ym-100<=Yrosto<=Ym+100:
    print("Rosto centralizado")
  #centralizacao
  else:
    if Ym-Yrosto>100:
      tellinho.move_up(20)
    elif Ym-Yrosto<-100:
       tellinho.move_down(20)
    elif(Xm-Xrosto<-100):
      tellinho.rotate_clockwise(15) 
    elif (Xm-Xrosto>100):
      tellinho.rotate_counter_clockwise(15)       

#Main Loop 
while (cv2.waitKey(10) != ord(" ")):
  try:
    print("----Bateria:", tellinho.get_battery())
    frame = tellinho.get_frame_read().frame
    #mostrando a imagem crua e direta
    cv2.imshow('im1', frame)
    drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
    #pontos de interesse dos olhos
    eye_dots = [130, 145, 159, 243, 359, 374, 386, 463]
    with mp_face_mesh.FaceMesh(
      static_image_mode=True,
      max_num_faces=1,
      refine_landmarks=True,
      min_detection_confidence=0.5) as face_mesh:
      image = frame
      # Convert the BGR image to RGB before processing.
      results = face_mesh.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
      # Print and draw face mesh landmarks on the image.
      if not results.multi_face_landmarks:
        continue
      annotated_image = image.copy()
      for face_landmarks in results.multi_face_landmarks:
        i=0
        positionsx = {}
        positionsy = {}
        for landmarks in face_landmarks.landmark: 
          #marcando apenas os pontos de landmark dos olhos
          if i in eye_dots:
            shape = annotated_image.shape
            x = landmarks.x
            y = landmarks.y
            relative_x = int(x*shape[1])
            relative_y = int(y*shape[0])
            positionsx[str(i)] = relative_x
            positionsy[str(i)] = relative_y
            cv2.circle(annotated_image, (relative_x, relative_y), radius=1, color=(255,0,100), thickness=1)
          i+=1
        #calculando as distancias dos pontos (1-esquerda, 2-direita)
        distv1 = distance(positionsx["159"], positionsy["159"], positionsx["145"], positionsy["145"])
        disth1 = distance(positionsx["130"], positionsy["130"], positionsx["243"], positionsy["243"])
        distv2 = distance(positionsx["386"], positionsy["386"], positionsx["374"], positionsy["374"])
        disth2 = distance(positionsx["463"], positionsy["463"], positionsx["359"], positionsy["359"])
        faceTrack(annotated_image)
        #mostrando a imagem com a deteccao e os limites destacados
        cv2.imshow('im2', pisca(distv1, disth1, distv2, disth2, annotated_image))
        if cv2.waitKey(1) == ord('q'):
          break

  except KeyboardInterrupt:
    break
  
tellinho.land()
