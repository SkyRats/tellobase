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

# tellinho.send_keepalive()
# tellinho.send_control_command("command")

tellinho.takeoff()

tellinho.move_up(80)
oe = 0
od = 0

#funcoes utilizadas
def distance(x1, y1, x0, y0):
  return sqrt((x1-x0)**2 + (y1-y0)**2)

def pisca(v1, d1, v2, d2, img):
    global piscando

    olhoDireito = v1 / d1
    olhoEsquerdo = v2 / d2
    print('AAAAAAAAAAAAAA', olhoDireito, olhoEsquerdo)
    if olhoDireito < 0.08 or olhoEsquerdo < 0.08:
        # Piscando os dois olhos
        if olhoDireito < 0.08 and olhoEsquerdo < 0.08 and piscando == False:
            piscando = True
            print('PISCOU')
            if(tellinho.get_battery()>50):
              tellinho.flip('b')
              
            # img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
        # if olhoDireito >= 0.12 and olhoEsquerdo >= 0.12 and piscando == True:
        #     piscando = False
        #     #img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        
        # Piscando o olho direito
        if olhoDireito < 0.08 and olhoEsquerdo >= 0.12 and piscando == False:
            piscando = True
            print('Piscou Direito ;)')
            if(tellinho.get_battery()>50):
              tellinho.flip('l')
              tellinho.move_right(40)
            # img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            
        # elif olhoDireito >= 0.12 and olhoEsquerdo >= 0.12 and piscando == True:
        #     piscando = False

        # Piscando o olho esquerdo
        if olhoEsquerdo < 0.08 and olhoDireito >= 0.12 and piscando == False:
            piscando = True
            print('Piscou Esquerdo ;)')
            if(tellinho.get_battery()>50):
              tellinho.flip('r')
              tellinho.move_left(40)

            # img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)


    #mudei aqui
    elif olhoEsquerdo >= 0.4 and olhoDireito >= 0.4 and piscando == False: 
        tellinho.land()

    else:
        piscando = False
        # print('Nao piscou :(')

    return img

def faceTrack(img):
  height, width, _ = img.shape
  #acha centro da imagem
  Xm = round(width/2)
  Ym = round(height/2)
  cv2.circle(img, (Xm, Ym), radius=1, color=(255, 0, 0), thickness=5)
  cv2.rectangle(img, (Xm-80, Ym+80), (Xm+80, Ym-80), color=(0, 0, 255), thickness=5)
  #acha o centro do rosto
  Xrosto = round((positionsx["243"]+positionsx["463"])/2)
  Yrosto = positionsy["243"]
  cv2.circle(img, (Xrosto, Yrosto), radius=1, color=(255, 0, 0), thickness=5)
  #checa se o rosto esta no centro
  if Xm-50<=Xrosto<=Xm+50 and Ym-50<=Yrosto<=Ym+50:
    print("Rosto centralizado")
  else:
    if Ym-Yrosto>100:
      tellinho.move_up(20)
    elif Ym-Yrosto<-100:
       tellinho.move_down(20)
    elif(Xm-Xrosto<-100):
      tellinho.rotate_clockwise(15) 
    elif (Xm-Xrosto>100):
      tellinho.rotate_counter_clockwise(15)       




#loop principal 
while (cv2.waitKey(10) != ord(" ")):
  try:
    # tellinho.send_keepalive()
    print("----Bateria:", tellinho.get_battery())
    frame = tellinho.get_frame_read().frame
    cv2.imshow('aba1', frame)
    drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
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
        print("AAAAAA", distv1/disth1, distv2/disth2)
        # pisca(distv1, disth1, distv2, disth2, annotated_image)
        faceTrack(annotated_image)
        # cv2.imshow('Aba', annotated_image)
        cv2.imshow('Aba', pisca(distv1, disth1, distv2, disth2, annotated_image))
        if cv2.waitKey(1) == ord('q'):
          break

  except KeyboardInterrupt:
    break
  
tellinho.land()