from djitellopy import Tello
import time

# Conectar ao Tello
tello = Tello()
tello.connect()

pousou = False

bateria = tello.get_battery()
print("BATERIA:", bateria)

tello.takeoff()
time.sleep(3)

altura_referencia = tello.get_distance_tof()

def verificar_mudanca_de_altura():
    altura_atual = tello.get_distance_tof()
    if abs(altura_atual - altura_referencia) > 60:
        global pousou
        tello.emergency()
        print("Base detectada abaixo! Desligando motores...")
        pousou = True                                                  #duração ciclo do 8

while not pousou:
    verificar_mudanca_de_altura()