from djitellopy import Tello

tello = Tello()
tello.connect()

print("Battery level:", tello.get_battery())

tello.end()