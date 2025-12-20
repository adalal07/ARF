from djitellopy import Tello

Tello.TELLO_IP = '192.168.10.1'
Tello.RESPONSE_TIMEOUT = 1

print('Tello: Connecting to drone')
tello = Tello()
tello.connect(True)
print('Tello: Connected to drone')

print('Tello: Getting battery')
battery = tello.get_battery()
print(f'Battery: {battery}%')