#-*- coding: UTF-8 -*-
# Librerias
import RPi.GPIO as GPIO
import time
import numpy as np
import time

# Variables predefinidas
ratio = float(512)/float(360) #Ratio de ciclos por grado
radius = 4.36  #Radio de la rueda en cm
center = 13.1  #Distancia entre la rueda y el centro del robot en cm
pi = 3.1415926535
delay = 0.5

GPIO.setmode(GPIO.BOARD)

pins = [7,11,13,15,31,33,35,37]

for pin in pins:
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, 0)





move_seq = np.array([
  [1,0,0,1,1,0,0,1],
  [0,0,0,1,0,0,0,1],
  [0,0,1,1,0,0,1,1],
  [0,0,1,0,0,0,1,0],
  [0,1,1,0,0,1,1,0],
  [0,1,0,0,0,1,0,0],
  [1,1,0,0,1,1,0,0],
  [1,0,0,0,1,0,0,0]
])
rotate_seq = np.array([
  [1,0,0,1,1,0,0,0],
  [0,0,0,1,1,1,0,0],
  [0,0,1,1,0,1,0,0],
  [0,0,1,0,0,1,1,0],
  [0,1,1,0,0,0,1,0],
  [0,1,0,0,0,0,1,1],
  [1,1,0,0,0,0,0,1],
  [1,0,0,0,1,0,0,1]
])


# Funciones de movimiento

def move(dist):
  
  
  # Se calcula la direccion
  dir = True
  if (dist < 0):
    dist = abs(dist)
    dir = False

  # Se calculan los ciclos que deben girar los motores
  deg = (dist/radius)*(180/pi)
  cycles = deg * ratio
  cycles = int(cycles)
  print("moving "+str(dist)+" cm")

  # Se calcula la matriz de movimiento en funcion de la direccion
  seq = move_seq
  if(dir == False):
    seq = np.flip(seq,0)

  # Se manda la matriz de movimiento por los pines de la GPIO
  for i in range(cycles):
    for halfstep in range(8):
      for pin in range(8):
        GPIO.output(pins[pin], int(seq[halfstep][pin]))
      time.sleep(0.001)
  time.sleep(delay)

def rotate(rot):
  # Se calcula la direccion
  dir = True
  if (rot < 0):
    rot = abs(rot)
    dir = False

  # Se calcula el numero de ciclos que deben girar los motores
  deg = ((rot*center)/radius)/2
  cycles = deg * ratio
  cycles = int(cycles)
  print("rotating " + str(rot) + " degrees")

  # Se calcula la matriz de rotacion en funcion de la rotacion
  seq = rotate_seq.copy()
  if (dir == False):
    seq = np.flip(seq, 0)

  # Se manda la matriz de rotacion por los pines de la GPIO
  for i in range(cycles):
    for halfstep in range(8):
      for pin in range(8):
        GPIO.output(pins[pin], int(seq[halfstep][pin]))
      time.sleep(0.001)
      
  time.sleep(delay)

def calibration():

  M0 = time.time()
  move(1)
  M1 = time.time()
  move(-1)
  R0 = time.time()
  rotate(1)
  R1 = time.time()
  rotate(-1)
  TM = M1-M0
  TR = R1-R0
  print('tiempo de giro (1deg): '+str(TR) +' seg')
  print('tiempo de movimiento(1cm): '+str(TM)+' seg')
  return TR,TM,delay

def main():
  
  #calibration()
  rotate(360)
  rotate(360)
  
  
main()
# Poner al final de la ejecución
GPIO.cleanup()
  
  
  

