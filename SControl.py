#-*- coding: UTF-8 -*-
import threading as th
import time
from queue import Queue
import numpy as np
import RPi.GPIO as GPIO
import threading as th
GPIO.cleanup()
qprint = Queue()
mod_pos = th.Lock()
# Variables predefinidas
ratio = float(512)/float(360) #Ratio de ciclos por grado
invratio = float(360)/float(512)
radius = 4.36  #Radio de la rueda en cm
center = 13.1  #Distancia entre la rueda y el centro del robot en cm
pins = [7,11,13,15,31,33,35,37]
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


# Configurar los pines
GPIO.setmode(GPIO.BOARD)
for pin in pins:
    
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, 0)

# Funciones de movimiento
def calibration(qin,qout):
    M0 = time.time()
    move(1,qin,qout)
    qout.get()
    M1 = time.time()
    move(-1,qin,qout)
    qout.get()
    R0 = time.time()
    rotate(1,qin,qout)
    qout.get()
    R1 = time.time()
    rotate(-1,qin,qout)
    qout.get()
    TM = M1 - M0
    TR = R1 - R0
    print('tiempo de giro (1deg): ' + str(TR) + ' seg')
    print('tiempo de movimiento(1cm): ' + str(TM) + ' seg')
    return TR, TM

def move(dist,qin,qout):
    # Se calcula la direccion
    
    dir = True
    if (dist < 0):
        dist = abs(dist)
        dir = False

    # Se calculan los ciclos que deben girar los motores
    deg = (dist / radius) * (180 / np.pi)
    cycles = deg * ratio
    cycles = int(cycles)
    qprint.put("moving " + str(dir) +' '+ str(dist) + " cm")

    # Se calcula la matriz de movimiento en funcion de la direccion
    seq = move_seq
    if (dir == False):
        seq = np.flip(seq, 0)

    # Se manda la matriz de movimiento por los pines de la GPIO
    travel = 0
    const = invratio*(np.pi/180)*radius
    for i in range(cycles):
        for halfstep in range(8):
            for pin in range(8):
                GPIO.output(pins[pin], int(seq[halfstep][pin]))
            
            if not qin.empty():
                dist = i*const
                qout.put(('M',dist))
                return
            time.sleep(0.001)
        #mod_pos.acquire()
        #travel += const
        #mod_pos.release()
    qout.put(('M',dist))

def rotate(rot,qin,qout):
    # Se calcula la direccion
    dir = True
    if (rot < 0):
        rot = abs(rot)
        dir = False

    # Se calcula el numero de ciclos que deben girar los motores
    deg = ((rot * center) / radius) / 2
    cycles = deg * ratio
    cycles = int(cycles)
    qprint.put("rotating " + str(dir) +' ' + str(rot) + " degrees")

    # Se calcula la matriz de rotacion en funcion de la rotacion
    seq = rotate_seq
    if (dir == False):
        seq = np.flip(seq, 0)

    # Se manda la matriz de rotacion por los pines de la GPIO
    for i in range(cycles):
        for halfstep in range(8):
            for pin in range(8):
                GPIO.output(pins[pin], int(seq[halfstep][pin]))
            if not qin.empty():
                rot = (2*(i*invratio)*radius)/center
                qout.put(('R',rot))
                return
            time.sleep(0.001)
    qout.put(('R',rot))

# Funciones de gestión
def wait(duration,qin, qout,):
    while True:
        if qin.empty():
            break

        if duration > 0.05:
            time.sleep(0.05)
            duration -= 0.05
        else:
            time.sleep(duration)
            qout.put(True)
            break

def movement(qin, qout):
    calibration(qin,qout)
    qout.put('ready')
    prev = (0, 0)
    while True:
        aux = qin.get()
        if aux[0] == 1:  # 1 = Move
            if prev[0] == 2:
                time.sleep(0.2)

            move(aux[1],qin, qout)

        elif aux[0] == 2:  # 2 = Rotate
            if prev[0] == 1:
                time.sleep(0.2)

            rotate(aux[1],qin, qout)

        elif aux[0] == 3:  # 3 = Wait
            wait(aux[1],qin, qout)

        else:
            break  # 4 = Break
        prev = aux



