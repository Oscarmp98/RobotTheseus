import threading as th
import time
from queue import Queue
from img_persp import *
from shapes import *


def clearQ(q):
    try:
        while not q.empty():
            q.get_nowait()
    except:
        return


def lineas(qin, qout):
    aux = ''
    while True:
        img = qin.get()
        rayas(img, qin, qout)

        aux = qin.get()

        if aux == 'cont':
            pass
        elif aux == 'stop':
            break


def nodos(qin, qout):
    aux = ''
    while True:
        
        aux = qin.get()

        if type(aux) is np.ndarray:
            res = detNode(aux, qin, qout)
            qout.put(res)
            #print('nodos',res)
        elif aux == 'stop':
            break


def coreVision(qin, qout, LIn, LOut, NIn, NOut):
    print('corevisions')
    action = 'L'
    mode = 'A' #mode all
    while True:
        img = getPhoto()
        img = cv2.GaussianBlur(img,(5,5),0)
        LIn.put(img)
        if NIn.empty() and mode!= 'L':
            NIn.put(img)
        dline = LOut.get()

        try:
            dnode = NOut.get_nowait()
            #print('dnode',dnode)
        except:
            dnode = -1

        if dnode == -1 or mode =='L':  # Si no se detectan Nodos...
            qout.put(['L', dline])
        else:
            if dnode[0] == -1:  # Si no ha encontrado nodo
                qout.put(['L', dline])
            else:
                qout.put(['N', dnode])

        while True:
            aux = qin.get()
            if aux == 'cont':
                LIn.put('cont')
                break
            elif aux == 'stop':
                LIn.put('stop')
                NIn.put('stop')
                print('finaliza')
                return
            elif aux =='line':
                mode = 'L'
            elif aux =='all':
                mode = 'A'


def Vision(qin, qout):
    # Wait for core
    while qin.get() != 'ready':
        pass

    LIn = Queue()
    LOut = Queue(maxsize = 1)
    NIn = Queue()
    NOut = Queue()
    # BIn = Queue()
    # BOut = Queue()

    # Crear thread para el seguidor de líneas
    thread1 = th.Thread(target=lineas, args=(LIn, LOut,))
    thread1.start()
    # Crear thread para el detector de nodos
    thread2 = th.Thread(target=nodos, args=(NIn, NOut,))
    thread2.start()

    coreVision(qin, qout, LIn, LOut, NIn, NOut)
    thread1.join()
    thread2.join()


def getPhoto():
    with picamera.PiCamera() as camera:
        camera.rotation = 270
        with picamera.array.PiRGBArray(camera) as stream:
            camera.capture(stream, format='bgr')
            # At this point the image is available as stream.array
            return stream.array
    #return cv2.imread('lineaHetero.png')
