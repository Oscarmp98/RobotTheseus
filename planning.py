from VforVision import *
from SControl import *
from map import *




def exploreNode(nid, movIn, movOut, visIn, visOut):
    global x, y, rot,prevn,path,ang_dic
    
    print('id: ',nid)
    print('prevn',prevn)
    N = None
    res = 0
    try:
        
        N = M.findNode(nid)
        print('node found')
        res = 1
    except:
        pass
    if res == 0:
        
        print('rot: ',rot)
        N = M.addNode(x,y,nid)
        
        
        for i in range(0,4):

            visIn.put('line') #Hace que solo detecte lineas
            visIn.put('cont') #Hace que continue con la detección
            res=visOut.get()
            if res[1]!='S': #Si ve una línea
                print('Arista en àngulo', rot)
                M.addEdge(N,None,rot,0)
            movIn.put((2, 90))
            updateCoords(movOut)

        print('N.edges',N.edges)
    
    if prevn != None:
        print('prev.edges',prevn[0].edges)
        M.delEdgeAng(prevn[0],prevn[1])    
        M.delEdgeAng(N,(prevn[1]-180)%360)
        M.addEdge(N,prevn[0],(prevn[1]-180)%360,prevn[1])
        print('N.edges',N.edges)
        
    nxt = None
    for i in N.edges:
        print('i: ',i)
        if i[0] == None:
            nxt = i
    print('nxt',nxt)
    
    if nxt != None:
        print(nxt[1]-rot)
        
        movIn.put((2,nxt[1]-rot)) # Optimizar angulos de giro
        updateCoords(movOut)
        movIn.put((1,5))
        updateCoords(movOut)
        
        prevn = [N,nxt[1]]
        print('prevn',prevn)
        path += [N]
    else:
        if path != []:
            for i in N.edges:
                if i[0] == path[-1]:
                    print('volver a: ',i)
                    movIn.put((2,i[1]-rot))
                    updateCoords(movOut)
                    movIn.put((1,5))
                    updateCoords(movOut)
                    prevn = None
                    del path[-1]
                    print(path)
                    break
        else:
            path = None
            print('end')
            return True
            

    visIn.put('all')
    return False

    


def explore(movIn, movOut, visIn, visOut):
    movOut.put(('M', 0))  # Para que updateCoords no se vuelva loco en la primera iteración
    ndetected = False #Se ha detectado algún nodo en algún momento
    last = -1
    prevn = None
    path = []
    while True:
        aux = visOut.get()

        # print(aux)
        if aux[0] == 'L' and not ndetected :  # Si la información es sobre la línea y no se detecta nodo
            follow(aux,last,movIn,movOut)
        elif aux[0] == 'N':
            
            print('Nodo detectado',aux[1])
            ndetected =True
            arr = aux[1]
            #print(arr[1][0])
            if abs(arr[1][0]) >= 9:
                print('Corrige')
                if abs(arr[1][0]) > 20:
                    movIn.put((2,np.sign(arr[1][0])*2))
                else:
                    movIn.put((2,np.sign(arr[1][0])*1))
                updateCoords(movOut)
            else:
                dst = (-arr[1][1]/50)*2 +15.9
                print('dst ',dst)
                movIn.put((1, dst))
                updateCoords(movOut)
                updateCoords(movOut)
                if exploreNode(arr[0],movIn, movOut, visIn, visOut):
                    break
                ndetected = False
                movOut.put(('R', 0))  # Los siguientes movimientos no tendrán que ser esperados
                
        printer()

        visIn.put('cont')

        if path == None:
            # Cerrar subthreads
            break

def follow(aux,last,movIn,movOut):
    if aux[1] == 'W' or aux[1] == 'S':
        if last != 1 or not movOut.empty():
            movIn.put((1, 15))
            updateCoords(movOut)
            last = 1
        elif aux[1] == 'L':
            if last != 2 or not movOut.empty():
                movIn.put((2, -15))
                updateCoords(movOut)
                last = 2
        elif aux[1] == 'R':
            if last != 3 or not movOut.empty():
                movIn.put((2, 15))
                updateCoords(movOut)
                last = 3


def printer():
    global qprint
    try:
        while True:
            print(qprint.get_nowait())
    except:
        pass

def updateCoords(movOut):
    global x, y, rot
    aux = movOut.get()
    if aux[0] == 'R':
        rot += aux[1]
        rot = rot % 360
        #print('Rot', rot)
    else:
        sin = np.sin(np.deg2rad(rot))
        cos = np.cos(np.deg2rad(rot))
        x += cos * aux[1]
        y += sin * aux[1]
        #print('X, Y:', x, y)


def core(movIn, movOut, visIn, visOut):
    global x, y, rot
    print('Core')
    while movOut.get() != 'ready':
        pass
    print('Movement ready')

    visIn.put('ready')
    explore(movIn, movOut, visIn, visOut)
    obey()
    visIn.put('stop')
    movIn.put((4, 0))



def threading():
    #
    # FALTA EL CÓDIGO DE SCONTROL
    #
    # Colas para comunicar threads
    movIn = Queue()
    movOut = Queue()
    visIn = Queue()
    visOut = Queue(maxsize = 1)
    # Creación de threads
    vth = th.Thread(target=Vision, args=(visIn, visOut,))
    vth.start()
    mth = th.Thread(target=movement, args=(movIn, movOut,))
    mth.start()
    # Core
    core(movIn, movOut, visIn, visOut)
    # Joins
    vth.join()
    mth.join()


def main():
    threading()


main()


# GPIO.cleanup()
