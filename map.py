from node import *
import math
from Algorithm import *



class map:

    def __init__(self):
        self.nodes = []
        self.nodedic = {}
        self.tolerance = 5

    def addNode(self, nx, ny, nid=-1):

        # Comprobar que no hay otro nodo en este punto
        over = self.Coord2Node(nx, ny)
        if over != None:
            raise Exception(
                'Position of Node {} already in use by Node {}{}'.format(nid, over.id, (over.x, over.y)))
        del over

        # Buscar si ya hay otro nodo con su id y si eso no ocurre añadirlo
        blank = nid == -1
        if blank or nid not in self.nodedic:
            N = Node(nx, ny, nid)
            self.nodes += [N]
            if not blank:
                self.nodedic[nid] = N
            return N
        else:
            raise Exception('Node ID ({}) already in use'.format(nid))

    def delNode(self, N):
        # Borra el nodo que entre por referencia
        if N.id != -1:
            del self.nodedic[N.id]
        self.nodes.remove(N)

    def addEdge(self, N, N2,a1,a2):
        N.edges += [[N2,a1]]
        if N2 != None:
            N2.edges += [[N,a2]]

    def delEdgeAng(self, N, a):
        found = 0
        for i in N.edges:
            if i[1] == a:
                N.edges.remove(i)
                found += 1
                break
        if found == 0:
            raise Exception('No edge with this angle')


    def delEdge(self, N, N2):
        found=0
        for i in N.edges:
            if i[0] == N2:
                N.edges.remove(i)
                found+=1
                break

        for i in N2.edges:
            if i[0] == N:
                N2.edges.remove(i)
                found+=1
                break

        if found < 2:
            raise Exception(
                'No existing edge between Nodes {}{} and {}{}'.format(N.id, (N.x, N.y), N2.id, (N2.x, N2.y)))



    def Coord2Node(self, nx, ny):
        # Busca un nodo por coordenadas con un margen de error
        ret = None
        for N in self.nodes:
            if math.sqrt((N.x - nx) ** 2 + (N.y - ny) ** 2) <= self.tolerance:
                ret = N
                break

        return ret

    def findNode(self, nid):
        # Busca un nodo por ID
        try:
            return self.nodedic[nid]
        except:
            raise Exception('No Node with ID = {}'.format(nid))

    def recenter(self, mx, my, angle):
        # Modifica las coordenadas de los nodos de forma que el laberinto no cambia, solo se modifican los ejes
        pass



def obey():
    global M, x,y,rot
    print(M.nodedic)
    #print('Modificar Origen para coger de las coordenadas del robot')
    while True:
        or_id = M.Coord2Node(x,y)
        des_id=input('Destino: ')
        if int(des_id) < 0:
            break
        route=Astar2(M.findNode(int(or_id)), M.findNode(int(des_id)))
        print(route)
        updateCoords(movOut) # para asegurarse de que acaban las ordenes
        for i in range(0,len(route)-1):
            for n in route[i].edges:
                if n[0] == route[i+1]:
                    print(n[1])
                    movIn.put((2, n[1]-rot))
                    updateCoords(movOut)
                    #Hacer que rote justo aquí
                    break

            #Loop de ir en línea recta hasta detectar nodo
            movOut.put(('M', 0))
            ndetected = False #Se ha detectado algún nodo en algún momento
            last = -1
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
                        ndetected = False
                        break
                        
                printer()

                visIn.put('cont')



# Variables globales
x = y = rot = 0
M = map()
prevn = None
path = []



# M=map()
# N1=M.addNode(0,0,1)
# N2=M.addNode(0,10,2)
# N3=M.addNode(10,10,3)
# N4=M.addNode(10,0,4)
#
# M.addEdge(N1,N2,90,0)
# M.addEdge(N1,N4,0,0)
# M.addEdge(N3,N2,0,180)
# M.addEdge(N4,N3,0,0)
#
# obey()
