import math


class Node:

    def __init__(self, nx, ny, nid=-1):
        self.edges = []
        self.x = float(nx)
        self.y = float(ny)
        self.id = nid

    def __str__(self):
        # Permite hacer un Print directo de la clase Node
        return 'Node ' + str(self.id) + str((self.x, self.y))

    def __repr__(self):
        # Permite hacer un Print indirecto de la clase Node
        return 'Node ' + str(self.id) + str((self.x, self.y))


def distance(A, B):
    return math.sqrt((A.x - B.x) ** 2 + (A.y - B.y) ** 2)

def line_angle(x1,y1,x2,y2):
    return math.atan2(y2-y1, x2-x1) / math.pi * 180
