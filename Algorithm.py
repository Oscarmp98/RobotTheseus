
from node import *
from queue import PriorityQueue






def Astar2(Origin, Destiny):
    paths = PriorityQueue()
    paths.put((0, 0, Origin, []))
    visited = {}
    res = []
    while True:
        if paths.empty():
            break
        elem = paths.get()
        dist = elem[1]
        node = elem[2]
        prev = elem[3]

        if node == Destiny:
            res = prev + [node]
            break

        for edge in node.edges:
            if edge[0] in visited:
                continue
            d = dist + distance(node, edge[0])
            h = distance(edge[0], Destiny) + d
            paths.put((h, d, edge[0], prev + [node]))
            visited[edge[0]] = True

    return res



