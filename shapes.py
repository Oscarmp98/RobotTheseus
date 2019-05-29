from functions import *
import time


def detNode(img,qin,qout):
    #start = time.time()






    thresh = blackThresh(img)


    mask = getCircles(thresh)
    # Show Original


    # Obtener contenido Círculo
    content = getContent(img, mask)
    show(content, 'content', True)
    thresh = greenThresh(content)
    show(thresh, 'second', True)


    nodes = getRectVec(thresh)

    if nodes:
        closest, position = nodeSelect(nodes, img.shape)
        homo = getHomo(img, closest)

        value = codeValue(homo[32:-32, 32:-32])
        
        

        # Show Code
        #print("Code Value: ", value)
        #Time
        end = time.time()
        #print('Time: ', end - start)
        return value,position[0]

    else:
        #print("No node found")
        #Time
        end = time.time()
        #print('Time: ', end - start)
        return -1,()


