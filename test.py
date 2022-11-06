import random
from queue import Queue

import numpy as np


class LaneQueue:
    def __init__(self, maxSize):
        self.maxSize = maxSize
        self.data = Queue(maxSize)
        self.coeff1 = Queue(maxSize) #for valid graph vector

    def put(self, val:np.poly1d):
        if self.data.qsize() == self.maxSize:
            self.data.get()
            self.data.put(val)
            self.coeff1.get()
            self.coeff1.put(val.coeffs[1])
        else:
            self.data.put(val)
            self.coeff1.put(val.coeffs[1])

    def getItems(self):
        return self.data.queue

    def get(self, i):
        return self.data.queue[i]

    def getFirst(self):
        return self.data.queue[0]

    def getLast(self):
        return self.data.queue[self.maxSize-1]

    def getValidLine(self):
        # vdata = np.median(self.coeff1.queue)
        # print(self.coeff1.queue)
        # print(vdata)
        # print(np.argsort(self.coeff1.queue)[len(self.coeff1.queue)//2])
        medianIdx = np.argsort(self.coeff1.queue)[len(self.coeff1.queue) // 2]
        return self.get(medianIdx)

    def getSize(self):
        return self.data.qsize()


l = LaneQueue(5)
for i in range(10):
    d = np.poly1d([random.randint(1,10), random.randint(1, 10), random.randint(1, 10)])
    print(d)
    l.put(d)

print('----------------------')
l.getValidLine()