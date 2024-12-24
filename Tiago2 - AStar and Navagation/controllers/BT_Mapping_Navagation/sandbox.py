import numpy as np
from matplotlib import pyplot as plt
def world2map(xw, yw):
    return max(min(int((xw + 2.15)/3.6 * 300), 299), 0) , max(min(int((1.7 - yw)/5.55 * 460), 459), 0)

def map2world(px, py):
    xw = px / (300.0/3.6) - 2.15
    yw = 1.7 - py / (460.0/5.55)
    
    return [xw, yw]

x, y = world2map(0, 0)
print(map2world(x, y))