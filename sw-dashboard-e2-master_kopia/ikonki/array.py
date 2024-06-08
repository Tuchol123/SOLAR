import sys, os
import matplotlib.pyplot as plt
import numpy as np

def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.2989, 0.5870, 0.1140])

im = plt.imread(sys.argv[1])
gray = rgb2gray(im)
# im = (255-im)
# gray = (255-gray)
gray = gray / np.amax(gray)
gray = gray * 255
name = os.path.splitext(os.path.basename(sys.argv[1]))[0]
first = True
pixels=0
min_level = 40

with open(os.path.splitext(sys.argv[1])[0]+'.txt', 'w') as f:
    f.write('iconA_t '+name+'IconA = {'+str(im.shape[1])+','+str(im.shape[0])+',{')
    for row in gray:
        for col in row:
            if first:
                val = int(col/16)
                first = False
            else:
                val = val + 16*int(col/16)
                f.write(str(val)+', ')
                first = True
            pixels = pixels + 1
            if pixels>79:
                f.write('\n')
                pixels = 0
    f.write('}};')
