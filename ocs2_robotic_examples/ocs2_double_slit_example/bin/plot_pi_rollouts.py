#! /usr/bin/env python3

import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm # color map
import re
import math

sampleData = []

maxCost = -1.0

# read data from console output file
with open("console.log") as inFile:
    reader = csv.reader(inFile, delimiter='\t')
    for row in reader:
        if re.match(r'\+\+\+ Sample',row[0]) is not None:
            sampleData.append(np.array([]).reshape(0,3))
            continue
        if row[0] == "time":
            continue
        sampleData[-1] = np.vstack((sampleData[-1], np.array([float(row[0]), float(row[1]), float(row[2])])))
        cost = sampleData[-1][-1,1]
        if (not math.isinf(cost)) and (cost > maxCost):
            maxCost = cost


print(maxCost)


for data_i in sampleData:
    colors = np.arctan(data_i[:,1] / maxCost)*2.0/np.pi
    print(colors)
    plt.scatter(data_i[:,0], data_i[:,2], c=cm.hot(colors), edgecolor='none')
    plt.plot(data_i[:,0], data_i[:,2])

plt.title('darker dots = lower cost-to-go')
plt.show()
