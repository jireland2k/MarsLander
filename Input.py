import numpy as np

OneDTest = False
if OneDTest == True:
    #init conditions
    wind = 0
    hoffset = 0
    VXinit = 0
    VYinit = -20
else:
    wind = 31
    hoffset = 500
    VXinit = 10
    VYinit = -20

#search inputs
a1 = 0.005
b1 = 0.050
c1 = 10
a2 = 0.200
b2 = 2.000
c2 = 10
a3 = 0.002
b3 = 0.020
c3 = 10
a4 = 0.050
b4 = 0.500
c4 = 10