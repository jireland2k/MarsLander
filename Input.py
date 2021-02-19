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
a1 = 0.015
b1 = 0.025
c1 = 11
a2 = 0.600
b2 = 2.000
c2 = 15
a3 = 0.002
b3 = 0.004
c3 = 11
a4 = 0.100
b4 = 0.500
c4 = 9

ax1 = 0.005
bx1 = 0.050
cx1 = 10
ax2 = 0.200
bx2 = 0.600
cx2 = 9
ax3 = 0.002
bx3 = 0.014
cx3 = 13
ax4 = 0.050
bx4 = 0.500
cx4 = 10