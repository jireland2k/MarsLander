import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import pprint
import itertools
import csv
import json
import time

from numpy.random import randint
from numpy.linalg import norm
from ipywidgets import interactive
from matplotlib import rcParams

rcParams['figure.figsize'] = (10, 8)

g = 3.711  #  m/s^2, gravity on Mars
power2thrust = 1000
dt = 0.1
parameters = {
        'K_h': 0,
        'K_p': 0,
        'K_i': 0,
        'K_d': 0,
        'K_diffx': 0,
        'K_px': 0,
        'K_ix': 0,
        'K_dx': 0
    }


def mars_surface():
    surfaceN = randint(5, 15)
    land = np.zeros((surfaceN, 2), dtype=int)

    # first ensure there's a flat landing site at least 1000m long
    landing_site = randint(1, surfaceN-1)
    land[landing_site, 0] = randint(2000, 5000)
    land[landing_site+1, 0] = min(land[landing_site, 0] +
                                  randint(1000, 2000), 6999)
    land[landing_site+1, 1] = land[landing_site, 1] = randint(1, 1500)

    # fill in the rest of the terrain
    for i in range(landing_site):
        land[i, 0] = (land[landing_site, 0] / landing_site) * i
        land[i, 1] = randint(0, 1500)

    for i in range(landing_site + 2, surfaceN):
        land[i, 0] = (land[landing_site + 1, 0] +
                      (7000 - land[landing_site + 1, 0]) / len(land[landing_site + 2:]) *
                      (i - (landing_site + 1)))
        land[i, 1] = randint(0, 1500)

    # impose boundary conditions
    land[0, 0] = 0
    land[-1, 0] = 6999

    return land, landing_site


def plot_surface(land, landing_site):
    fig, ax = plt.subplots()
    ax.plot(land[:landing_site+1, 0], land[:landing_site+1, 1], 'k-')
    ax.plot(land[landing_site+1:, 0], land[landing_site+1:, 1], 'k-')
    ax.plot([land[landing_site, 0], land[landing_site+1, 0]],
            [land[landing_site, 1], land[landing_site+1, 1]], 'k--')
    ax.set_xlim(0, 7000)
    ax.set_ylim(0, 3000)
    return ax


def plot_lander(land, landing_site, X, thrust=None, animate=False, step=10):
    if animate:
        def plot_frame(n=len(X)-1):
            ax = plot_surface(land, landing_site)
            ax.plot(X[:n, 0], X[:n, 1], 'b--')
            ax.plot(X[n, 0], X[n, 1], 'b^', ms=20)
            if thrust is not None:
                ax.plot([X[n, 0], X[n, 0] - 100/power2thrust*thrust[n, 0]],
                        [X[n, 1] - 100., X[n, 1] - 100. -
                            100/power2thrust*thrust[n, 1]],
                        'r-', lw=10)
        return interactive(plot_frame, n=(0, len(X), step))
    else:
        ax = plot_surface(land, landing_site)
        ax.plot(X[:, 0], X[:, 1], 'b--')
        ax.plot(X[-1, 0], X[-1, 1], 'b^')
        return ax


np.random.seed(42)  # seed random number generator for reproducible results
land, landing_site = mars_surface()
# plot_surface(land, landing_site)


def interpolate_surface(land, x):
    i,  = np.argwhere(land[:, 0] < x)[-1]  # segment containing x is [i, i+1]
    m = (land[i+1, 1] - land[i, 1])/(land[i+1, 0] - land[i, 0])  # gradient
    x1, y1 = land[i, :]  # point on line with eqn. y - y1 = m(x - x1)
    return m*(x - x1) + y1


def height(land, X):
    return X[1] - interpolate_surface(land, X[0])

def horizontal_diff(land, X):
    landtarget = ((land[landing_site+1, 0] + land[landing_site, 0]) // 2)
    return X[0] - landtarget


def simulate(X0, V0, land, landing_site,
             fuel=200, dt=0.1, Nstep=1000,
             autopilot=None, print_interval=100, parameters=None):

    n = len(X0)       # number of degrees of freedom (2 here)
    X = X0.copy()     # current position
    V = V0.copy()     # current velocity
    A = A0.copy()     # current acceleration

    Xs = np.zeros((Nstep, n))  # position history (trajectory)
    Vs = np.zeros((Nstep, n))  # velocity history
    As = np.zeros((Nstep, n))  # acceleration history
    fuels = np.zeros(Nstep)    # fuel history
    errory = np.zeros(Nstep)
    errorx = np.zeros(Nstep)

    thrust = np.zeros((Nstep, n))  # thrust history
    success = False
    fuel_warning_printed = False
    rotate = 0  #  degrees, initial angle
    power = 0            # m/s^2, initial thrust power
    mass = 1000

    for i in range(Nstep):
        Xs[i, :] = X     # Store positions
        Vs[i, :] = V     # Store velocities
        As[i, :] = A     # Store accelerations
        fuels[i] = fuel  # Store fuel

        if autopilot is not None:
            # call user-supplied function to set `rotate` and `power`
            rotate, power, ey, ex = autopilot(
                i, X, V, fuel, rotate, power, errory, errorx, parameters)
            assert abs(rotate) <= 45
            assert 0 <= power <= 4

            rotate_rad = rotate * np.pi / 180.0  # degrees to radians
            thrust[i, :] = power2thrust * power * np.array([np.sin(rotate_rad),
                                                            np.cos(rotate_rad)])

            if fuel <= 0:
                if not fuel_warning_printed:
                    # print("Fuel empty! Setting thrust to zero")
                    fuel_warning_printed = True
                thrust[i, :] = 0
                fuel = 0
            else:
                fuel -= power * dt
                mass -= power * dt

        # force to acceleration
        Cd = 1.17  # assume drag coefficient of hemisphere with flat side pointed in velocity direction
        atmos_density = 0.020  # in kg/m ^ 3
        lander_area = 4.0  # in m^2
        # D = Drag force: drag coefficient * ((atmospheric density * velocity^2)/2) * lander area
        Dh = - Cd * ((atmos_density * (((wind - V[0])**2) * np.sign(V[0])))/2) * lander_area / mass
        Dv = - Cd * ((atmos_density * (((V[1])**2) * np.sign(V[1]))/2)) * lander_area / mass
        A = np.array([0+Dh, -g+Dv]) + thrust[i, :] / mass  # acceleration
        V += A * dt  # update velocities
        X += V * dt  # update positions

        # if i % print_interval == 0:
        #     print(f"i={i:03d} X=[{X[0]:8.3f} {X[1]:8.3f}] V=[{V[0]:8.3f} {V[1]:8.3f}]"
        #           f" thrust=[{thrust[i, 0]:8.3f} {thrust[i, 1]:8.3f}] fuel={fuel:8.3f} mass={mass:8.3f}")

        # check for safe or crash landing
        if X[1] < interpolate_surface(land, X[0]):
            if not (land[landing_site, 0] <= X[0] and X[0] <= land[landing_site + 1, 0]):
                # print("Crash! did not land on flat ground!")
                pass
            elif abs(rotate) > 5:  # Degrees
                # print(
                #     "Crash! did not land in a vertical position (tilt angle < 5 degrees)")
                pass
            elif abs(V[1]) >= 5:
                # print(
                #     "Crash! vertical speed must be limited (<5m/s in absolute value), got ", abs(V[1]))
                pass
            elif abs(V[0]) >= 2:
                # print(
                #     "Crash! horizontal speed must be limited (<2m/s in absolute value), got ", abs(V[0]))
                pass
            else:
                # print("Safe landing - Well done!")
                success = True
            Nstep = i
            break

    return (Xs[:Nstep, :], Vs[:Nstep, :], As[:Nstep, :],
            thrust[:Nstep, :], fuels[:Nstep], errory[:Nstep], errorx[:Nstep], success)


def proportional_autopilot(i, X, V, fuel, rotate, power, errory, errorx, parameters):
    c = 0.0  # target landing speed, m/s
    K_h = parameters['K_h']
    K_p = parameters['K_p']
    K_diffx = parameters['K_diffx']
    K_px = parameters['K_px']

    h = height(land, X)
    diffx = horizontal_diff(land, X)

    ey = - (c + K_h*h + V[1])
    ex = - (c +K_diffx*diffx + V[0])
    errory[i] = ey
    errorx[i] = ex

    Pouty = K_p*ey
    Poutx = K_px*ex

    if Pouty > 0:
        power = min(max(np.sqrt(Pouty**2+Poutx**2), 0.0), 4.0)
        angle = min(45,max(np.arctan2(Poutx, Pouty)*(180/np.pi),-45))
        rotate = angle
    else:
        power = 0
        rotate = 0
    
    return (rotate, power, ey, ex)


def pi_autopilot(i, X, V, fuel, rotate, power, errory, errorx, parameters):
    c = 0.0  # target landing speed, m/s
    K_h = parameters['K_h']
    K_p = parameters['K_p']
    K_i = parameters['K_i']
    K_diffx = parameters['K_diffx']
    K_px = parameters['K_px']
    K_ix = parameters['K_ix']

    h = height(land, X)
    diffx = horizontal_diff(land, X)

    ey = - (c + K_h*h + V[1])
    ex = - (c +K_diffx*diffx + V[0])
    errory[i] = ey
    errorx[i] = ex
    integral_ey = np.sum(errory[:i]) * dt
    integral_ex = np.sum(errorx[:i]) * dt
    
    Pouty = K_p*ey + K_i*integral_ey
    Poutx = K_px*ex + K_ix*integral_ex

    if Pouty > 0:
        power = min(max(np.sqrt(Pouty**2+Poutx**2), 0.0), 4.0)
        angle = min(45,max(np.arctan2(Poutx, Pouty)*(180/np.pi),-45))
        rotate = angle
    else:
        power = 0
        rotate = 0
    
    return (rotate, power, ey, ex)


def pid_autopilot(i, X, V, fuel, rotate, power, errory, errorx, parameters):
    c = 0.0  # target landing speed, m/s
    K_h = parameters['K_h']
    K_p = parameters['K_p']
    K_i = parameters['K_i']
    K_d = parameters['K_d']
    K_diffx = parameters['K_diffx']
    K_px = parameters['K_px']
    K_ix = parameters['K_ix']
    K_dx = parameters['K_dx']

    h = height(land, X)
    diffx = horizontal_diff(land, X)

    ey = - (c + K_h*h + V[1])
    ex = - (c +K_diffx*diffx + V[0])
    errory[i] = ey
    errorx[i] = ex
    integral_ey = np.sum(errory[:i]) * dt
    integral_ex = np.sum(errorx[:i]) * dt
    diff_ey = (errory[i] - errory[i-1]) / dt
    diff_ex = (errorx[i] - errorx[i-1]) / dt

    Pouty = K_p*ey + K_i*integral_ey + K_d*diff_ey
    Poutx = K_px*ex + K_ix*integral_ex + K_dx*diff_ex

    if Pouty > 0:
        power = min(max(np.sqrt(Pouty**2+Poutx**2), 0.0), 4.0)
        angle = min(45,max(np.arctan2(Poutx, Pouty)*(180/np.pi),-45))
        rotate = angle
    else:
        power = 0
        rotate = 0
    
    return (rotate, power, ey, ex)


# Automated Testing Score Function
def score(result):
    Xs, Vs, As, thrust, fuels, errory, errorx, success = result
    fuel_use_bias = 0.005
    return np.sqrt(Vs[-1][1]**2) + (fuel_use_bias * (((500-fuels[-1]))))


# Initial conditions
X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
V0 = [0., 0., ]
A0 = [0., -g]
# wind = np.random(0., 70)
wind = +70