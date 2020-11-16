import numpy as np
import matplotlib.pyplot as plt
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
# TODO: Run current code before meeting to showcase testing


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


def simulate(X0, V0, A0, land, landing_site,
             fuel=200, dt=0.1, Nstep=1000,
             autopilot=None, print_interval=100, parameters=None):

    n = len(X0)       # number of degrees of freedom (2 here)
    X = X0.copy()     # current position
    V = V0.copy()     # current velocity
    A = A0.copy()     # current acceleration
    Xs = np.zeros((Nstep, n))  # position history (trajectory)
    Vs = np.zeros((Nstep, n))  # velocity history
    As = np.zeros((Nstep, n))  # acceleration history
    fuels = np.zeros(Nstep)

    thrust = np.zeros((Nstep, n))  # thrust history
    success = False
    fuel_warning_printed = False
    rotate = 0  #  degrees, initial angle
    power = 0            # m/s^2, initial thrust power
    mass = 1000

    for i in range(Nstep):
        Xs[i, :] = X     # Store positions
        Vs[i, :] = V     # Store velocities
        As[i, :] = A    # Store accelerations
        fuels[i] = fuel  # Store fuel

        if autopilot is not None:
            # call user-supplied function to set `rotate` and `power`
            rotate, power = autopilot(i, X, V, fuel, rotate, power, parameters)
            assert abs(rotate) <= 90
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
# TODO: Showcase implementation of drag
        Cd = 1.17  # assume drag coefficient of hemisphere with flat side pointed in velocity direction
        atmos_density = 0.020  # in kg/m ^ 3
        lander_area = 4.0  # in m^2
        # D = Drag force: drag coefficient * ((atmospheric density * velocity^2)/2) * lander area
        Dh = - Cd*((atmos_density * (V[0]*np.abs(V[0])))/2)*lander_area/mass
        Dv = - Cd*((atmos_density * (V[1]*np.abs(V[1])))/2)*lander_area/mass
        A = np.array([0+Dh, -g+Dv]) + thrust[i, :] / mass  # acceleration
        V += A * dt                                  # update velocities
        X += V * dt                                  # update positions

        # if i % print_interval == 0:
        #     print(f"i={i:03d} X=[{X[0]:8.3f} {X[1]:8.3f}] V=[{V[0]:8.3f} {V[1]:8.3f}]"
        #           f" thrust=[{thrust[i, 0]:8.3f} {thrust[i, 1]:8.3f}] fuel={fuel:8.3f} mass={mass:8.3f}")

        # check for safe or crash landing
        if X[1] < interpolate_surface(land, X[0]):
            if not (land[landing_site, 0] <= X[0] and X[0] <= land[landing_site + 1, 0]):
                # print("Crash! did not land on flat ground!")
                pass
            elif abs(rotate) > 0.087:  # radians
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
            thrust[:Nstep, :], fuels[:Nstep], success)


def proportional_autopilot(i, X, V, fuel, rotate, power, parameters):
    c = 0.0  # target landing speed, m/s
    K_h = parameters['K_h']
    K_p = parameters['K_p']
    h = height(land, X)
    e = - (c + K_h*h + V[1])
    Pout = K_p*e
    power = min(max(Pout, 0.0), 4.0)
    # if i % 100 == 0:
    # print(f'e={e:8.3f} Pout={Pout:8.3f} power={power:8.3f}')
    return (rotate, power)


def pi_autopilot(i, X, V, fuel, rotate, power, parameters):
    c = 0.0  # target landing speed, m/s
    K_h = parameters['K_h']
    K_p = parameters['K_p']
    K_i = parameters['K_i']
    h = height(land, X)
    e = - (c + K_h*h + V[1])
    integral_e = 0
    integral_e += e * dt
    Pout = K_p*e + K_i*integral_e
    power = min(max(Pout, 0.0), 4.0)
    # if i % 100 == 0:
    # print(f'e={e:8.3f} Pout={Pout:8.3f} power={power:8.3f}')
    return (rotate, power)

# TODO: Check if correct implementation of PID controller


def pid_autopilot(i, X, V, fuel, rotate, power, parameters):
    c = 0.0  # target landing speed, m/s
    e = 0
    e_last = 0
    K_h = parameters['K_h']
    K_p = parameters['K_p']
    K_i = parameters['K_i']
    K_d = parameters['K_d']
    h = height(land, X)
    e = - (c + K_h*h + V[1])
    integral_e = 0
    integral_e += e * dt
    diff_e = 0
    diff_e = (e - e_last) / dt
    e_last = e
    Pout = K_p*e + K_i*integral_e + K_d*diff_e
    power = min(max(Pout, 0.0), 4.0)
    # if i % 100 == 0:
    # print(f'e={e:8.3f} Pout={Pout:8.3f} power={power:8.3f}')
    return (rotate, power)


# Automated Testing
def score(result):
    Xs, Vs, As, thrust, fuels, success = result
    unit_conversion = 1.0
    return np.sqrt(Vs[-1][1]**2)  # + unit_conversion * fuels[-1]**2)


X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
V0 = [0., 0., ]
A0 = [0., -g]
results = []

# TODO: Arange gives weird floats not nice for formatting, fix
# first number on linspace must not be 0 otherwise is not height dependent (hovers)
K_hlist = list(np.linspace(0.001, 0.050, 8))
K_plist = list(np.linspace(0.000, 2.000, 6))
K_ilist = list(np.linspace(0.000, 2.000, 6))
K_dlist = list(np.linspace(0.000, 2.000, 6))


# Automated Testing (P Vertical)
print("Initialising proportional autopilot testing:")
print()

start_time = time.clock()

Trial_combinations = np.size(K_hlist) * np.size(K_plist)

Trials = itertools.product(K_hlist, K_plist)

for Trial in Trials:
    parameters = {
        'K_h': Trial[0],
        'K_p': Trial[1]
    }
    result = simulate(X0, V0, A0, land, landing_site, dt=0.1, Nstep=2000, print_interval=10000000,
                      autopilot=proportional_autopilot, fuel=500, parameters=parameters)
    # add final positions, velocities and fuel load
    results.append([parameters, score(result)])

results = sorted(results, key=lambda x: x[1])

with open('Trial Results P.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in results:
        writer.writerow([json.dumps(row[0]), str(row[1])])

print("The top 5 tuning combinations tested for the proportional autopilot are:")
print()
pprint.pprint(results[:5])
print()

trial_time = time.clock() - start_time
print("It took " + f'{trial_time:.3f}' + " seconds to test " +
      str(Trial_combinations) + " Proportional autopilot trials.")
print("\n")


# Automated Testing (PI Vertical)
print("Initialising proportional-integral autopilot testing:")
print()

start_time = time.clock()

Trial_combinations = np.size(
    K_hlist) * np.size(K_plist) * np.size(K_ilist)

Trials = itertools.product(K_hlist, K_plist, K_ilist)

for Trial in Trials:
    parameters = {
        'K_h': Trial[0],
        'K_p': Trial[1],
        'K_i': Trial[2]
    }
    result = simulate(X0, V0, A0, land, landing_site, dt=0.1, Nstep=2000, print_interval=10000000,
                      autopilot=pi_autopilot, fuel=500, parameters=parameters)
    # add final positions, velocities and fuel load
    results.append([parameters, score(result)])

results = sorted(results, key=lambda x: x[1])


with open('Trial Results PI.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in results:
        writer.writerow([json.dumps(row[0]), str(row[1])])

print("The top 5 tuning combinations tested for the PI autopilot are:")
print()
pprint.pprint(results[:5])
print()

trial_time = time.clock() - start_time
print("It took " + f'{trial_time:.3f}' + " seconds to test " +
      str(Trial_combinations) + " PI autopilot trials.")
print("\n")


# Automated Testing (PID Vertical)
print("Initialising proportional-integral-derivative autopilot testing:")
print()

start_time = time.clock()

Trial_combinations = np.size(
    K_hlist) * np.size(K_plist) * np.size(K_ilist) * np.size(K_dlist)

Trials = itertools.product(K_hlist, K_plist, K_ilist, K_dlist)

for Trial in Trials:
    parameters = {
        'K_h': Trial[0],
        'K_p': Trial[1],
        'K_i': Trial[2],
        'K_d': Trial[3]
    }
    result = simulate(X0, V0, A0, land, landing_site, dt=0.1, Nstep=2000, print_interval=10000000,
                      autopilot=pid_autopilot, fuel=500, parameters=parameters)
    # add final positions, velocities and fuel load
    results.append([parameters, score(result)])

results = sorted(results, key=lambda x: x[1])

# TODO: Write to csv with parameters in a box as a float, retrieve best result for plotting
with open('Trial Results PID.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in results:
        writer.writerow([json.dumps(row[0]), str(row[1])])

print("The top 5 tuning combinations tested for the PID autopilot are:")
print()
pprint.pprint(results[:5])
print()
trial_time = time.clock() - start_time
print("It took " + f'{trial_time:.3f}' + " seconds to test " +
      str(Trial_combinations) + " PID autopilot trials.")
print("\n")
print("Testing complete!")

#
# argmin for retrieving coefficients
# scipy.minimize for more efficient search
#

# Best Autopilot test:


def best_autopilot(i, X, V, fuel, rotate, power, parameters):
    c = 0.0  # target landing speed, m/s
    e = 0
    e_last = 0
    K_h = 0.029
    K_p = 2
    K_i = 2
    K_d = 1.2
    h = height(land, X)
    e = - (c + K_h*h + V[1])
    integral_e = 0
    integral_e += e * dt
    diff_e = 0
    diff_e = (e - e_last) / dt
    e_last = e
    Pout = K_p*e + K_i*integral_e + K_d*diff_e
    power = min(max(Pout, 0.0), 4.0)
    # if i % 100 == 0:
    # print(f'e={e:8.3f} Pout={Pout:8.3f} power={power:8.3f}')
    return (rotate, power)


# TODO: Showcase random velocity
X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
Vv_init = np.random.uniform(-10, -20)
V0 = [0., Vv_init]
Xs, Vs, As, thrust, fuels, success = simulate(X0, V0, A0, land, landing_site, dt=0.1, Nstep=2000,  # Increase Nstep for longer simulation
                                              autopilot=best_autopilot, fuel=500)
plot_lander(land, landing_site, Xs, thrust, animate=True, step=10)

# PLOTTING TARGET SPEED AND ACTUAL SPEED
c = 0.0
K_h = 0.029  # fill in your value of K_h here
K_p = 2
K_i = 2
K_d = 1.2
h = np.array([height(land, Xs[i, :]) for i in range(len(Xs))])

fig, ax = plt.subplots()
ax.plot(-h, Vs[:, 1], label="Actual speed")
ax.plot(-h, As[:, 1], label="Acceleration experienced")
ax.set_xlabel("- Altitude")
ax.set_ylabel("Vertical velocity")
ax.plot(-h, -(c + K_h*h), label=f"Target speed K$_h$={K_h}")
ax.legend()


# # PLOTTING ENERGY DRIFT
# m = 1000.  # mass of lander in kg
# dt = 0.1
# X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
# V0 = [0., Vv_init]
# # number of steps required for 1 second of simulated time
# Nstep = int((1.0 / dt) + 1)
# Xs, Vs, thrust, success = simulate(
#     X0, V0, land, landing_site, dt=dt, Nstep=Nstep)

# t = np.array([dt*i for i in range(Nstep)])
# V = np.array([m*g*Xs[i, 1] for i in range(Nstep)])
# T = np.array([0.5*m*norm(Vs[i, :])**2 for i in range(Nstep)])
# E = T + V

# fig, ax = plt.subplots()
# ax.plot(t, abs(E - E[0])/E[0], label="Total energy drift")
# ax.set_xlabel('Time / s')
# ax.set_ylabel('Energy drift')
# ax.legend()
# assert t[0] == 0.0
# assert t[-1] >= 1.0
# assert len(t) == len(E) == Nstep
# assert abs(E[-1] - E[0])/E[0] < 1e-3
