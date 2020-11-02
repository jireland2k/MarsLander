import numpy as np
import matplotlib.pyplot as plt

from numpy.random import randint
from numpy.linalg import norm

from ipywidgets import interactive

from matplotlib import rcParams
rcParams['figure.figsize'] = (10, 8)


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
                ax.plot([X[n, 0], X[n, 0] - 100*thrust[n, 0]],
                        [X[n, 1] - 100., X[n, 1] - 100. - 100*thrust[n, 1]],
                        'r-', lw=10)
        return interactive(plot_frame, n=(0, len(X), step))
    else:
        ax = plot_surface(land, landing_site)
        ax.plot(X[:, 0], X[:, 1], 'b--')
        ax.plot(X[-1, 0], X[-1, 1], 'b^')
        return ax


np.random.seed(42)  # seed random number generator for reproducible results
land, landing_site = mars_surface()
plot_surface(land, landing_site)


def interpolate_surface(land, x):
    i,  = np.argwhere(land[:, 0] < x)[-1]  # segment containing x is [i, i+1]
    m = (land[i+1, 1] - land[i, 1])/(land[i+1, 0] - land[i, 0])  # gradient
    x1, y1 = land[i, :]  # point on line with eqn. y - y1 = m(x - x1)
    return m*(x - x1) + y1


x = 2000  # try varying x and testing if function works as expected
y = interpolate_surface(land, x)
plot_surface(land, landing_site)
plt.plot(x, y, 'bo')


def height(land, X):
    return X[1] - interpolate_surface(land, X[0])


#  height when on surface left edge should be close to zero
assert abs(height(land, [1, land[0, 1]])) < 100.0
# height when on surface at right edge should be close to zero
assert abs(height(land, [6999, land[-1, 1]])) < 100.0

_land, _landing_site = mars_surface()


def _height(_land, X):
    return X[1] - interpolate_surface(_land, X[0])


points = np.zeros((10, 2))
points[:, 0] = randint(0, 7000, size=10)
points[:, 1] = randint(0, 3000, size=10)
for i in range(10):
    assert abs(height(_land, points[i, :]) -
               _height(_land, points[i, :])) < 1e-6
g = 3.711  #  m/s^2, gravity on Mars


def simulate(X0, V0, land, landing_site,
             fuel=200, dt=0.1, Nstep=1000,
             autopilot=None, print_interval=100):

    n = len(X0)       # number of degrees of freedom (2 here)
    X = X0.copy()     # current position
    V = V0.copy()     # current velocity
    Xs = np.zeros((Nstep, n))  # position history (trajectory)
    Vs = np.zeros((Nstep, n))  # velocity history
    thrust = np.zeros((Nstep, n))  # thrust history
    success = False
    fuel_warning_printed = False
    rotate = 0  #  degrees, initial angle
    power = 0            # m/s^2, initial thrust power

    for i in range(Nstep):
        Xs[i, :] = X     # Store positions
        Vs[i, :] = V     # Store velocities

        if autopilot is not None:
            # call user-supplied function to set `rotate` and `power`
            rotate, power = autopilot(i, X, V, fuel, rotate, power)
            assert abs(rotate) <= 90
            assert 0 <= power <= 4

            rotate_rad = rotate * np.pi / 180.0  # degrees to radians
            thrust[i, :] = power * np.array([np.sin(rotate_rad),
                                             np.cos(rotate_rad)])
            if fuel <= 0:
                if not fuel_warning_printed:
                    print("Fuel empty! Setting thrust to zero")
                    fuel_warning_printed = True
                thrust[i, :] = 0
            else:
                fuel -= power*dt

        A = np.array([0, -g]) + thrust[i, :]  # acceleration
        V += A * dt                          # update velocities
        X += V * dt                          # update positions

        if i % print_interval == 0:
            print(f"i={i:03d} X=[{X[0]:8.3f} {X[1]:8.3f}] V=[{V[0]:8.3f} {V[1]:8.3f}]"
                  f" thrust=[{thrust[i, 0]:8.3f} {thrust[i, 1]:8.3f}] fuel={fuel:8.3f}")

        # check for safe or crash landing
        if X[1] < interpolate_surface(land, X[0]):
            if not (land[landing_site, 0] <= X[0] and X[0] <= land[landing_site + 1, 0]):
                print("crash! did jot land on flat ground!")
            elif rotate != 0:
                print(
                    "crash! did not land in a vertical position (tilt angle = 0 degrees)")
            elif abs(V[1]) >= 40:
                print(
                    "crash! vertical speed must be limited (<40m/s in absolute value), got ", abs(V[1]))
            elif abs(V[0]) >= 20:
                print(
                    "crash! horizontal speed must be limited (<20m/s in absolute value), got ", abs(V[0]))
            else:
                print("safe landing - well done!")
                success = True
            Nstep = i
            break

    return Xs[:Nstep, :], Vs[:Nstep, :], thrust[:Nstep, :], success


X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
V0 = [0.0, 0.0]
Xs, Vs, thrust, success = simulate(X0, V0, land, landing_site)
plot_lander(land, landing_site, Xs, thrust, animate=True, step=10)
m = 100.  # mass of lander in kg
dt = 0.1
# number of steps required for 1 second of simulated time
Nstep = int((1.0 / dt) + 1)
Xs, Vs, thrust, success = simulate(
    X0, V0, land, landing_site, dt=dt, Nstep=Nstep)

t = np.array([dt*i for i in range(Nstep)])
V = np.array([m*g*Xs[i, 1] for i in range(Nstep)])
T = np.array([0.5*m*norm(Vs[i, :])**2 for i in range(Nstep)])
E = T + V

fig, ax = plt.subplots()
ax.plot(t, abs(E - E[0])/E[0], label="Total energy drift")
ax.set_xlabel('Time / s')
ax.set_ylabel('Energy drift')
ax.legend()
assert t[0] == 0.0
assert t[-1] >= 1.0
assert len(t) == len(E) == Nstep
assert abs(E[-1] - E[0])/E[0] < 1e-3


def dummy_autopilot(i, X, V, fuel, rotate, power):
    return (rotate, power)  # do nothing


X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
V0 = [0., 0.]
Xs, Vs, thrust, success = simulate(
    X0, V0, land, landing_site, dt=dt, autopilot=dummy_autopilot)
plot_lander(land, landing_site, Xs, thrust, animate=True, step=10)


def proportional_autopilot(i, X, V, fuel, rotate, power):
    c = 10.0  # target landing speed, m/s
    K_h = 0.01
    K_p = 0.2
    h = height(land, X)
    e = - (c + K_h*h + V[1])
    Pout = K_p*e
    power = min(max(Pout, 0.0), 4.0)
    if i % 100 == 0:
        print(f'e={e:8.3f} Pout={Pout:8.3f} power={power:8.3f}')
    return (rotate, power)


X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
V0 = [0., 0.]
Xs, Vs, thrust, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                   autopilot=proportional_autopilot, fuel=np.inf)
plot_lander(land, landing_site, Xs, thrust, animate=True, step=10)
c = 30.
K_h = 0.01  # fill in your value of K_h here

h = np.array([height(land, Xs[i, :]) for i in range(len(Xs))])

fig, ax = plt.subplots()
ax.plot(-h, Vs[:, 1], label="actual speed")
ax.set_xlabel("- altitude")
ax.set_ylabel("Vertical velocity")
ax.plot(-h, -(c + K_h*h), label=f"target speed K$_h$={K_h}")
ax.legend()
# first test with infinite fuel
X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
V0 = [0., 0., ]
Xs, Vs, thrust, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                   autopilot=proportional_autopilot, fuel=np.inf)
assert success
# now test with fuel=200
X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
V0 = [0., 0., ]
Xs, Vs, thrust, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                   autopilot=proportional_autopilot, fuel=200)
assert success
# Test 1. Random vertical starting positions
np.random.seed(123)  # seed random number generator for reproducible results
land, landing_site = mars_surface()

trials = 20
count = 0
for i in range(trials):
    X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) //
          2, randint(1500, 3000)]
    V0 = [0., 0.]
    print(f'X0={X0} V0={V0}')
    Xs, Vs, thrust, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                       autopilot=proportional_autopilot, fuel=200)
    count += success
    print()

print(f'count/trials = {count/trials}')
assert count/trials > 0.8  # require 80% success rate
# Test 2 - random initial vertical velocity

np.random.seed(123)  # seed random number generator for reproducible results
land, landing_site = mars_surface()

trials = 20
count = 0
for i in range(trials):
    X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) //
          2, randint(1500, 3000)]
    V0 = [0., np.random.uniform(-50, 50)]
    print(f'X0={X0} V0={V0}')
    Xs, Vs, thrust, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                       autopilot=proportional_autopilot, fuel=200)
    count += success
    print()

print(f'count/trials = {count/trials}')
assert count/trials > 0.8  # require 80% success rate
# Test 3 - random Martian surfaces

np.random.seed(123)  # seed random number generator for reproducible results

trials = 20
count = 0
for i in range(trials):
    land, landing_site = mars_surface()
    X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) //
          2, randint(1500, 3000)]
    V0 = [0., 0.]
    print(f'X0={X0} V0={V0}')
    Xs, Vs, thrust, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                       autopilot=proportional_autopilot, fuel=200)
    count += success
    print()

print(f'count/trials = {count/trials}')
assert count/trials > 0.8  # require 80% success rate
