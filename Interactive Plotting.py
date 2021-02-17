from MarsLanderPython import *

# Extract best P test result
with open('1D Trial Results P.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

K_h = float(data[0][0])
K_p = float(data[0][1])
K_i = 0
K_d = 0

with open('2D Trial Results P.csv') as csvDataFile:
    data2 = list(csv.reader(csvDataFile))

K_diffx = float(data2[0][0])
K_px = float(data2[0][1])
K_ix = 0
K_dx = 0

parameters = {'K_h': K_h,
              'K_p': K_p,
              'K_i': K_i,
              'K_d': K_d,
              'K_diffx': K_diffx,
              'K_px': K_px,
              'K_ix': K_ix,
              'K_dx': K_dx,}

best_autopilot = p_autopilot
np.random.seed(42)
land, landing_site = mars_surface()
Xs, Vs, As, thrust, fuels, errory, errorx, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                                      autopilot=best_autopilot, fuel=500, parameters=parameters)
X1=Xs
thrust1=thrust


# Extract best PI test result

with open('1D Trial Results PI.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

K_h = float(data[0][0])
K_p = float(data[0][1])
K_i = float(data[0][2])
K_d = 0

with open('2D Trial Results PI.csv') as csvDataFile:
    data2 = list(csv.reader(csvDataFile))

K_diffx = float(data2[0][0])
K_px = float(data2[0][1])
K_ix = float(data2[0][2])
K_dx = 0

parameters = {'K_h': K_h,
              'K_p': K_p,
              'K_i': K_i,
              'K_d': K_d,
              'K_diffx': K_diffx,
              'K_px': K_px,
              'K_ix': K_ix,
              'K_dx': K_dx,}

best_autopilot = pi_autopilot
Xs, Vs, As, thrust, fuels, errory, errorx, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                                      autopilot=best_autopilot, fuel=500, parameters=parameters)
X2=Xs
thrust2=thrust


# Extract best PID test result

with open('1D Trial Results PID.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

K_h = float(data[0][0])
K_p = float(data[0][1])
K_i = float(data[0][2])
K_d = float(data[0][3])

with open('2D Trial Results PID.csv') as csvDataFile:
    data2 = list(csv.reader(csvDataFile))

K_diffx = float(data2[0][0])
K_px = float(data2[0][1])
K_ix = float(data2[0][2])
K_dx = float(data2[0][3])

parameters = {'K_h': K_h,
              'K_p': K_p,
              'K_i': K_i,
              'K_d': K_d,
              'K_diffx': K_diffx,
              'K_px': K_px,
              'K_ix': K_ix,
              'K_dx': K_dx,}

best_autopilot = pid_autopilot
Xs, Vs, As, thrust, fuels, errory, errorx, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                                      autopilot=best_autopilot, fuel=500, parameters=parameters)
X3=Xs
thrust3=thrust


# Extract 500th best PID test result

with open('1D Trial Results PID.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

K_h = float(data[500][0])
K_p = float(data[500][1])
K_i = float(data[500][2])
K_d = float(data[500][3])

with open('2D Trial Results PID.csv') as csvDataFile:
    data2 = list(csv.reader(csvDataFile))

K_diffx = float(data2[500][0])
K_px = float(data2[500][1])
K_ix = float(data2[500][2])
K_dx = float(data2[500][3])

parameters = {'K_h': K_h,
              'K_p': K_p,
              'K_i': K_i,
              'K_d': K_d,
              'K_diffx': K_diffx,
              'K_px': K_px,
              'K_ix': K_ix,
              'K_dx': K_dx,}

best_autopilot = pid_autopilot
Xs, Vs, As, thrust, fuels, errory, errorx, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                                      autopilot=best_autopilot, fuel=500, parameters=parameters)
X4=Xs
thrust4=thrust


# Extract 3000th best PID test result

with open('1D Trial Results PID.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

K_h = float(data[3000][0])
K_p = float(data[3000][1])
K_i = float(data[3000][2])
K_d = float(data[3000][3])

with open('2D Trial Results PID.csv') as csvDataFile:
    data2 = list(csv.reader(csvDataFile))

K_diffx = float(data2[3000][0])
K_px = float(data2[3000][1])
K_ix = float(data2[3000][2])
K_dx = float(data2[3000][3])

parameters = {'K_h': K_h,
              'K_p': K_p,
              'K_i': K_i,
              'K_d': K_d,
              'K_diffx': K_diffx,
              'K_px': K_px,
              'K_ix': K_ix,
              'K_dx': K_dx,}

best_autopilot = pid_autopilot
Xs, Vs, As, thrust, fuels, errory, errorx, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                                                      autopilot=best_autopilot, fuel=500, parameters=parameters)
X5=Xs
Xz=[X1, X2, X3, X4, X5]
thrust5=thrust
thrustz=[thrust1, thrust2, thrust3, thrust4, thrust5]
plot_lander(land, landing_site, Xz, thrustz, animate=True, step=10)