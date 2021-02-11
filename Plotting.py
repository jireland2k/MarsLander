from MarsLanderPython import *
from mpl_toolkits.mplot3d import Axes3D

#Success Statistics
#P
with open('2D Trial Results P.csv') as csvDataFile:
    succdata = list(csv.reader(csvDataFile))

testlength = len(succdata)
succ = 0
for i in range(testlength):
    succ += int(succdata[i][7])
print(str(succ) + " successful parameter combinations out of " + str(testlength) + " in P optimisation")

#PI
with open('2D Trial Results PI.csv') as csvDataFile:
    succdata = list(csv.reader(csvDataFile))

testlength = len(succdata)
succ = 0
for i in range(testlength):
    succ += int(succdata[i][8])
print(str(succ) + " successful parameter combinations out of " + str(testlength) + " in PI optimisation")

#PID
with open('2D Trial Results PID.csv') as csvDataFile:
    succdata = list(csv.reader(csvDataFile))

testlength = len(succdata)
succ = 0
for i in range(testlength):
    succ += int(succdata[i][9])
print(str(succ) + " successful parameter combinations out of " + str(testlength) + " in PID optimisation")

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


# Plotting target speed and actual speed

c = 0.0
h = np.array([height(land, Xs[i, :]) for i in range(len(Xs))])

fig = plt.figure()
plt.subplots_adjust(hspace=0.3, wspace=0.4)
ax = fig.add_subplot(221)
Actual_velocity = ax.plot(-h, Vs[:, 1], 'b-', label="Actual velocity (m/s)")
Target_velocity = ax.plot(-h, -(c + K_h*h), 'g-',
                        label=f"Target velocity K$_h$={K_h:.3f}")
ax2 = ax.twinx()
Accel_exp = ax2.plot(-h, As[:, 1], 'r-',
                    label="Acceleration (m/s^2)")

lines = Actual_velocity+Target_velocity+Accel_exp
labs = [l.get_label() for l in lines]
ax.legend(lines, labs, loc=0)
fig.suptitle("Optimal PID Autopilot Flight Data")
ax.set_xlabel("-Altitude (m)")
ax.set_ylabel("Vertical velocity (m/s)")
ax2.set_ylabel("Vertical acceleration (m/s^2)")
ax2.set_ylim(-4, +2)
ax.grid(True)

ax3 = fig.add_subplot(222)
Fuel1 = ax3.plot(-h, fuels[:], 'm-', label="Fuel remaining (kg)")

ax4 = ax3.twinx()
Thrust1 = ax4.plot(-h, thrust[:, 1],
                'c-', label="Thrust (N)")

lines = Thrust1+Fuel1
labs = [l.get_label() for l in lines]
ax3.legend(lines, labs, loc=8)
ax3.set_xlabel("-Altitude (m)")
ax3.set_ylabel("Fuel remaining (kg)")
ax3.set_ylim(0)
ax4.set_ylabel("Thrust (N)")
# this locator puts ticks at regular intervals
loc = plticker.MultipleLocator(base=100)
ax3.yaxis.set_major_locator(loc)
ax3.grid(True)


ax5 = fig.add_subplot(223)
Nstep = int(len(Vs))
t = np.array([dt*i for i in range(Nstep)])
Actual_velocity = ax5.plot(
    t, Vs[:, 1], 'b-', label="Actual velocity (m/s)")
Target_velocity = ax5.plot(t, -(c + K_h*h), 'g-',
                        label=f"Target velocity K$_h$={K_h:.3f}")
ax6 = ax5.twinx()
Accel_exp = ax6.plot(t, As[:, 1], 'r-',
                    label="Acceleration (m/s^2)")

lines = Actual_velocity+Target_velocity+Accel_exp
labs = [l.get_label() for l in lines]
ax5.legend(lines, labs, loc=4)
ax5.set_xlabel("Time(s)")
ax5.set_ylabel("Vertical velocity (m/s)")
ax6.set_ylabel("Vertical acceleration (m/s^2)")
ax6.set_ylim(-4, +2)
ax5.grid(True)


ax7 = fig.add_subplot(224)
Nstep = int(len(Vs))
t = np.array([dt*i for i in range(Nstep)])
Actual_velocity = ax7.plot(t, Vs[:, 0], 'b-', label="Horizontal velocity (m/s)")
landtarget = ((land[landing_site+1, 0] + land[landing_site, 0]) // 2)
hdiff = Xs[:, 0]-landtarget
hdiffmax = np.amax(hdiff)
if hdiffmax >= 1500:
    munitprefix = 1000
elif hdiffmax >= 150:
    munitprefix = 100
elif hdiffmax >= 15:
    munitprefix = 10
else: 
    munitprefix = 1
Hdiff = ax7.plot(t, hdiff/munitprefix, '-', color='orange', 
                 label="Horizontal position error (*"+str(munitprefix)+"m)")
ax8 = ax7.twinx()
Accel_exp = ax8.plot(t, As[:, 0], 'r-',
                    label="Horizontal acceleration (m/s^2)")

lines = Actual_velocity+Accel_exp+Hdiff
labs = [l.get_label() for l in lines]
ax7.legend(lines, labs, loc=4)
ax7.set_xlabel("Time(s)")
ax7.set_ylabel("Horizontal velocity (m/s)")
ax8.set_ylabel("Horizontal acceleration (m/s^2)")
ax7.set_ylim(-15, +15)
ax8.set_ylim(-3, +3)
ax7.grid(True)


additional_plots = True
if additional_plots == True:

    #P trials velocity/position plot
    scoreP = []
    fuelP = []
    dftP = []
    velP = []
    with open('2D Trial Results P.csv') as csvDataFile:
        data3 = list(csv.reader(csvDataFile))
        
    for row in data3:
        scoreP.append([float(row[2])])
        fuelusedP = 500 - float(row[3])
        fuelP.append([fuelusedP])
        distance_from_target = float(row[4])
        dftP.append(distance_from_target)
        vel1 = float(row[5])
        vel2 = float(row[6])
        vel3 = np.sqrt(vel1**2 + vel2**2)
        velP.append([vel3])

    fig2 = plt.figure()
    ax = fig2.add_subplot(111)
    ax.scatter(dftP, velP, c='r', marker='.', alpha = 0.3)       
    ax.set_xlabel("Distance from Target (m)")
    ax.set_ylabel("Velocity at Impact (m/s)")
    ax.grid(True)


    #PI trials velocity/position plot
    scorePI = []
    fuelPI = []
    dftPI = []
    velPI = []
    with open('2D Trial Results PI.csv') as csvDataFile:
        data4 = list(csv.reader(csvDataFile))
        
    for row in data4:
        scorePI.append([float(row[3])])
        fuelusedPI = 500 - float(row[4])
        fuelPI.append([fuelusedPI])
        distance_from_target = float(row[5])
        dftPI.append(distance_from_target)
        vel1 = float(row[6])
        vel2 = float(row[7])
        vel3 = np.sqrt(vel1**2 + vel2**2)
        velPI.append([vel3])

    fig3 = plt.figure()
    ax = fig3.add_subplot(111)
    ax.scatter(dftPI, velPI, c='orange', marker='.', alpha = 0.3)       
    ax.set_xlabel("Distance from Target (m)")
    ax.set_ylabel("Velocity at Impact (m/s)")
    ax.grid(True)


    #PID trials velocity/position plot
    scorePID = []
    fuelPID = []
    dftPID = []
    velPID = []
    with open('2D Trial Results PID.csv') as csvDataFile:
        data5 = list(csv.reader(csvDataFile))
        
    for row in data5:
        scorePID.append([float(row[4])])
        fuelusedPID = 500 - float(row[5])
        fuelPID.append([fuelusedPID])
        distance_from_target = float(row[6])
        dftPID.append(distance_from_target)
        vel1 = float(row[7])
        vel2 = float(row[8])
        vel3 = np.sqrt(vel1**2 + vel2**2)
        velPID.append([vel3])

    fig4 = plt.figure()
    ax = fig4.add_subplot(111)
    ax.scatter(dftPID, velPID, c='g', marker='.', alpha = 0.3)       
    ax.set_xlabel("Distance from Target (m)")
    ax.set_ylabel("Velocity at Impact (m/s)")
    ax.grid(True)


    #all three autopilots superposed

    #vel/dis
    fig5 = plt.figure()
    ax = fig5.add_subplot(111)
    ax.scatter(dftP, velP,c='r', marker='.', alpha = 0.1)
    ax.scatter(dftPI, velPI, c='orange', marker='.', alpha = 0.2)
    ax.scatter(dftPID, velPID, c='g', marker='.', alpha = 0.1)       
    ax.set_xlabel("Distance from Target (m)")
    ax.set_ylabel("Velocity at Impact (m/s)")
    ax.grid(True)

    #score/dis
    fig6 = plt.figure()
    ax = fig6.add_subplot(111)
    ax.scatter(dftP, scoreP,c='r', marker='.', alpha = 0.1)
    ax.scatter(dftPI, scorePI, c='orange', marker='.', alpha = 0.2)
    ax.scatter(dftPID, scorePID, c='g', marker='.', alpha = 0.1)       
    ax.set_xlabel("Distance from Target (m)")
    ax.set_ylabel("Score (less is better)")
    ax.grid(True)

    #score/vel
    fig7 = plt.figure()
    ax = fig7.add_subplot(111)
    ax.scatter(velP, scoreP,c='r', marker='.', alpha = 0.1)
    ax.scatter(velPI, scorePI, c='orange', marker='.', alpha = 0.2)
    ax.scatter(velPID, scorePID, c='g', marker='.', alpha = 0.1)       
    ax.set_xlabel("Velocity at Impact (m/s)")
    ax.set_ylabel("Score (less is better)")
    ax.grid(True)

    #fuel/vel
    fig8 = plt.figure()
    ax = fig8.add_subplot(111)
    ax.scatter(velP, fuelP, c='r', marker='.', alpha = 0.1)
    ax.scatter(velPI, fuelPI, c='orange', marker='.', alpha = 0.2)
    ax.scatter(velPID, fuelPID, c='g', marker='.', alpha = 0.1)       
    ax.set_xlabel("Velocity at Impact (m/s)")
    ax.set_ylabel("Fuel used (kg)")
    ax.grid(True)

    #fuel/dis
    fig9 = plt.figure()
    ax = fig9.add_subplot(111)
    ax.scatter(dftP, fuelP,c='r', marker='.', alpha = 0.1)
    ax.scatter(dftPI, fuelPI, c='orange', marker='.', alpha = 0.2)
    ax.scatter(dftPID, fuelPID, c='g', marker='.', alpha = 0.1)       
    ax.set_xlabel("Distance from Target (m)")
    ax.set_ylabel("Fuel used (kg)")
    ax.grid(True)

    #score/fuel
    fig10 = plt.figure()
    ax = fig10.add_subplot(111)
    ax.scatter(fuelP, scoreP,c='r', marker='.', alpha = 0.1)
    ax.scatter(fuelPI, scorePI, c='orange', marker='.', alpha = 0.2)
    ax.scatter(fuelPID, scorePID, c='g', marker='.', alpha = 0.1)       
    ax.set_xlabel("Fuel used (kg)")
    ax.set_ylabel("Score (less is better)")
    ax.grid(True)
    print("Done!")
else:
    print("Done!")