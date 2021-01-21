from MarsLanderPython import *


# Best Autopilot test:

# Extract PID test results
with open('1D Trial Results Praw.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

K_h = float(data[0][0])
K_p = float(data[0][1])
K_i = 0
K_d = 0

with open('2D Trial Results Praw.csv') as csvDataFile:
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


# Plotting the best score trial
X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
V0 = [0., 0.]
#Vv_init = np.random.uniform(-10, -20)
#V0 = [0., Vv_init]
best_autopilot = proportional_autopilot
Xs, Vs, As, thrust, fuels, errory, errorx, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,  # Increase Nstep for longer simulation
                                                      autopilot=best_autopilot, fuel=500, parameters=parameters)
plot_lander(land, landing_site, Xs, thrust, animate=True, step=10)

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
if hdiffmax >= 300:
    munitprefix = 1000
elif hdiffmax >= 30:
    munitprefix = 100
elif hdiffmax >= 3:
    munitprefix = 10
else: 
    munitprefix = 1
Hdiff = ax7.plot(t, hdiff/munitprefix, '-', color='orange', label="Horizontal position error (*"+str(munitprefix)+"m)")
ax8 = ax7.twinx()
Accel_exp = ax8.plot(t, As[:, 0], 'r-',
                     label="Horizontal acceleration (m/s^2)")

lines = Actual_velocity+Accel_exp+Hdiff
labs = [l.get_label() for l in lines]
ax7.legend(lines, labs, loc=4)
ax7.set_xlabel("Time(s)")
ax7.set_ylabel("Horizontal velocity (m/s)")
ax8.set_ylabel("Horizontal acceleration (m/s^2)")
ax7.set_ylim(-3, +3)
ax8.set_ylim(-1, +1)
ax7.grid(True)


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
