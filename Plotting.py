from MarsLanderPython import *


# Best Autopilot test:

# Extract PID test results
with open('Trial Results PIDraw.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

# K_h = float(data[0][0])
# K_p = float(data[0][1])
# K_i = float(data[0][2])
# K_d = float(data[0][3])
K_h = 0.009
K_p = 0.1
K_i = 0.004
K_d = 0.03

parameters = {'K_h': K_h,
              'K_p': K_p,
              'K_i': K_i,
              'K_d': K_d}


# Plotting the best score trial
X0 = [(land[landing_site+1, 0] + land[landing_site, 0]) // 2, 3000]
V0 = [0., 0.]
#Vv_init = np.random.uniform(-10, -20)
#V0 = [0., Vv_init]
best_autopilot = pid_autopilot
Xs, Vs, As, thrust, fuels, errors, success = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,  # Increase Nstep for longer simulation
                                                      autopilot=best_autopilot, fuel=500, parameters=parameters)
plot_lander(land, landing_site, Xs, thrust, animate=True, step=10)

# Plotting target speed and actual speed
c = 0.0
h = np.array([height(land, Xs[i, :]) for i in range(len(Xs))])

fig = plt.figure()
plt.subplots_adjust(hspace=0.4)
ax = fig.add_subplot(311)

Actual_velocity = ax.plot(-h, Vs[:, 1], 'b-', label="Actual velocity (m/s)")
Target_velocity = ax.plot(-h, -(c + K_h*h), 'g-',
                          label=f"Target velocity K$_h$={K_h}")
ax2 = ax.twinx()
Accel_exp = ax2.plot(-h, As[:, 1], 'r-',
                     label="Acceleration experienced (m/s^2)")

lines = Actual_velocity+Target_velocity+Accel_exp
labs = [l.get_label() for l in lines]
ax.legend(lines, labs, loc=0)
ax.set_xlabel("-Altitude (m)")
ax.set_ylabel("Vertical velocity (m/s)")
ax2.set_ylabel("Vertical acceleration (m/s^2)")
ax2.set_ylim(-5, +5)
ax.grid(True)

ax3 = fig.add_subplot(312)
Fuel1 = ax3.plot(-h, fuels[:], 'm-', label="Fuel remaining (kg)")

ax4 = ax3.twinx()
Thrust1 = ax4.plot(-h, thrust[:, 1],
                   'c-', label="Thrust (N)")

lines = Thrust1+Fuel1
labs = [l.get_label() for l in lines]
ax3.legend(lines, labs, loc=0)
ax3.set_xlabel("-Altitude (m)")
ax3.set_ylabel("Fuel remaining (kg)")
#ax3.set_ylim(0, +500)
ax4.set_ylabel("Thrust (N)")
#ax4.set_ylim(0, +100)
ax3.grid(True)


ax5 = fig.add_subplot(313)
Nstep = int(len(Vs))
t = np.array([dt*i for i in range(Nstep)])
Actual_velocity = ax5.plot(
    t, Vs[:, 1], 'b-', label="Actual velocity (m/s)")
Target_velocity = ax5.plot(t, -(c + K_h*h), 'g-',
                           label=f"Target velocity K$_h$={K_h}")
ax6 = ax5.twinx()
Accel_exp = ax6.plot(t, As[:, 1], 'r-',
                     label="Acceleration experienced (m/s^2)")

lines = Actual_velocity+Target_velocity+Accel_exp
labs = [l.get_label() for l in lines]
ax5.legend(lines, labs, loc=0)
ax5.set_xlabel("Time")
ax5.set_ylabel("Vertical velocity (m/s)")
ax6.set_ylabel("Vertical acceleration (m/s^2)")
ax6.set_ylim(-5, +5)
ax5.grid(True)


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
