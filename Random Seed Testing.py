from MarsLanderPython import *

randomPresult = []
randomPIresult = []
randomPIDresult = []
ntest = 1000


#P Random Seed Testing
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

print("Initialising proportional autopilot testing on random seeds:")
print()

start_time = time.clock()

for i in range(ntest):
    land, landing_site = mars_surface()
    wind = np.random.uniform(-31, 31)
    hoffset = np.random.uniform(-500, 500)
    VXinit = np.random.uniform(-10, 10)
    VYinit = np.random.uniform(-10, -20)
    X0 = [((land[landing_site+1, 0] + land[landing_site, 0]) // 2)+hoffset, 3000]
    V0 = [VXinit, VYinit]
    
    best_autopilot = p_autopilot
    result = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                      autopilot=best_autopilot, fuel=500, parameters=parameters)
    Xs, Vs, As, thrust, fuels, errory, errorx, success = result
    landtarget = ((land[landing_site+1, 0] + land[landing_site, 0]) // 2)
    hdifffinal = Xs[-1][0]-landtarget

    randomPresult.append(["wind", wind, "hoffset", hoffset, "V0", V0, "Score", score(result, land, landing_site), 
                            "Fuel Remaining", fuels[-1], "Distance to Target", hdifffinal, "Final Velocity", 
                            Vs[-1][0], Vs[-1][1], "Success", int(success)])

randomPresult = sorted(randomPresult, key=lambda x: x[7])

with open('randomseedP.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in randomPresult:
        writer.writerow([str(row[1]), str(row[3]), str(row[5]), str(row[7]), str(row[9]), str(row[11]), 
                         str(row[13]), row[14], row[16]])

trial_time = time.clock() - start_time

print("It took " + f'{trial_time:.3f}' + " seconds to test " +
      str(ntest) + " P autopilot trials.")

with open('randomseedP.csv') as csvDataFile:
    succdata = list(csv.reader(csvDataFile))

testlength = len(succdata)
succ = 0
for i in range(testlength):
    succ += int(succdata[i][8])
print(str(succ) + " successful landings out of " + str(testlength) + " random seeds in P testing.")
print("\n")

Xz = [Xs]
thrustz = [thrust]
plot_lander(land, landing_site, Xz, thrustz, animate=True, step=10)


# PI Random Seed Testing
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

print("Initialising proportional-integral autopilot testing on random seeds:")
print()

start_time = time.clock()

for i in range(ntest):
    land, landing_site = mars_surface()
    wind = np.random.uniform(-31, 31)
    hoffset = np.random.uniform(-250, 250)
    VXinit = np.random.uniform(-5, 5)
    VYinit = np.random.uniform(-10, -20)
    X0 = [((land[landing_site+1, 0] + land[landing_site, 0]) // 2)+hoffset, 3000]
    V0 = [VXinit, VYinit]
    
    best_autopilot = pi_autopilot
    result = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                      autopilot=best_autopilot, fuel=500, parameters=parameters)
    Xs, Vs, As, thrust, fuels, errory, errorx, success = result
    landtarget = ((land[landing_site+1, 0] + land[landing_site, 0]) // 2)
    hdifffinal = Xs[-1][0]-landtarget

    randomPIresult.append(["wind", wind, "hoffset", hoffset, "V0", V0, "Score", score(result, land, landing_site), 
                            "Fuel Remaining", fuels[-1], "Distance to Target", hdifffinal, "Final Velocity", 
                            Vs[-1][0], Vs[-1][1], "Success", int(success)])

randomPIresult = sorted(randomPIresult, key=lambda x: x[7])

with open('randomseedPI.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in randomPIresult:
        writer.writerow([str(row[1]), str(row[3]), str(row[5]), str(row[7]), str(row[9]), str(row[11]), 
                         str(row[13]), row[14], row[16]])

trial_time = time.clock() - start_time

print("It took " + f'{trial_time:.3f}' + " seconds to test " +
      str(ntest) + " PI autopilot trials.")


with open('randomseedPI.csv') as csvDataFile:
    succdata = list(csv.reader(csvDataFile))

testlength = len(succdata)
succ = 0
for i in range(testlength):
    succ += int(succdata[i][8])
print(str(succ) + " successful landings out of " + str(testlength) + " random seeds in PI testing.")
print("\n")

Xz = [Xs]
thrustz = [thrust]
plot_lander(land, landing_site, Xz, thrustz, animate=True, step=10)


# PID Random Seed Testing
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

print("Initialising proportional-integral-derivative autopilot testing on random seeds:")
print()

start_time = time.clock()

for i in range(ntest):
    land, landing_site = mars_surface()
    wind = np.random.uniform(-31, 31)
    hoffset = np.random.uniform(-250, 250)
    VXinit = np.random.uniform(-5, 5)
    VYinit = np.random.uniform(-10, -20)
    X0 = [((land[landing_site+1, 0] + land[landing_site, 0]) // 2)+hoffset, 3000]
    V0 = [VXinit, VYinit]
    
    best_autopilot = pid_autopilot
    result = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000,
                      autopilot=best_autopilot, fuel=500, parameters=parameters)
    Xs, Vs, As, thrust, fuels, errory, errorx, success = result
    landtarget = ((land[landing_site+1, 0] + land[landing_site, 0]) // 2)
    hdifffinal = Xs[-1][0]-landtarget

    randomPIDresult.append(["wind", wind, "hoffset", hoffset, "V0", V0, "Score", score(result, land, landing_site), 
                            "Fuel Remaining", fuels[-1], "Distance to Target", hdifffinal, "Final Velocity", 
                            Vs[-1][0], Vs[-1][1], "Success", int(success)])

randomPIDresult = sorted(randomPIDresult, key=lambda x: x[7])

with open('randomseedPID.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in randomPIDresult:
        writer.writerow([str(row[1]), str(row[3]), str(row[5]), str(row[7]), str(row[9]), str(row[11]), 
                         str(row[13]), row[14], row[16]])

trial_time = time.clock() - start_time

print("It took " + f'{trial_time:.3f}' + " seconds to test " +
      str(ntest) + " PID autopilot trials.")

with open('randomseedPID.csv') as csvDataFile:
    succdata = list(csv.reader(csvDataFile))

testlength = len(succdata)
succ = 0
for i in range(testlength):
    succ += int(succdata[i][8])
print(str(succ) + " successful landings out of " + str(testlength) + " random seeds in PID testing.")
print("\n")

Xz = [Xs]
thrustz = [thrust]
plot_lander(land, landing_site, Xz, thrustz, animate=True, step=10)