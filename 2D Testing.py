from MarsLanderPython import *

# Result storage variables
results = []
resultsP = []
resultsPI = []
resultsPID = []


# Generating parameter range (a, b...) and number of equally spaced values (c) within the range 
K_diffxlist = list(np.linspace(0.001, 0.050, 8))
K_pxlist = list(np.linspace(0.000, 0.500, 6))
K_ixlist = list(np.linspace(0.000, 0.005, 6))
K_dxlist = list(np.linspace(0.000, 0.500, 6))


# Automated Testing (P Vertical)

# Extract 1D P test results
with open('Trial Results Praw.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

K_h = float(data[0][0])
K_p = float(data[0][1])


print("Initialising 2D proportional autopilot testing:")
print()
print("The ideal tuning parameters from 1D test used in this test are: K_h: " + '{:.3f}'.format(round(K_h, 3)) + ", " + "K_p: " + '{:.3f}'.format(round(K_p, 3)))
print()
start_time = time.clock()

# Number of parameter combinations to be tested
Trial_combinations = np.size(K_diffxlist) * np.size(K_pxlist)

# Every combination of each parameter
Trials = itertools.product(K_diffxlist, K_pxlist)

for Trial in Trials:
    parameters = {
        'K_h': K_h,
        'K_p': K_p,
        'K_diffx': Trial[0],
        'K_px': Trial[1]
    }
    result = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000, print_interval=10000000,
                      autopilot=proportional_autopilot, fuel=500, parameters=parameters)
    results.append([parameters, score(result)])

    # resultsP is for pretty printing
    Xs, Vs, As, thrust, fuels, errory, errorx, success = result
    resultsP.append(["K_diffx", Trial[0], "K_px", Trial[1], "Score", score(result), "Fuel remaining", fuels[-1], "Final Velocity", Vs[-1]])

# Sorting results by score
results = sorted(results, key=lambda x: x[1])
resultsP = sorted(resultsP, key=lambda x: x[5])

# Writing results to CSV with the parameters explicitly stated
with open('2D Trial Results P.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in results:
        writer.writerow([json.dumps(row[0]), str(row[1])])

# Writing results to CSV with just values of each parameter, the score, and fuel remaining for each trial
with open('2D Trial Results Praw.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in resultsP:
        writer.writerow([str(row[1]), str(row[3]), str(
            row[5]), str(row[7]), str(row[9])])

# Printing the top 5 results in the interactive window
print("The top 5 tuning combinations tested for the proportional autopilot are:")
print()
top_fiveP = resultsP[:5]
for i in top_fiveP:
    i[1] = '{:.3f}'.format(round(i[1], 3))
    i[3] = '{:.3f}'.format(round(i[3], 3))
    i[5] = '{:.3f}'.format(round(i[5], 3))
    i[7] = '{0:07.3f}'.format(round(i[7], 3))
    i[9] = str(np.round(i[9], 3))

pp = pprint.PrettyPrinter(width=200)
pp.pprint(top_fiveP)
print()

trial_time = time.clock() - start_time
print("It took " + f'{trial_time:.3f}' + " seconds to test " +
      str(Trial_combinations) + " Proportional autopilot trials.")
print("\n")


# Automated Testing (PI Vertical)

# Extract 1D PI test results
with open('Trial Results PIraw.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

K_h = float(data[0][0])
K_p = float(data[0][1])
K_i = float(data[0][2])


print("Initialising proportional-integral autopilot testing:")
print()
print("The ideal tuning parameters from 1D test used in this test are: K_h: " + '{:.3f}'.format(round(K_h, 3)) + ", " + "K_p: " + '{:.3f}'.format(round(K_p, 3)) + ", " + "K_i: " + '{:.3f}'.format(round(K_i, 3)))
print()

start_time = time.clock()

Trial_combinations = np.size(K_diffxlist) * np.size(K_pxlist) * np.size(K_ixlist)

Trials = itertools.product(K_diffxlist, K_pxlist, K_ixlist)

for Trial in Trials:
    parameters = {
        'K_h': K_h,
        'K_p': K_p,
        'K_i': K_i,
        'K_diffx': Trial[0],
        'K_px': Trial[1],
        'K_ix': Trial[2]
        }
    result = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000, print_interval=10000000,
                      autopilot=pi_autopilot, fuel=500, parameters=parameters)
    # add final positions, velocities and fuel load
    results.append([parameters, score(result)])
    Xs, Vs, As, thrust, fuels, errory, errorx, success = result
    resultsPI.append(["K_diffx", Trial[0], "K_px",
                      Trial[1], "K_ix", Trial[2], "Score", score(result), "Fuel remaining", fuels[-1], "Final Velocity", Vs[-1]])

results = sorted(results, key=lambda x: x[1])
resultsPI = sorted(resultsPI, key=lambda x: x[7])


with open('2D Trial Results PI.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in results:
        writer.writerow([json.dumps(row[0]), str(row[1])])

with open('2D Trial Results PIraw.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in resultsPI:
        writer.writerow([str(row[1]), str(row[3]), str(
            row[5]), str(row[7]), str(row[9]), str(row[11])])

print("The top 5 tuning combinations tested for the PI autopilot are:")
print()
top_fivePI = resultsPI[:5]
for i in top_fivePI:
    i[1] = '{:.3f}'.format(round(i[1], 3))
    i[3] = '{:.3f}'.format(round(i[3], 3))
    i[5] = '{:.3f}'.format(round(i[5], 3))
    i[7] = '{:.3f}'.format(round(i[7], 3))
    i[9] = '{0:07.3f}'.format(round(i[9], 3))
    i[11] = str(np.round(i[11], 3))

pp = pprint.PrettyPrinter(width=200)
pp.pprint(top_fivePI)
print()

trial_time = time.clock() - start_time
print("It took " + f'{trial_time:.3f}' + " seconds to test " +
      str(Trial_combinations) + " PI autopilot trials.")
print("\n")


# Automated Testing (PID Vertical)

# Extract 1D PI test results
with open('Trial Results PIDraw.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

K_h = float(data[0][0])
K_p = float(data[0][1])
K_i = float(data[0][2])
K_d = float(data[0][3])
print("Initialising proportional-integral-derivative autopilot testing:")
print()
print("The ideal tuning parameters from 1D test used in this test are: K_h: " + '{:.3f}'.format(round(K_h, 3)) + ", " + "K_p: " + '{:.3f}'.format(round(K_p, 3)) + ", " + "K_i: " + '{:.3f}'.format(round(K_i, 3)) + ", " + "K_d: " + '{:.3f}'.format(round(K_d, 3)))
print()

start_time = time.clock()

Trial_combinations = np.size(
    K_diffxlist) * np.size(K_pxlist) * np.size(K_ixlist) * np.size(K_dxlist)

Trials = itertools.product(K_diffxlist, K_pxlist, K_ixlist, K_dxlist)

for Trial in Trials:
    parameters = {
        'K_h': K_h,
        'K_p': K_p,
        'K_i': K_i,
        'K_d': K_d,
        'K_diffx': Trial[0],
        'K_px': Trial[1],
        'K_ix': Trial[2],
        'K_dx': Trial[3]
    }
    result = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000, print_interval=10000000,
                      autopilot=pid_autopilot, fuel=500, parameters=parameters)
    # add final positions, velocities and fuel load
    results.append([parameters, score(result)])
    Xs, Vs, As, thrust, fuels, errory, errorx, success = result
    resultsPID.append(["K_diffx", Trial[0], "K_px",
                       Trial[1], "K_ix", Trial[2], "K_dx", Trial[3], "Score", score(result), "Fuel remaining", fuels[-1], "Final Velocity", Vs[-1]])

results = sorted(results, key=lambda x: x[1])
resultsPID = sorted(resultsPID, key=lambda x: x[9])

# TODO: Write to csv with parameters in a box as a float, retrieve best result for plotting
with open('2D Trial Results PID.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in results:
        writer.writerow([json.dumps(row[0]), str(row[1])])

with open('2D Trial Results PIDraw.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in resultsPID:
        writer.writerow([str(row[1]), str(row[3]), str(
            row[5]), str(row[7]), str(row[9]), str(row[11]), str(row[13])])

print("The top 5 tuning combinations tested for the PID autopilot are:")
print()
top_fivePID = resultsPID[:5]
for i in top_fivePID:
    i[1] = '{:.3f}'.format(round(i[1], 3))
    i[3] = '{:.3f}'.format(round(i[3], 3))
    i[5] = '{:.3f}'.format(round(i[5], 3))
    i[7] = '{:.3f}'.format(round(i[7], 3))
    i[9] = '{:.3f}'.format(round(i[9], 3))
    i[11] = '{0:07.3f}'.format(round(i[11], 3))
    i[13] = str(np.round(i[13], 3))

pp = pprint.PrettyPrinter(width=200)
pp.pprint(top_fivePID)
print()
trial_time = time.clock() - start_time
print("It took " + f'{trial_time:.3f}' + " seconds to test " +
      str(Trial_combinations) + " PID autopilot trials.")
print("\n")
print("Testing complete!")
