from MarsLanderPython import *

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
    result = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000, print_interval=10000000,
                      autopilot=proportional_autopilot, fuel=500, parameters=parameters)
    # add final positions, velocities and fuel load
    results.append([parameters, score(result)])
    Xs, Vs, As, thrust, fuels, success = result
    # results 2 is pretty printing
    resultsP.append(["K_h", Trial[0], "K_p",
                     Trial[1], "Score", score(result), "Fuel remaining", fuels[-1]])

results = sorted(results, key=lambda x: x[1])
resultsP = sorted(resultsP, key=lambda x: x[5])

with open('Trial Results P.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in results:
        writer.writerow([json.dumps(row[0]), str(row[1])])

with open('Trial Results Praw.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in resultsP:
        writer.writerow([str(row[1]), str(row[3]), str(row[5]), str(row[7])])

print("The top 5 tuning combinations tested for the proportional autopilot are:")
print()
top_fiveP = resultsP[:5]
for i in top_fiveP:
    i[1] = '{:.3f}'.format(round(i[1], 3))
    i[3] = '{:.3f}'.format(round(i[3], 3))
    i[5] = '{:.3f}'.format(round(i[5], 3))
    i[7] = '{:.3f}'.format(round(i[7], 3))

pp = pprint.PrettyPrinter(width=120)
pp.pprint(top_fiveP)
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
    result = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000, print_interval=10000000,
                      autopilot=pi_autopilot, fuel=500, parameters=parameters)
    # add final positions, velocities and fuel load
    results.append([parameters, score(result)])
    Xs, Vs, As, thrust, fuels, success = result
    resultsPI.append(["K_h", Trial[0], "K_p",
                      Trial[1], "K_i", Trial[2], "Score", score(result), "Fuel remaining", fuels[-1]])

results = sorted(results, key=lambda x: x[1])
resultsPI = sorted(resultsPI, key=lambda x: x[7])


with open('Trial Results PI.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in results:
        writer.writerow([json.dumps(row[0]), str(row[1])])

with open('Trial Results PIraw.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in resultsPI:
        writer.writerow([str(row[1]), str(row[3]), str(
            row[5]), str(row[7]), str(row[9])])

print("The top 5 tuning combinations tested for the PI autopilot are:")
print()
top_fivePI = resultsPI[:5]
for i in top_fivePI:
    i[1] = '{:.3f}'.format(round(i[1], 3))
    i[3] = '{:.3f}'.format(round(i[3], 3))
    i[5] = '{:.3f}'.format(round(i[5], 3))
    i[7] = '{:.3f}'.format(round(i[7], 3))
    i[9] = '{:.3f}'.format(round(i[9], 3))

pp = pprint.PrettyPrinter(width=120)
pp.pprint(top_fivePI)
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
    result = simulate(X0, V0, land, landing_site, dt=0.1, Nstep=2000, print_interval=10000000,
                      autopilot=pid_autopilot, fuel=500, parameters=parameters)
    # add final positions, velocities and fuel load
    results.append([parameters, score(result)])
    Xs, Vs, As, thrust, fuels, success = result
    resultsPID.append(["K_h", Trial[0], "K_p",
                       Trial[1], "K_i", Trial[2], "K_d", Trial[3], "Score", score(result), "Fuel remaining", fuels[-1]])

results = sorted(results, key=lambda x: x[1])
resultsPID = sorted(resultsPID, key=lambda x: x[9])

# TODO: Write to csv with parameters in a box as a float, retrieve best result for plotting
with open('Trial Results PID.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in results:
        writer.writerow([json.dumps(row[0]), str(row[1])])

with open('Trial Results PIDraw.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in resultsPID:
        writer.writerow([str(row[1]), str(row[3]), str(
            row[5]), str(row[7]), str(row[9]), str(row[11])])

print("The top 5 tuning combinations tested for the PID autopilot are:")
print()
top_fivePID = resultsPID[:5]
for i in top_fivePID:
    i[1] = '{:.3f}'.format(round(i[1], 3))
    i[3] = '{:.3f}'.format(round(i[3], 3))
    i[5] = '{:.3f}'.format(round(i[5], 3))
    i[7] = '{:.3f}'.format(round(i[7], 3))
    i[9] = '{:.3f}'.format(round(i[9], 3))
    i[11] = '{:.3f}'.format(round(i[11], 3))

pp = pprint.PrettyPrinter(width=120)
pp.pprint(top_fivePID)
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