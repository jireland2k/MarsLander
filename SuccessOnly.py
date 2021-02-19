import csv

with open('2D Trial Results P.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

with open('2D Trial Results P Successful.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in data:
        if int(row[7]) == 1:
            writer.writerow([str(row[0]), str(row[1]), str(row[2]), str(row[3]), str(row[4]), str(row[5]), str(row[6]), str(row[7])])
        else:
            pass

with open('2D Trial Results PI.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

with open('2D Trial Results PI Successful.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in data:
        if int(row[8]) == 1:
            writer.writerow([str(row[0]), str(row[1]), str(row[2]), str(row[3]), str(row[4]), str(row[5]), str(row[6]), str(row[7]), str(row[8])])
        else:
            pass

with open('2D Trial Results PID.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

with open('2D Trial Results PID Successful.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in data:
        if int(row[9]) == 1:
            writer.writerow([str(row[0]), str(row[1]), str(row[2]), str(row[3]), str(row[4]), str(row[5]), str(row[6]), str(row[7]), str(row[8]), str(row[9])])
        else:
            pass

with open('randomseedP.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

with open('randomseedP Successful.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in data:
        if int(row[8]) == 1:
            writer.writerow([str(row[0]), str(row[1]), str(row[2]), str(row[3]), str(row[4]), str(row[5]), str(row[6]), str(row[7]), str(row[8])])
        else:
            pass

with open('randomseedPI.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

with open('randomseedPI Successful.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in data:
        if int(row[8]) == 1:
            writer.writerow([str(row[0]), str(row[1]), str(row[2]), str(row[3]), str(row[4]), str(row[5]), str(row[6]), str(row[7]), str(row[8])])
        else:
            pass

with open('randomseedPID.csv') as csvDataFile:
    data = list(csv.reader(csvDataFile))

with open('randomseedPID Successful.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for row in data:
        if int(row[8]) == 1:
            writer.writerow([str(row[0]), str(row[1]), str(row[2]), str(row[3]), str(row[4]), str(row[5]), str(row[6]), str(row[7]), str(row[8])])
        else:
            pass
print("done")