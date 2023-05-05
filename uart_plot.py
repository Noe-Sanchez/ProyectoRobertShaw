import serial
import sys
import csv
import os
import time
import matplotlib as mp
import matplotlib.pyplot as plt
import numpy as np

mp.use("tkAgg")

filename = "allTest.csv"
raw_data = False #set to true to write raw data instead of l/min and psi

if not (os.path.exists(filename)):
    print("Creating file...")
    with open(filename,"a") as f:
        writer = csv.writer(f,delimiter=",")
        writer.writerow(["Time", "Flow sensor", "Pressure sensor"])

ser = serial.Serial(sys.argv[1], sys.argv[2], timeout=1)
ser.flushInput()

waitTime = 0.1 #in seconds
counter = 0

max_freq = 100
max_v = 3.3

win = 20
y = np.array(np.zeros([win]))
y2 = np.array(np.zeros([win]))
plt.ion()
fig, ax = plt.subplots()
pline, = ax.plot(y, c = 'r')
pline2, = ax.plot(y, c = 'g')
plt.ylim(0, max_v)

prevTime = time.time()

IDLE_STATE = "idle"
ON_STATE = "on"
curr_state = IDLE_STATE
selection = "A" # Ask for both data by default

selection = input("Enter A, F or P: ")
curr_state = ON_STATE

while True:

    try:
        if ((time.time() - prevTime) > waitTime) and curr_state==ON_STATE:
            prevTime = time.time()
            if counter>300:
                pass
            else:
                # Send signal to arduino
                ser.write(selection.encode('utf-8'))
                # Wait for arduino to respond
                #line = input("data: ")
                print("Asked data...")
                line = ser.readline().decode('utf-8').rstrip()
                lineArray = line.split()
                flow_sensor = float(lineArray[0])
                flow_sensor_plot = flow_sensor * max_v / max_freq # Frequency to 3.3 as max
                pressure_sensor = float(lineArray[1])
                print(f"Sensor 1: {flow_sensor}, Sensor 2: {pressure_sensor}")
                # Print response
                print(line)

                y = np.append(y, flow_sensor_plot)
                y = y[1:win+1]
                y2 = np.append(y2, pressure_sensor)
                y2 = y2[1:win+1]
                pline.set_ydata(y)
                pline2.set_ydata(y2)
                ax.relim()
                ax.autoscale_view()
                fig.canvas.draw()
                fig.canvas.flush_events()

                # Transforming hz to L/min and V to psi
                # 16hz = 2L/min, 32.5hz = 4L/min, 49.3hz = 6L/min, 65.5 = 8L/min, 82 = 10L/min
                # Formula: L/min = 0.12121hz + 0.05349
                l_min = 0.12121 * flow_sensor + 0.05349

                # Transforming V to PSI
                # 0.5V = 0psi; 2.5V = 50psi, 4.5V = 100psi
                # convert from 3.3V to 5V (if using voltage divider)
                pressure_sensor = pressure_sensor * 5/3.3
                #Formula: psi = 25V - 12.5
                psi = 25 * pressure_sensor - 12.5

                with open(filename,"a") as f:
                    writer = csv.writer(f,delimiter=",")
                    if raw_data:
                        writer.writerow([time.time(),round(float(flow_sensor),2), float(pressure_sensor)])
                    else:
                        writer.writerow([time.time(),round(float(l_min),2), float(psi)])

                counter+=1


    except serial.SerialException:
        ser.close()
        print("")
        print("Conn closed")
        break

    except KeyboardInterrupt:
        ser.close()
        print("")
        print("Con closed")
        break
