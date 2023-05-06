from pathlib import Path
import serial
import sys
import csv
import os
import time
import matplotlib as mp
import matplotlib.pyplot as plt
import numpy as np
import socket

mp.use("tkAgg")

uiSocket = None
while (uiSocket is None):
    try:
        uiSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        uiSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        uiSocket.bind(('localhost',19200))
        uiSocket.settimeout(1) #wait only 1ms
        uiSocket.listen()
        conn, addr =  uiSocket.accept()
    except:
        uiSocket = None
        print("Could not connect to socket, retrying...")

# Setting up socket
HOST = "localhost"
PORT = 19200


# A name for the csv file created/updated
csv_name = "robertShaw_1.csv"

# Setting up csv path
path = Path(os.getcwd())
print(path.parent.absolute())
csvs_path = f"{path.parent.absolute()}/csvs/"
filename = f"{csvs_path}{csv_name}"

raw_data = False #set to true to write raw data instead of l/min and psi

if not (os.path.exists(filename)):
    print("Creating file...")
    with open(filename,"a") as f:
        writer = csv.writer(f,delimiter=",")
        writer.writerow(["Time", "Flow sensor", "Pressure sensor"])

print(len(sys.argv))
device =  "/dev/ttyACM0" if len(sys.argv) < 2 else sys.argv[1]
baud_rate = 115200 if len(sys.argv) < 3 else sys.argv[2]

print(f"device : {device}, baud_rate: {baud_rate}")

ser = None
while (ser is None):
    try:
        ser = serial.Serial(device, baud_rate, timeout=1)
    except:
        ser = None
        print("Waiting for connection...")
        time.sleep(1)

ser.flushInput()

# Set up according to your electronics

counter = 0

max_freq = 100
max_v = 3.3

win = 20
y = np.array(np.zeros([win]))
y2 = np.array(np.zeros([win]))
plt.ion()
fig, ax = plt.subplots()
pline, = ax.plot(y, c = 'r')
pline2, = ax.plot(y2, c = 'g')
plt.ylim(0, max_v)

# State Machine
IDLE_STATE = "idle"
ON_STATE = "on"
curr_state = IDLE_STATE
selection = "A" # Ask for both data by default

curr_state = IDLE_STATE
print("Currently on IDLE")

# Frequency of readings
waitTime = 0.1 #in seconds
prevTime = time.time()

while True:

    # Reading if new info is available on socket
    uiSocket.settimeout(0.001)
    conn.settimeout(0.001)
    try:
        data = conn.recv(1024)
        if not data:
            break

        # Process the data received from the server here
        data = data.decode('utf-8')
        if data in ['A', 'F', 'P']:
            selection = data
            curr_state = ON_STATE
        elif data == 'O':
            curr_state = ON_STATE
        elif data == 'S':
            print("Currently on IDLE")
            curr_state = IDLE_STATE
        else:
            print(f"Received {data}, INVALID")

    except socket.timeout:
        pass # No data received within the timeout period, continue operation

    # Main routine
    try:
        if ((time.time() - prevTime) > waitTime) and curr_state==ON_STATE:
            prevTime = time.time()
            if counter>10000:
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
                #pressure_sensor = pressure_sensor * 5/3.3
                #Formula: psi = 25V - 12.5
                psi = 25 * pressure_sensor - 12.5

                with open(filename,"a") as f:
                    writer = csv.writer(f,delimiter=",")
                    if raw_data:
                        writer.writerow([time.time(),round(float(flow_sensor),2), round(float(pressure_sensor),2)])
                    else:
                        writer.writerow([time.time(),round(float(l_min),2), round(float(psi),2)])

                counter+=1
        else:
            # Either timer has not passed or state is IDLE
            pass


    except serial.SerialException:
        ser.close()
        print("")
        print("Connection closed")
        break

    except KeyboardInterrupt:
        ser.close()
        print("")
        print("Connection closed")
        break

    except Exception as e:
        print(e)
        ser.close()
        print("Connection closed")
        break
