import numpy as np
import pandas as pd
import matplotlib . pyplot as plt



# IMU 4 used in class
# DATA_2024-03-18_10-50-28.csv   newer one
# DATA_2024-03-25_11-03-58.csv task 7 data
data = pd.read_csv("DATA_2024-03-11_10-45-36.csv", delimiter=';')
roll = data[['Roll']].values
pitch = data[['Pitch']].values
yaw = data[['Yaw']].values
gyro_x = data[['Gyro_x']].values
gyro_y = data[['Gyro_y']].values

time = np.arange(len(roll)) / 500

Ay = data[['Accel_y']].values
Ax = data[['Accel_x']].values
Az = data[['Accel_z']].values

dataN = pd.read_csv("DATA_2024-03-18_10-50-28.csv", delimiter=';')
bias = np.mean(dataN)
rollN = dataN[['Roll']].values
pitchN = dataN[['Pitch']].values
yawN = dataN[['Yaw']].values
gyro_xN = dataN[['Gyro_x']].values
gyro_yN = dataN[['Gyro_y']].values

gyro_x=gyro_x-np.mean(dataN[['Gyro_x']].values)
gyro_y=gyro_y-np.mean(dataN[['Gyro_y']].values)



fs=500
#∆x = 1/fs

calcroll=np.zeros(len(roll))
calcpitch=np.zeros(len(pitch))
for i in range (1,len(calcroll)):
    calcroll[i] = calcroll[i-1] - 1/(2*fs)*(gyro_x[i-1]+ gyro_x[i])

for i in range (1,len(calcpitch)):
    calcpitch[i] = calcpitch[i-1] + 1/(2*fs)*(gyro_y[i-1]+ gyro_y[i])

phyy = np.rad2deg(np.arctan(Ay/np.sqrt(Ax*Ax+Az*Az)))# for roll
teta = np.rad2deg(np.arctan(Ax/np.sqrt(Ay*Ay+Az*Az)))# for pitch
nuu = np.rad2deg(np.arctan(np.sqrt(Ax*Ax+Ay*Ay))/Az)


k=100

#Acci=Acci-k+Acci_k+1+...+Acci-1/k
accroll = np.zeros(len(Ax))
accpitch = np.zeros(len(Ay))
accyaw = np.zeros(len(Az))

for i in range(1, len(Ax)):
    if i < k:
        accroll[i] = np.mean(Ax[0:i])
    else:
        accroll[i] = np.mean(Ax[i-k:i])
for i in range(1, len(Ay)):
    if i < k:
        accpitch[i] = np.mean(Ay[0:i])
    else:
        accpitch[i] = np.mean(Ay[i-k:i])
for i in range(1, len(Az)):
    if i < k:
        accyaw[i] = np.mean(Az[0:i])
    else:
        accyaw[i] = np.mean(Az[i-k:i])


pyy1= np.rad2deg(np.arctan(accpitch/np.sqrt(accroll*accroll+accyaw*accyaw)))# for roll
teta1 = np.rad2deg(np.arctan(accroll/np.sqrt(accpitch*accpitch+accyaw*accyaw)))# for pitch
#nuu1 = np.rad2deg(np.arctan(np.sqrt(Ax*Ax+Ay*Ay))/Az)

fig, ax = plt .subplots(2, 1)
ax[0]. plot(time, roll)
ax[1]. plot(time, pitch)
#ax[0]. plot(time, calcroll)
#ax[1]. plot(time, calcpitch)
#ax[0]. plot(time, phyy)    # task5
#ax[1]. plot(time, teta)    # task5

ax[0]. plot(time,pyy1)    # task 6
ax[1]. plot(time,teta1)   # task6

#plt .show()

# task 7
if __name__=="__main__":
    data7 = pd.read_csv("DATA_2024-03-25_11-03-58.csv",delimiter=';')
    Acc_x7 = data7[['Accel_y']].values
    Acc_y7 = data7[['Accel_x']].values
    Acc_z7 = data7[['Accel_z']].values
    A=np.asarray([[0,0,1],
                  [0,0,-1],
                  [0,-1,0],
                  [0,1,0],
                  [-1,0,0],
                  [1,0,0]])
   # starts=[1000,4000,8500,13000,17000,22000]
   # ends=[2000,6000,10000,15000,20000,24000]
    starts = [1500, 6200, 12500, 17500, 20800, 24500]
    ends = [3500, 8000, 14000, 19000, 21600, 25800]

    A_raw=np.ones((6,4))
    for i in range(6):
        start=starts[i]
        end=ends[i]
        A_raw[i,0]=np.mean(Acc_x7[start:end])
        A_raw[i, 1] = np.mean(Acc_y7[start:end])
        A_raw[i, 2] = np.mean(Acc_z7[start:end])

    Y=np.linalg.lstsq(A_raw,A,rcond=None)
    a=Y[0][0:3]
    b=Y[0][3]

    fig, ax = plt .subplots(3, 1)
    ax[0]. plot(Acc_x7)
    ax[1]. plot(Acc_y7)
    ax[2]. plot(Acc_z7)

    plt .show()

# task 8
   # data = pd.read_csv("DATA_2024-03-11_10-45-36.csv", delimiter=';')
   # gyro_x = data[['Gyro_x']].values
   # gyro_y = data[['Gyro_y']].values

    # Accelerometer part begins here

    data_row=np.concatenate((Ax,Ay,Az),1)
    data8=data_row.dot(a)
    calc_acc_x = data8[:, 0] + b[0]
    calc_acc_y = data8[:, 1] + b[1]
    calc_acc_z = data8[:, 2] + b[2]

    accroll8 = np.zeros(len(Ax))
    accpitch8 = np.zeros(len(Ay))
    accyaw8 = np.zeros(len(Az))

    for i in range(1, len(Ax)):
        if i < k:
            accroll8[i-1] = np.mean(Ax[0:i])
        else:
            accroll8[i-1] = np.mean(Ax[i - k:i])
    for i in range(1, len(Ay)):
        if i < k:
            accpitch8[i-1] = np.mean(Ay[0:i])
        else:
            accpitch8[i-1] = np.mean(Ay[i - k:i])
    for i in range(1, len(Az)):
        if i < k:
            accyaw8[i-1] = np.mean(Az[0:i])
        else:
            accyaw8[i-1] = np.mean(Az[i - k:i])

    pyy8 = np.rad2deg(np.arctan(accpitch8 / np.sqrt(accroll8 * accroll8 + accyaw8 * accyaw8)))  # for roll
    teta8 = np.rad2deg(np.arctan(accroll8 / np.sqrt(accpitch8 * accpitch8 + accyaw8 * accyaw8)))  # for pitch
    beta8 = np.rad2deg(np.arctan(accyaw8 / np.sqrt(accpitch8 * accpitch8 + accroll8 * accroll8)))  # for yaw

    # Accelereometre data ends here

    gyro_x8 = data[['Gyro_x']].values
    gyro_y8 = data[['Gyro_y']].values
    gyro_z8 = data[['Gyro_z']].values

  #  dataN = pd.read_csv("DATA_2024-03-18_10-50-28.csv", delimiter=';')
    bias8 = np.mean(dataN)

    gyro_xN8 = dataN[['Gyro_x']].values
    gyro_yN8 = dataN[['Gyro_y']].values
    gyro_zN8 = dataN[['Gyro_z']].values

    gyro_x8 = gyro_x8 - np.mean(dataN[['Gyro_x']].values)
    gyro_y8 = gyro_y8 - np.mean(dataN[['Gyro_y']].values)
    gyro_z8 = gyro_z8 - np.mean(dataN[['Gyro_z']].values)


    calcroll8=np.zeros(len(roll))
    calcroll8[0]=pyy8[0]
    calcpitch8=np.zeros(len(pitch))
    calcpitch8[0]=teta8[0]
    calcyaw8 = np.zeros(len(yaw))
    calcyaw8[0] = teta8[0]
    for i in range (1,len(calcroll8)):
        calcroll8[i] = calcroll8[i-1] - 1/(2*fs)*(gyro_x8[i-1]+ gyro_x8[i])

    for i in range (1,len(calcpitch8)):
        calcpitch8[i] = calcpitch8[i-1] + 1/(2*fs)*(gyro_y8[i-1]+ gyro_y8[i])
    for i in range(1, len(calcyaw8)):
        calcyaw8[i] = calcyaw8[i - 1] + 1 / (2 * fs) * (gyro_z8[i - 1] + gyro_z8[i])

    phyy82 = np.rad2deg(np.arctan(Ay/np.sqrt(Ax*Ax+Az*Az)))# for roll
    teta82 = np.rad2deg(np.arctan(Ax/np.sqrt(Ay*Ay+Az*Az)))# for pitch
    nuu82 = np.rad2deg(np.arctan(np.sqrt(Ax*Ax+Ay*Ay))/Az) # for yaw

    # θ = µ · θG + (1 − µ) · θA

    #mu=0.1   # we can adjust this parameter
   
    #est_ang_rol=mu*calcroll8+(1-mu)*pyy8
    #est_ang_pit=mu*calcpitch8+(1-mu)*teta8
    #est_ang_yaw=mu*calcyaw8+(1-mu)*beta8

    # θ = µ · θG + (1 − µ) · θA

    mu = 0.1   # we can adjust this parameter

    est_ang_rol = np.zeros(len(Ax))
    est_ang_pit = np.zeros(len(Ay))
    est_ang_yaw = np.zeros(len(Az))

    for i in range(len(Ax)):
        # Perform complementary filter computation for roll
        est_ang_rol[i] = mu * calcroll8[i] + (1 - mu) * pyy8[i]

        # Perform complementary filter computation for pitch
        est_ang_pit[i] = mu * calcpitch8[i] + (1 - mu) * teta8[i]

        # Perform complementary filter computation for yaw
        est_ang_yaw[i] = mu * calcyaw8[i] + (1 - mu) * beta8[i]


    f=500
    dt=1/500
    time8=np.arange(len(Ax))/f
    fig,ax=plt.subplots(nrows=3,ncols=1)
    #ax[0].plot(Ax)
    #ax[0].plot(calc_acc_x)
    #ax[0].plot(pyy8)
    #ax[0].plot(calcroll8)
    ax[0].plot(est_ang_rol)

    #ax[1].plot(Ay)
    #ax[1].plot(calc_acc_y)
    #ax[1].plot(teta8)
    #ax[1].plot(calcpitch8)
    ax[1].plot(est_ang_pit)

    #ax[2].plot(Az)
    #ax[2].plot(calc_acc_z)
    #ax[2].plot(beta8)
    #ax[2].plot(calcyaw8)
    ax[2].plot(est_ang_yaw)


    plt.show()




