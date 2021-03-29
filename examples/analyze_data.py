
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# reading the data from the file

X = []
Y = []
Z = []
X_TAR = []
Y_TAR = []
Z_TAR = []
VX = []
VY = []
VZ = []
VX_TAR = []
VY_TAR = []
VZ_TAR = []
ROLL = []
PITCH = []
YAW = []
ROLL_TAR = []
PITCH_TAR = []
YAW_TAR = []

with open("crazyflie-lib-python/20210329-172523.txt") as f:
    for line in f:
        values = line.split()
        values.pop(0)
        if "pm.vbat" not in line:
            if "stateEstimate.x" in line:
                x = values[1].replace(',', '')
                y = values[3].replace(',', '')
                z = values[5].replace('}', '')
                X.append(float(x))
                Y.append(float(y))
                Z.append(float(z))

            if "stateEstimate.vx" in line:
                vx = values[1].replace(',', '')
                vy = values[3].replace(',', '')
                vz = values[5].replace('}', '')
                VX.append(float(vx))
                VY.append(float(vy))
                VZ.append(float(vz))

            if "stateEstimate.roll" in line:
                roll = values[1].replace(',', '')
                pitch = values[3].replace(',', '')
                yaw = values[5].replace('}', '')
                ROLL.append(float(roll))
                PITCH.append(float(pitch))
                YAW.append(float(yaw))


            if "controller.roll" in line:
                roll = values[1].replace(',', '')
                pitch = values[3].replace(',', '')
                yaw = values[5].replace('}', '')
                ROLL_TAR.append(float(roll))
                PITCH_TAR.append(float(pitch))
                YAW_TAR.append(float(yaw))

            if "posCtl.targetX" in line: #posCtl.targetX or ctrltarget.x
                x = values[1].replace(',', '')
                y = values[3].replace(',', '')
                z = values[5].replace('}', '')
                X_TAR.append(float(x))
                Y_TAR.append(float(y))
                Z_TAR.append(float(z))

            if "posCtl.targetVX" in line: #posCtl.targetX or ctrltarget.x
                vx = values[1].replace(',', '')
                vy = values[3].replace(',', '')
                vz = values[5].replace('}', '')
                VX_TAR.append(float(vx))
                VY_TAR.append(float(vy))
                VZ_TAR.append(float(vz))

'''VX = []
VX_TAR = []
for i in range(1, len(X)):
    vx = (X[i] - X[i-1])
    vx_tar = (X_TAR[i]-X_TAR[i-1])
    VX.append(vx)
    VX_TAR.append(vx_tar)
    
window_size = 30
i = 0
moving_averages = []
moving_averages2 = []

while i < len(VX) - window_size + 1:
    this_window = VX[i: i + window_size]
    window_average = sum(this_window) / window_size
    moving_averages.append(window_average)
    this_window2 = VX_TAR[i: i + window_size]
    window_average2 = sum(this_window2) / window_size
    moving_averages2.append(window_average2)
    i += 1 '''

fig, axs = plt.subplots(3, 3)
axs[0, 0].plot(X)
axs[0, 0].plot(X_TAR)
axs[0, 0].legend(["est", "cmd"])
axs[0, 0].set_title("Position X")
axs[1, 0].plot(Y)
axs[1, 0].plot(Y_TAR)
axs[1, 0].legend(["est", "cmd"])
axs[1, 0].set_title("Position Y")
axs[2, 0].plot(Z)
axs[2, 0].plot(Z_TAR)
axs[2, 0].legend(["est", "cmd"])
axs[2, 0].set_title("Position Z")
axs[0, 1].plot(VX)
axs[0, 1].plot(VX_TAR)
axs[0, 1].legend(["est", "cmd"])
axs[0, 1].set_title("Velocity X")
axs[1, 1].plot(VY)
axs[1, 1].plot(VY_TAR)
axs[1, 1].legend(["est", "cmd"])
axs[1, 1].set_title("Velocity Y")
axs[2, 1].plot(VZ)
axs[2, 1].plot(VZ_TAR)
axs[2, 1].legend(["est", "cmd"])
axs[2, 1].set_title("Velocity Z")
axs[0, 2].plot(PITCH)
axs[0, 2].plot(PITCH_TAR)
axs[0, 2].legend(["est", "cmd"])
axs[0, 2].set_title("Pitch")
axs[1, 2].plot(ROLL)
axs[1, 2].plot(ROLL_TAR)
axs[1, 2].legend(["est", "cmd"])
axs[1, 2].set_title("Roll")
axs[2, 2].plot(YAW)
axs[2, 2].plot(YAW_TAR)
axs[2, 2].legend(["est", "cmd"])
axs[2, 2].set_title("Yaw")

plt.show()













# %%
