import numpy as np
import control as ct
import matplotlib.pyplot as plt

# assign constants (g, ell) values
simLength = 60
omega = 1
damping = 0

kp = 1
ki = 0
kd = 0

def plotFunction(xData, yData, xLabelString, yLabelString):
    plt.figure()
    plt.plot(xData, yData, color='blue')
    plt.xlabel(xLabelString)
    plt.ylabel(yLabelString)
    plt.grid()
    plt.show()

plant = ct.tf([omega ** 2], [1, omega * damping, omega ** 2])
print(plant)

kpTerm = ct.tf([kp], [1])

kiTerm = ct.tf([ki], [1, 0])

kdTerm = ct.tf([kd, 0], [1])

pid = ct.parallel()

closedLoop = ct.feedback(plant)

timeVector = np.linspace(0, simLength, 5000)
timeOutput, posOutput = ct.step_response(closedLoop, timeVector)

plotFunction(timeOutput, posOutput, 'time', 'amplitude')
