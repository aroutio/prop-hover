import numpy as np
import control as ct
import matplotlib.pyplot as plt

# assign constants (g, ell) values
simLength = 60
mass = 0.945 # ~ 1 lb weight estimate for hover machine
referenceHeight = 0.400 # m
throttle = 11.021
g = 9.81
c = 0.1
# tau = 0.001
# controlLoopTime = 0.05

kp = 0.4
ki = 0
kd = 0

def plotFunction(xData, yData, xLabelString, yLabelString):
    plt.figure()
    plt.plot(xData, yData, color='blue')
    plt.xlabel(xLabelString)
    plt.ylabel(yLabelString)
    plt.grid()
    plt.show()

thrust = ((throttle * g) / 1000)
verticalAccel = (thrust / mass)
vehicleDynamics = ct.tf([verticalAccel], [1, c, 0])
print(vehicleDynamics)

#motorDynamics = ct.tf([1], [tau / 3, 1])
#print(motorDynamics)

#distanceSensorDynamics = ct.tf([1], [1])
#print(distanceSensorDynamics)

proportionalTerm = ct.tf([kp], [1])
print(proportionalTerm)

integralTerm = ct.tf([ki], [1, 0])
print(integralTerm)

#TsTerm = controlLoopTime / 2
#derivativeTerm = ct.tf([kd * 1, 0], [TsTerm, 1])
derivativeTerm = ct.tf([kd, 0 ], [1])
print(derivativeTerm)

pidController = ct.parallel(proportionalTerm, integralTerm, derivativeTerm)
print(pidController)

openLoop = ct.series(pidController, vehicleDynamics)
print(openLoop)

closedLoop = ct.feedback(openLoop)
print(closedLoop)

timeVector = np.linspace(0, simLength, 5000)
timeOutput, posOutput = ct.step_response(referenceHeight * closedLoop, timeVector)

plotFunction(timeOutput, posOutput, 'time', 'amplitude')

#ct.root_locus(ct.series(motorDynamics, vehicleDynamics))
#plt.show()
