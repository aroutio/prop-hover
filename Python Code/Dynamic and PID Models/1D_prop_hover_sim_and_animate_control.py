import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import csv
import time

# assign constants values
sim_length = 10
g = 9.81
m_mach = 0.945 # kg
C_damp = 0
desired_height = 0.3

Kp = 1
Ki = 0
Kd = 1

'''
Kp = 0.6
Ki = 0.01
Kd = 2
'''

# PID contoller variable assignments
delta_time = 0.01
steps = int(sim_length / delta_time)

# initial conditions: y = 0, y_dot = 0
y0 = 0.162
y_dot0 = 0

# csv file name
csv_file_name = 'model_output.csv'

# list definitions
thrust_data = []
control_constrained_data = []

# Constrian values function definition
def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)

# PID controller function definition
def pid_controller(current_height, current_time):
    global delta_time

        # Static variables for storing previous states
    if not hasattr(pid_controller, "previous_time"):
        pid_controller.previous_time = time.time()
    if not hasattr(pid_controller, "previous_error"):
        pid_controller.previous_error = 0
    if not hasattr(pid_controller, "integral"):
        pid_controller.integral = 0

    # Current time and delta time
    current_time = time.time()
    elapsed_time = current_time - pid_controller.previous_time

    if elapsed_time >= delta_time:
        pid_controller.previous_time = current_time

        # PID calcs
    error = (desired_height - current_height) * 1000
    pid_controller.integral += error * delta_time
    derivative = (error - pid_controller.previous_error) / delta_time
    output = Kp * error + Ki * pid_controller.integral + Kd * derivative

    # Update previous values
    pid_controller.previous_error = error

    return output

# our system of differential equations
# z[0] is y, z[1] is y_dot
def hover_mach_ODE(t, z):
    global thrust_data

    control_input = pid_controller(z[0], t)

    control_constrained = constrain(control_input, 75, 100)

    term1 = 0.00000000806203918467946 * control_constrained ** 6 
    term2 = - 0.00000354481663517037 * control_constrained ** 5 
    term3 = 0.000508078928071143 * control_constrained ** 4
    term4 = - 0.0310657211248326 * control_constrained ** 3 
    term5 = 0.874155641722609 * control_constrained ** 2 
    term6 = - 0.380647144396789 * control_constrained 
    term7 = 0.572951247857418

    thrust = (term1 + term2 + term3 + term4 + term5 + term6 + term7) / 1000

    control_constrained_data.append(control_constrained)
    thrust_data.append(thrust)

    # Calculate forces and acceleration
    thrust_acc = (thrust * g) / m_mach
    gravity_acc = -g
    damping_acc = -((C_damp * z[1]) / m_mach)

    total_acc = thrust_acc + gravity_acc + damping_acc

    if z[0] < y0:
        z[0] = y0
        z[1] = y_dot0

    print(f"Time: {t:.2f}, Height: {z[0]:.4f}, Velocity: {z[1]:.4f}, Thrust: {thrust_acc}, Total Acc: {total_acc:.4f}")

    return (z[1], total_acc)

# solve the ODE
sol = solve_ivp(hover_mach_ODE, [0, sim_length], (y0, y_dot0), t_eval=np.linspace(0, sim_length, steps), method="RK45")

# output of the solver
y_sol = sol.y[0]
y_dot_sol = sol.y[1]
t = sol.t

# static plot of position and velocity
plt.plot(t, y_sol, 'r', lw = 2, label = r'$\theta$')
plt.plot(t, y_dot_sol, 'b', lw = 2, label = r'$\dot \theta$')
plt.show()

# write data to csv file
with open(csv_file_name, mode='w', newline='') as file:
    writer = csv.writer(file)

    # write header row
    writer.writerow(["Time [s]", "Height [m]", "Velocity [m/s]", "Thrust [kg]", "Throttle [%]"])

    # write each row of data
    for t, h, v, f, c in zip(t, y_sol, y_dot_sol, thrust_data, control_constrained_data):
        writer.writerow([t, h, v, f, c])

print("Model data has been saved to '{csv_file_name}'.")
