import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import minimize

# load csv time data
time_data = np.genfromtxt("V1.1_flight_2_time_data.csv")
height_data = np.genfromtxt("V1.1_flight_2_height_data.csv")

# assign constants values
c_damp = 0.3 # minimized value
c_filter = 0.4
g = 9.81
m_mach = 0.929 # kg
last_thrust = 0
desired_height = 0.3
Kp = 5
Ki = 0
Kd = 0

# PID contoller variable assignments
integral_error = 0
previous_error = 0
previous_time = 0

# initial conditions: y = 0, y_dot = 0
y0 = 0.162
y_dot0 = 0

# list definitions
thrust_data = []

# Constrian values function definition
def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)

# PID controller function definition
def pid_controller(current_height, current_time):
    global integral_error
    global previous_error
    global previous_time

    # Error
    error = (desired_height - current_height) * 1000

    # Proportional term
    proportional = Kp * error

    # Integral term
    dt = current_time - previous_time if previous_time > 0 else 0
    integral_error += error * dt
    integral = Ki * integral_error

    # Derivative term
    derivative = Kd * (error - previous_error) / dt if dt > 0 else 0
 
    # PID output
    output = proportional + integral + derivative

    # Update previous values
    previous_error = error
    previous_time = current_time

    return output

# our system of differential equations
# z[0] is y, z[1] is y_dot
def hover_mach_ODE(t, z):
    global thrust_data, last_thrust

    control_input = pid_controller(z[0], t)

    control_constrained = constrain(control_input, 80, 100)

    new_thrust = (10.67 * control_constrained) / 1000

    low_pass_filter_thrust = c_filter * new_thrust + (1 - c_filter) * last_thrust

    last_thrust = low_pass_filter_thrust

    thrust_data.append(control_constrained)

    return (z[1], (((low_pass_filter_thrust * g)) / m_mach) - g - ((c_damp * z[1]) / m_mach))

def mse_calc(height_flight_data):
    solution = solve_ivp(hover_mach_ODE, [time_data[0], time_data[-1]], (y0, y_dot0), t_eval=time_data)
    sim_height = solution.y[0]
    return np.mean((sim_height - height_flight_data) ** 2) # mean of squared error

def r_squared(height_flight_data):
    flight_data_mean = np.mean(height_flight_data)
    solution = solve_ivp(hover_mach_ODE, [time_data[0], time_data[-1]], (y0, y_dot0), t_eval=time_data)
    sim_height = solution.y[0]
    ss_res = np.sum((height_flight_data - sim_height) ** 2)
    ss_tot = np.sum((height_flight_data - flight_data_mean) ** 2)
    return 1 - (ss_res / ss_tot)

mse = mse_calc(height_data)
r2 = r_squared(height_data)

print(mse)
print(r2)
