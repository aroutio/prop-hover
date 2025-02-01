import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import minimize

# Load CSV time data
time_data = np.genfromtxt("V1.1_flight_2_time_data.csv")
height_data = np.genfromtxt("V1.1_flight_2_height_data.csv")

# Ensure valid time data
if not np.all(np.diff(time_data) > 0):
    raise ValueError("time_data must be strictly increasing")
if np.any(np.diff(time_data) < 1e-6):
    raise ValueError("Time steps in time_data are too small")

# Downsample time data for faster simulation
time_data = time_data[::10]
height_data = height_data[::10]

# Constants
g = 9.81
m_mach = 0.929  # kg
last_thrust = 0
desired_height = 0.3
Kp = 5
Ki = 0
Kd = 0

# PID controller variables
integral_error = 0
previous_error = 0
previous_time = 0

# Initial conditions: y = 0, y_dot = 0
y0 = 0.162
y_dot0 = 0

# Constrain function
def constrain(value, min_value, max_value):
    return max(min(value, max_value), min_value)

# PID controller
def pid_controller(current_height, current_time):
    global integral_error, previous_error, previous_time

    error = (desired_height - current_height) * 1000

    # Proportional term
    proportional = Kp * error

    # Integral term with anti-windup
    dt = current_time - previous_time if previous_time > 0 else 0
    integral_error = constrain(integral_error + error * dt, -1000, 1000)
    integral = Ki * integral_error

    # Derivative term with small epsilon to prevent division by zero
    derivative = Kd * (error - previous_error) / (dt + 1e-6)

    # Update previous values
    previous_error = error
    previous_time = current_time

    return proportional + integral + derivative

# ODE system
def hover_mach_ODE(t, z, c_damp, c_filter):
    global last_thrust

    control_input = pid_controller(z[0], t)
    control_constrained = constrain(control_input, 80, 100)

    new_thrust = (10.67 * control_constrained) / 1000
    low_pass_filter_thrust = c_filter * new_thrust + (1 - c_filter) * last_thrust
    last_thrust = low_pass_filter_thrust

    return (
        z[1],
        (((low_pass_filter_thrust * g)) / m_mach) - g - ((c_damp * z[1]) / m_mach),
    )

# Cost function
def cost_function(params):
    c_damp, c_filter = params

    solution = solve_ivp(
        hover_mach_ODE,
        [time_data[0], time_data[-1]],
        (y0, y_dot0),
        t_eval=time_data,
        args=(c_damp, c_filter),
        method="RK45",
        atol=1e-8,
        rtol=1e-6,
    )
    if not solution.success:
        print(f"Solver failed for params: {params}")
        return 1e6  # High penalty for solver failures

    sim_height = solution.y[0] if len(solution.y) > 0 else np.zeros_like(height_data)
    return np.sum((sim_height - height_data) ** 2)

# Initial guess for parameters
initial_guess = [0.3, 0.4]

# Optimization
result = minimize(
    cost_function,
    initial_guess,
    method="Nelder-Mead",
    options={"maxiter": 200, "disp": True},  # Limit iterations and display progress
)

# Output results
best_c_damp, best_c_filter = result.x
print("Optimization complete:")
print("Best c_damp:", best_c_damp)
print("Best c_filter:", best_c_filter)
