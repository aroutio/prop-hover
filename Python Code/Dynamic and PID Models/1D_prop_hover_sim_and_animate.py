import numpy as np
from scipy.integrate import solve_ivp

# matplotlib imports
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# assign constants (g, ell) values
sim_length = 10
g = 9.81
m_mach = 0.735 # ~ 1 lb weight estimate for hover machine
thrust = 0.960 # kg thrust weight based on data sheet
Cd = 1.1 # estimate for flat thin disk from engineering tool box website
rho = 1.195 # kg / m^3 (air)
d_mach = 0.2 # m assuming diameter of hover machine to be equal to diameter of prop for now
cross_area = np.pi * ((d_mach / 2) ** 2)

# initial conditions: y = 0, y_dot = 0
y0 = 0
y_dot0 = 0

# our system of differential equations
# z[0] is y, z[1] is y_dot
def hover_mach_ODE(t, z):
    return (z[1], (((thrust * g)) / m_mach) - g - ((1 / (2 * m_mach)) * Cd * rho * cross_area * (z[1] ** 2)))

# solve the ODE, 30 fps
sol = solve_ivp(hover_mach_ODE, [0, sim_length], (y0, y_dot0), t_eval=np.linspace(0, sim_length, 30 * sim_length))

# output of the solver
y_sol = sol.y[0]
y_dot_sol = sol.y[1]
t = sol.t

# static plot of position and velocity
plt.plot(t, y_sol, 'r', lw = 2, label = r'$\theta$')
plt.plot(t, y_dot_sol, 'b', lw = 2, label = r'$\dot \theta$')
plt.show()

# create an animation of 1D hover machine with matplotlib
def hover_mach_pos(y):
    return 0, y

# create figure
fig = plt.figure()
ax = fig.add_subplot(aspect='equal')
ax.set_xlim(-2, 2)
ax.set_ylim(0, 50)
ax.grid()
plt.ylabel('Height [m]')

# draw circle
circle = ax.add_patch(plt.Circle(hover_mach_pos(y0), 0.25, fc='r', zorder=3))

#animate each frame "i"
def animate(i):
    x, y = hover_mach_pos(y_sol[i])
    circle.set_center((x, y))

# save video: 30 fps
ani = animation.FuncAnimation(fig, animate, frames=len(t))
ffmpeg_writer = animation.FFMpegWriter(fps=30)
ani.save(filename='F:\Projects\Propulsive Lander\Python Code\OneD_hover_mach.mp4', writer=ffmpeg_writer)
