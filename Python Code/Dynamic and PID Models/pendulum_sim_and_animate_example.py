import numpy as np
from scipy.integrate import solve_ivp

# matplotlib imports
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation

# assign constants (g, ell) values
g = 9.81
l = 1
sim_length = 60 # seconds
Cd = 0.5 # resonable drag coefficient assumption for sphere
rho_steel = 7850
rho_air = 1.195
r_sphere = 0.005
m_sphere = rho_steel * (4 / 3) * np.pi * (r_sphere ** 3)

# initial conditions: theta = 30, velocity = 0
theta0 = np.deg2rad(90)
theta_dot0 = 0

# our system of differential equations
# y[0] is theta, y[1] is theta_dot
def pendulum_ODE(t, y):
    return (y[1], (- g * np.sin(y[0]) / l) + (((0.5 * l * Cd * rho_air * np.pi * (r_sphere ** 2) * (y[1] ** 2)) / m_sphere) * np.cos(y[0]) * - np.sign(y[1])))

# solve the ODE, 30 fps
sol = solve_ivp(pendulum_ODE, [0, sim_length], (theta0, theta_dot0), t_eval=np.linspace(0, sim_length, 30 * sim_length))

# output of the solver
theta = sol.y[0]
theta_dot = sol.y[1]
t = sol.t

# convert from radians to degrees
theta_deg = np.rad2deg(sol.y[0])
theta_dot_deg = np.rad2deg(sol.y[1])

'''
# create a plot of theta and theta_dot vs time
plt.plot(t, theta_deg, 'r', lw = 2, label = r'$\theta$')
plt.plot(t, theta_dot_deg, 'b', lw = 2, label = r'$\dot \theta$')
plt.show()
'''

# create an animation of the pendulum swinging with matplotlib
def pend_pos(theta):
    return (l * np.sin(theta), -l * np.cos(theta))

# create figure
fig = plt.figure()
ax = fig.add_subplot(aspect='equal')
ax.set_xlim(-1, 1)
ax.set_ylim(-1.25, 0.25)
ax.grid()

# draw pendulum
x0, y0 = pend_pos(theta0)
line, = ax.plot([0, x0], [0, y0], lw=2, c='k')
circle = ax.add_patch(plt.Circle(pend_pos(theta0), 0.05, fc='r', zorder=3))

#animate each frame "i"
def animate(i):
    x, y = pend_pos(theta[i])
    line.set_data([0, x], [0, y])
    circle.set_center((x, y))

# save video: 30 fps
ani = animation.FuncAnimation(fig, animate, frames=len(t))
ffmpeg_writer = animation.FFMpegWriter(fps=30)
ani.save(filename='F:\Propulsive Lander\Python Code\pendulum.mp4', writer=ffmpeg_writer)
# plt.show()