import numpy as np
import matplotlib.pyplot as plt
import do_mpc as mppc
import os
import imageio
from casadi import *
from mppc_func import *

show_animation = True
store_results = True

model = mppc_model()
mpc = mppc_mpc(model)
simulator = mppc_simulator(model)

x0 = np.array([2700, 2700, 2000, 2000])

mpc.x0 = x0

simulator.x0 = x0

mpc.set_initial_guess()

fig, ax, graphics = mppc.graphics.default_plot(mpc.data)
plt.ion()

imgnames = []

for i in range(120):
    u0 = mpc.make_step(x0)
    x0 = simulator.make_step(u0)
    
    if show_animation:
        graphics.plot_results(t_ind= i)
        graphics.plot_predictions(t_ind= i)
        graphics.reset_axes()
        imgname = f'{i}.png'
        imgnames.append(imgname)
        plt.show()
        plt.savefig(imgname)
        plt.pause(0.01)

with imageio.get_writer('mpc_results.gif', mode='I') as writer:
    for imgname in imgnames:
        image = imageio.imread(imgname)
        writer.append_data(image)

for imgname in set(imgnames):
    os.remove(imgname)
    
input('Press any key to exit.')

if store_results:
    mppc.data.save_results([mpc, simulator], 'mppc')