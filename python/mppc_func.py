import numpy as np
import do_mpc as mppc
from casadi import *

def mfd(n):

    G = (1.4877*(10**(-7))*(n**3) - 2.9815*(10**(-3))*(n**2) + 15.0912*n)/60

    return G

def mppc_model():

    model_type = 'discrete'
    model = mppc.model.Model(model_type)
    n = model.set_variable(var_type = '_x', var_name = 'n', shape = (4, 1))
    u = model.set_variable(var_type = '_u', var_name = 'u', shape = (2, 1))
    q = model.set_variable(var_type = '_tvp', var_name = 'q', shape = (4, 1))

    G = vertcat(mfd(n[0] + n[1]), mfd(n[2] + n[3]))

    model.set_expression(expr_name='TTS', expr=(sum1(n)*60))
    model.set_expression(expr_name='G(t)', expr=G)
    model.set_expression(expr_name='q(t)', expr=q)

    n_next = vertcat(
        q[0] + u[1]*(n[2]/(n[2] + n[3]))*mfd(n[2] + n[3]) - (n[0]/(n[0] + n[1]))*mfd(n[0] + n[1]), 
        q[1] - u[0]*(n[1]/(n[0] + n[1]))*mfd(n[0] + n[1]),
        q[2] - u[1]*(n[2]/(n[2] + n[3]))*mfd(n[2] + n[3]),
        q[3] + u[0]*(n[1]/(n[0] + n[1]))*mfd(n[0] + n[1]) - (n[3]/(n[2] + n[3]))*mfd(n[2] + n[3])
    )

    model.set_rhs('n', n + n_next)

    model.setup()

    return model

def mppc_mpc(model):
    mpc = mppc.controller.MPC(model)

    params = {
        'n_horizon': 20,
        't_step': 60,
        'use_terminal_bounds': False,
        'store_full_solution': True
    }

    mpc.set_param(**params)

    mpc.set_objective(mterm = model.aux['TTS'], lterm = model.aux['TTS'])
    mpc.set_rterm(u = 0.1)

    mpc.bounds['lower', '_x', 'n'] = 0
    mpc.bounds['upper', '_x', 'n'] = 10000
    mpc.bounds['lower', '_u', 'u'] = 0.1
    mpc.bounds['upper', '_u', 'u'] = 0.9

    demand = np.zeros((4, 140))

    demand[0, 0:5] = 0.25
    demand[0, 5:22] = 0.25 + (0.65/17)*(np.array(range(5, 22)) - 5)
    demand[0, 22:37] = 0.9
    demand[0, 37:55] = 0.9 - (0.65/18)*(np.array(range(37, 55)) - 37)
    demand[0, 55:140] = 0.25

    demand[1, 0:3] = 0.25 + (3/3)*np.array(range(0, 3))
    demand[1, 3:50] = 3.25
    demand[1, 50:60] = 3.25 - (3/10)*(np.array(range(50, 60)) - 50)
    demand[1, 60:140] = 0.25

    demand[2, 0:5] = 0.25
    demand[2, 5:30] = 0.25 + (0.95/25)*(np.array(range(5, 30)) - 5)
    demand[2, 30:55] = 1.2
    demand[2, 55:60] = 1.2 - (0.95/5)*(np.array(range(55, 60)) - 55)
    demand[2, 60:140] = 0.25

    demand[3, 0:3] = 0.25
    demand[3, 3:15] = 0.25 + (1.25/12)*(np.array(range(3, 15)) - 3)
    demand[3, 15:45] = 1.5
    demand[3, 45:57] = 1.5 - (1.25/12)*(np.array(range(45, 57)) - 45)
    demand[3, 57:140] = 0.25

    demand = demand*params['t_step']

    tvp_template = mpc.get_tvp_template()

    def tvp_fun(t_ind):
        k = int(t_ind) // params['t_step']
        for i in range(params['n_horizon'] + 1):
            tvp_template['_tvp', i, 'q'] = demand[:, k + i]

        return tvp_template

    mpc.set_tvp_fun(tvp_fun)

    mpc.setup()

    return mpc

def mppc_simulator(model):

    simulator = mppc.simulator.Simulator(model)

    t_step = 60

    simulator.set_param(t_step = t_step)

    demand = np.zeros((4, 140))

    demand[0, 0:5] = 0.25
    demand[0, 5:22] = 0.25 + (0.65/17)*(np.array(range(5, 22)) - 5)
    demand[0, 22:37] = 0.9
    demand[0, 37:55] = 0.9 - (0.65/18)*(np.array(range(37, 55)) - 37)
    demand[0, 55:140] = 0.25

    demand[1, 0:3] = 0.25 + (3/3)*np.array(range(0, 3))
    demand[1, 3:50] = 3.25
    demand[1, 50:60] = 3.25 - (1/10)*(np.array(range(50, 60)) - 50)
    demand[1, 60:140] = 0.25

    demand[2, 0:5] = 0.25
    demand[2, 5:30] = 0.25 + (0.95/25)*(np.array(range(5, 30)) - 5)
    demand[2, 30:55] = 1.2
    demand[2, 55:60] = 1.2 - (0.95/5)*(np.array(range(55, 60)) - 55)
    demand[2, 60:140] = 0.25

    demand[3, 0:3] = 0.25
    demand[3, 3:15] = 0.25 + (1.25/12)*(np.array(range(3, 15)) - 3)
    demand[3, 15:45] = 1.5
    demand[3, 45:57] = 1.5 - (1.25/12)*(np.array(range(45, 57)) - 45)
    demand[3, 57:140] = 0.25

    demand = demand*t_step

    tvp_template = simulator.get_tvp_template()

    def tvp_fun(t_ind):
        k = int(t_ind) // t_step
        tvp_template['q'] = demand[:, k]
        return tvp_template

    simulator.set_tvp_fun(tvp_fun)

    simulator.setup()

    return simulator