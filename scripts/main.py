import numpy as np
import matplotlib.pyplot as plt
from vehicle import Vehicle
from pathgen import PathGen
from optimize import MPC
from plotter import Plotter

def deg_to_rad(_deg):
    return np.pi*_deg/180

def rad_to_deg(_rad):
    return 180*_rad/np.pi

def beta_to_steer(_beta):
    steer = np.arctan((1.738*2)*np.tan(_beta)/(1.738))
    return steer

def steer_to_beta(_steer):
    return np.arctan(np.tan(_steer)*(1.738)/(1.738+1.738))

if __name__=='__main__':
    init = np.array([0,0])
    ref = np.array([0,0])
    des = np.array([50,40])

    #x0 = np.array([0,0,0,0.2462])
    beta_i = steer_to_beta(deg_to_rad(45))
    x0 = np.array([0,0,2.0,beta_i])
    xf = np.array([5,4])
    Ts = 0.05
    lr = 1.738
    vehicle = Vehicle(Ts, lr, lr, x0) 
    obs = np.array([3,3])
    steer_lim = deg_to_rad(90)
    beta_lim = 1.5
    accel_lim = 4.0
    mpc = MPC(20, Ts, steer_lim, beta_lim, accel_lim)
    state_hist = [vehicle.state[:2]]
    plotter = Plotter(x0, xf)
    plotter.update(vehicle.state, np.array([]), np.array([]), np.array([]), state_hist, obs)
    for i in range(100):
        mpc.optimize(vehicle.state, xf, 0)
        vehicle.update(mpc.input[0])
        state_hist.append(vehicle.state[:2])
        plotter.update(vehicle.state, mpc.pred, mpc.best_path, mpc.pot_paths, state_hist, obs) 
    '''
    plt.scatter(0, 0, c='g')
    plt.scatter(xf[0], xf[1], c='r')
    plt.plot(traj_x, traj_y, c='b')
    plt.show()
    '''
