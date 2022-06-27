import numpy as np
import matplotlib.pyplot as plt


class Plotter:
    def __init__(self, _x0, _xf):
        plt.ion()
        plt.style.use('dark_background')
        self.figure, self.ax = plt.subplots(1, 1, figsize=(10,10))
        self.ax.set_ylim([-10, 10])
        self.ax.set_xlim([-10, 10])
        self.ax.set_xlabel('x-axis')
        self.ax.set_ylabel('y-axis')
        plt.show()
        self.x0 = _x0[:2]
        self.xf = _xf
        self.p_init_des = plt.scatter([_x0[0], _xf[0]], [_x0[1], _xf[1]], c='g')
        self.vehicle_path_plot = plt.plot([], [], 'b--')[0]
        self.prediction_plot = plt.plot([], [], 'r--')[0]
        self.best_path_plot = plt.plot([], [], 'g.')[0]
        self.potential_paths_plot = plt.plot([], [], 'y:')[0]
        self.trajectory_plot = plt.plot([], [], 'g--')[0]
        plt.legend(('path', 'prediction horizon', 'trajectory'), loc='upper right')
    
    def update(self, state: np.ndarray, pred: np.ndarray, best_path, pot_paths, state_hist: list, obs: np.ndarray):
        self.vehicle_path_plot.set_xdata(state[0])
        self.vehicle_path_plot.set_ydata(state[1])        
        if (pred.size > 0):
            pred_l = pred[:2].transpose().tolist()
            pred_x, pred_y = zip(*pred_l)
            self.prediction_plot.set_xdata(pred_x)
            self.prediction_plot.set_ydata(pred_y)
            best_path_x, best_path_y = zip(*best_path)
            self.best_path_plot.set_xdata(best_path_x)
            self.best_path_plot.set_ydata(best_path_y)
        pot_x = []
        pot_y = []
        for pot_path in pot_paths:
            pot_xx, pot_yy = zip(*pot_path)
            pot_x.append(pot_xx)
            pot_y.append(pot_yy)
        self.potential_paths_plot.set_xdata(pot_x)
        self.potential_paths_plot.set_ydata(pot_y)
        self.ax.add_patch(plt.Circle((obs[0], obs[1]), 1))
        state_hist_x, state_hist_y = zip(*state_hist)
        self.trajectory_plot.set_xdata(state_hist_x)
        self.trajectory_plot.set_ydata(state_hist_y)
        plt.pause(0.05)
        plt.show()
