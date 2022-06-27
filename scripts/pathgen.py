import numpy as np
import matplotlib.pyplot as plt
from refgen import RefGen

class PathGen:
    def __init__(self, _num_case, _p_init, _p_des, _precision: int, _inc_deg: float):
        self.num_case = _num_case
        gen = RefGen(_num_case, 0.5, _inc_deg)
        gen.compute(_p_init, _p_des)
        self.control_pts = gen.get_ref()
        self.n = self.control_pts[0].shape[0]
        self.precision = _precision
        self.thresh = 1.0

    def compute(self, _algo : int) -> list:
        self._splines = [[_ for _ in range(self.precision+1)] for _ in range(self.num_case)]
        if (_algo == 0):
            # de Casteljau's Algorithm
            for ii in range(self.num_case):
                for i in range(self.precision+1):
                    t = float(i)/float(self.precision)
                    self._splines[ii][i] = self.deCasteljaus(t, ii)
        else:
            pass
        return self._splines

    def deCasteljaus(self, _t : float, i) -> np.ndarray:
        pts = [np.array(pt) for pt in self.control_pts[i]]
        for k in range(1, self.n+1):
            for i in range(0, self.n-k):
                pts[i] = (1-_t)*pts[i] + _t*pts[i+1]
        return pts[0]

    def get_splines(self):
        return self._splines
    def get_first_spline(self):
        return self._splines[9]

    def draw(self):
        f, ax = plt.subplots(1,1,figsize=(10,10))
        for ii in range(self.num_case):
            res_x, res_y = zip(*self._splines[ii])
            ax.scatter(res_x, res_y, s=4, cmap=plt.cm.jet)
            ax.plot(res_x, res_y)
        plt.show()

