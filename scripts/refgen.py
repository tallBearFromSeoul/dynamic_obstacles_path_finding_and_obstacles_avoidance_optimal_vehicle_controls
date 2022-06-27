import numpy as np
import matplotlib.pyplot as plt


def rotate(v, a):
    A = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    return A@v

def increment(v, a, step):
    A = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
    return A@v*step

def increment_and_rotate(v, a, step):
    A = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
    return A@(rotate(v, a))*step

def inc_N(v, a, N):
    res = []
    for i in range(N):
        if i == 0:
            v = increment(v, a)
            res.append(v)
        else:
            v = increment(v)
            res.append(v)

class RefGen:
    def __init__(self, _num_case: int, _step_size : float, _inc_deg: float):
        self.num_case = _num_case
        self.step_size = _step_size
        self.inc_deg = _inc_deg
    def compute(self, p_init, p_des):
        self.p_init = p_init
        self.p_des = p_des
        self.p_mid = self.step_size*(p_des-p_init) + p_init

        self._pts = [rotate(self.p_mid, self.inc_deg*(i-self.num_case/2.0)*np.pi/180.0) for i in range(self.num_case)]
        res = []
        for _pt in self._pts:
            aux = increment(_pt, 0, 2.0)
            if (aux.
            temp = self.p_des - self.p_init
            temp2 = self.p_des - aux
            direct = True
            
            else:
                res.append(np.array([self.p_init, _pt, aux, self.p_des]))
        self.ref = np.array(res)
        

    def plot(self):
        if not hasattr(self, "ref"):
            return
        l = self.ref.tolist()
        for ll in l:
            x, y = zip(*ll)
            plt.plot(x,y)
        plt.show()

    def get_ref(self):
        if not hasattr(self, "ref"):
            return
        return self.ref

