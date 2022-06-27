import numpy as np

class Objet:
    def __init__(self, _lf: float, _lr: float):
        self.lf = _lf
        self.lr = _lr

class Vehicle:
    def __init__(self, _del_t: float, _lf: float, _lr: float, _x0: np.ndarray):
        self.del_t = _del_t
        self.lf = _lf
        self.lr = _lr
        
        self.x = _x0[0]
        self.y = _x0[1]
        self.v = _x0[2]
        self.head = _x0[3]

    @property
    def location(self):
        self._location = np.array([self.x, self.y])
        return self._location
    
    @property
    def state(self):
        self._state = np.array([self.x, self.y, self.v, self.head])
        return self._state
    
    def beta_to_steer(self, _beta):
        steer = np.arctan((self.lf+self.lr)*np.tan(_beta)/(self.lr))
        return steer

    @property
    def v_ratio(self):
        if not hasattr(self, '_v_ratio'):
            self._v_ratio = self.lr / (self.lf+self.lr)
        return self._v_ratio

    def update(self, _input: np.ndarray):
        self.beta = _input[0]
        alpha = self.head+_input[0]
        self.x += self.v*np.cos(alpha)*self.del_t
        self.y += self.v*np.sin(alpha)*self.del_t
        self.head += (self.v/self.lr)*np.sin(_input[0])*self.del_t
        self.v += _input[1]*self.del_t
        return self.state
  
