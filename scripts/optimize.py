import numpy as np
from vehicle import Vehicle
from pathgen import PathGen
import pyomo.environ as pyo

class MPC:
    def __init__(self, _N: int, _ts: float, _steer_lim: float, _beta_lim:float, _accel_lim: float,  _lr: float = 1.738):
        self.N = _N
        self.ts = _ts
        self.dec_rat = 4

        self.nx = 4
        self.nu = 2
        self.M = 100
        self.lr = _lr 
        
        self.xOpt = np.zeros((self.nx,self.M+1))
        self.uOpt = np.zeros((self.nu,self.M))
        self.xPred = np.zeros((self.nx,self.N+1,self.M))
        self.feas = np.zeros((self.M,), dtype=bool)
        self.xN = np.zeros((self.nx,1))

        self.P = np.diag([10, 14, 0, 0])
        self.Q = np.diag([10, 14, 0, 0])
        self.R = np.eye(2)

        #State limit
        self.x3U = _steer_lim
        self.u0U = _beta_lim
        self.u1U = _accel_lim

        self.model = pyo.ConcreteModel()
        #self.model = pyo.AbstractModel()
        self.model.name = 'nonlinear optimizer with constraints'

        # length of finite optimization problem:
        self.model.tIDX = pyo.Set( initialize= range(self.N+1), ordered=True )
        self.model.xIDX = pyo.Set( initialize= range(self.nx), ordered=True )
        self.model.uIDX = pyo.Set( initialize= range(self.nu), ordered=True )
        
        # these are 2d arrays:
        self.model.Q = self.Q
        self.model.P = self.P
        self.model.R = self.R

        # Create state and input variables trajectory:
        self.model.x = pyo.Var(self.model.xIDX, self.model.tIDX)
        self.model.u = pyo.Var(self.model.uIDX, self.model.tIDX)


        self.model.equality_const1 = pyo.Constraint(self.model.tIDX, rule = lambda model, t: self.model.x[0, t+1] == self.model.x[0,t] + self.ts * (self.model.x[2,t] * pyo.cos(self.model.x[3,t]+ self.model.u[0, t]))
                                               if t < self.N else pyo.Constraint.Skip)
        self.model.equality_const2 = pyo.Constraint(self.model.tIDX, rule = lambda model, t: self.model.x[1, t+1] == self.model.x[1,t] + self.ts * (self.model.x[2,t] * pyo.sin(self.model.x[3,t]+ self.model.u[0, t]))
                                               if t < self.N else pyo.Constraint.Skip)
        self.model.equality_const3 = pyo.Constraint(self.model.tIDX, rule = lambda model, t: self.model.x[2, t+1] == self.model.x[2,t] + self.ts * self.model.u[1,t] if t < self.N else pyo.Constraint.Skip)
        self.model.equality_const4 = pyo.Constraint(self.model.tIDX, rule = lambda model, t: self.model.x[3, t+1] == self.model.x[3,t] + self.ts * (1/self.lr) * (self.model.x[2,t] * pyo.sin(self.model.u[0, t]))
                                               if t < self.N else pyo.Constraint.Skip)

        self.model.state_psi_const1 = pyo.Constraint(self.model.tIDX, rule = lambda model, t: self.model.x[3,t] <= self.x3U if t < self.N else pyo.Constraint.Skip)
        self.model.state_psi_const2 = pyo.Constraint(self.model.tIDX, rule = lambda model, t: self.model.x[3,t] >= -self.x3U if t < self.N else pyo.Constraint.Skip)
 
        self.model.input_beta_const1 = pyo.Constraint(self.model.tIDX, rule = lambda model, t: self.model.u[0,t] <= self.u0U if t < self.N else pyo.Constraint.Skip)
        self.model.input_beta_const2 = pyo.Constraint(self.model.tIDX, rule = lambda model, t: self.model.u[0,t] >= -self.u0U if t < self.N else pyo.Constraint.Skip)

        self.model.input_a_const1 = pyo.Constraint(self.model.tIDX, rule = lambda model, t: self.model.u[1,t] <= self.u1U if t < self.N else pyo.Constraint.Skip)
        self.model.input_a_const2 = pyo.Constraint(self.model.tIDX, rule = lambda model, t: self.model.u[1,t] >= -self.u1U if t < self.N else pyo.Constraint.Skip)
        self.solver = pyo.SolverFactory('ipopt')

    def objective_rule(self, model):
        costX = 0.0
        costU = 0.0
        costTerminal = 0.0
        for t in model.tIDX:
            for i in model.xIDX:
                if i >= self.nx-2:
                    continue
                for j in model.xIDX:
                    if j >= self.nx-2:
                        continue
                    costTerminal += (model.x[i, self.N] - model.zref[i//self.dec_rat, self.N//self.dec_rat]) * model.P[i, j] * (model.x[j, self.N] - model.zref[j//self.dec_rat, self.N//self.dec_rat])
                    if t < self.N:
                        costX += (model.x[i, t] - model.zref[i//self.dec_rat, t//self.dec_rat]) * model.Q[i, j] * (model.x[j, t] - model.zref[j//self.dec_rat, t//self.dec_rat])
            for i in model.uIDX:
                for j in model.uIDX:
                    if t < self.N:
                        costU += (model.u[i, t]) * model.R[i, j] * (model.u[j, t])
        return costX + costU + costTerminal

    def solve_cftoc(self, x0, zref): #, y_traj #, target_x_sim, target_y_sim
        self.model.zref = zref
        self.model.cost = pyo.Objective(rule = self.objective_rule, sense=pyo.minimize)
        self.model.init_const1 = pyo.Constraint(expr = self.model.x[0, 0] == x0[0])
        self.model.init_const2 = pyo.Constraint(expr = self.model.x[1, 0] == x0[1])
        self.model.init_const3 = pyo.Constraint(expr = self.model.x[2, 0] == x0[2])
        self.model.init_const4 = pyo.Constraint(expr = self.model.x[3, 0] == x0[3])

        res = self.solver.solve(self.model)
        if str(res.solver.termination_condition) == "optimal":
            feas = True
        else:
            feas = False

        xOpt = np.asarray([[self.model.x[i,t]() for i in self.model.xIDX] for t in self.model.tIDX]).T
        uOpt = np.asarray([self.model.u[:,t]() for t in self.model.tIDX]).T

        JOpt = self.model.cost()

        return [feas, xOpt, uOpt, JOpt]

    def optimize(self, x0, xf, t):
        xinit = x0[:2]
        pg = PathGen(20, xinit, xf, self.N//self.dec_rat, 5)
        pg.compute(0)
        paths = pg.get_splines()
        self.pot_paths = []
        best_path = None
        min_J = 100000000

        obss = [np.array([3,3,1])]
        
        for obs in obss:
            self.model.obstacle_const = pyo.Constraint(self.model.tIDX, rule = lambda model, t: np.abs((self.model.x[0,t]-obs[0])**2 + (self.model.x[1,t]-obs[1])**2) >= obs[2] if t<self.N else pyo.Constraint.Skip)

        for path in paths:
            x, y = zip(*path)
            x_np = np.array(x)
            y_np = np.array(y)

            if feas:
                self.pot_paths.append(path)
                #self.pot_paths.append({'path':path, 'pred':x, 'input':u, 'cost':J})
                if J < min_J:
                    min_J = J
                    best_path = {'path':path, 'pred':x, 'input':u, 'cost':J}
        self.best_path = best_path['path']
        self.pred = best_path['pred']
        self.input = best_path['input']
        self.n_pot_paths = len(self.pot_paths)
        print(f'Number of potential paths out of {self.N} are {self.n_pot_paths}')

