import casadi as ca
import casadi.tools as ca_tools
import numpy as np
import time

class MPC:
    def __init__(self):
        self.v_max = 0.6
        self.omega_max = np.pi/4.0
        self.T = 0.2
        self.N = 100
    
    def shift_movement(self, cur_state, cur_control):
        f_value = self.f(cur_state, cur_control)
        cur_state = cur_state + self.T*f_value
        return cur_state
    
    def solver_generate(self, T, N):

        self.opti = ca.Opti()
        # control input
        self.opt_controls = self.opti.variable(N, 2)
        v = self.opt_controls[:, 0]
        omega = self.opt_controls[:, 1]
        # state
        self.opt_states = self.opti.variable(N+1, 3)
        x = self.opt_states[:, 0]
        y = self.opt_states[:, 1]
        theta = self.opt_states[:, 2]
        # f_d
        self.f = lambda x_, u_: ca.vertcat(*[u_[0]*np.cos(x_[2]), u_[0]*np.sin(x_[2]), u_[1]])
        self.f_np = lambda x_, u_: np.array([u_[0]*np.cos(x_[2]), u_[0]*np.sin(x_[2]), u_[1]])

        self.opt_x0 = self.opti.parameter(3) # init state
        self.opt_x_ref = self.opti.parameter(3) # ref state

        self.opti.subject_to(self.opt_states[0, :] == self.opt_x0.T)
        for i in range(N):
            # x(k+i+1|k) = f_d(x(k+i|k), u(k+i|k))
            x_next = self.opt_states[i, :] + self.f(self.opt_states[i, :], self.opt_controls[i, :]).T*T
            self.opti.subject_to(self.opt_states[i+1, :]==x_next)

        Q = np.array([[5.0, 0.0, 0.0],[0.0, 5.0, 0.0],[0.0, 0.0, .1]])
        R = np.array([[0.5, 0.0], [0.05, 0.00]])

        obj = 0
        for i in range(N):

            obj = obj + ca.mtimes([(self.opt_states[i, :]-self.opt_x_ref.T), Q, (self.opt_states[i, :]-self.opt_x_ref.T).T]) \
                + ca.mtimes([self.opt_controls[i, :], R, self.opt_controls[i, :].T])
        
        # optimization objective
        self.opti.minimize(obj)

        self.opti.subject_to(self.opti.bounded(-2.0, x, 2.0))
        self.opti.subject_to(self.opti.bounded(-2.0, y, 2.0))
        self.opti.subject_to(self.opti.bounded(-self.v_max, v, self.v_max))
        self.opti.subject_to(self.opti.bounded(-self.omega_max, omega, self.omega_max))
        
        opts_setting = {'ipopt.max_iter':200, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}

        self.opti.solver('ipopt', opts_setting)

    def mpc_nonliner(self, cur_state, cur_control, final_state):
        self.solver_generate(T = self.T, N = self.N)
        self.opti.set_value(self.opt_x_ref, final_state)
        pred_state = np.zeros((self.N+1, 3))
        pred_cur = np.zeros((self.N, 2))
        pred_cur[0] = cur_control
        tot_control = []
        mpciter = 0
        while(np.linalg.norm(cur_state - final_state) > 1e-1 and mpciter < 100  ):
            
            self.opti.set_value(self.opt_x0, cur_state)
            self.opti.set_initial(self.opt_controls, pred_cur) # (N, 2)
            self.opti.set_initial(self.opt_states, pred_state) # (N+1, 3)
            sol = self.opti.solve()

            pred_control = sol.value(self.opt_controls)
            tot_control.append(pred_control[0, :])
            pred_state = sol.value(self.opt_states)

            f_value = self.f_np(cur_state, pred_control[0])
            cur_state = cur_state + self.T*f_value
            print(cur_state, pred_control[0])
            # update variable
            # pred_state = np.concatenate((pred_state[1:], pred_state[-1:]))
            # pred_control = np.concatenate((pred_control[1:], pred_control[-1:]))
            mpciter = mpciter + 1

if __name__ == '__main__':
    controller = MPC()
    print(1)
    cur_state = np.array([-1., -1., 0.])
    cur_control = np.array([0., 0.])
    final_state = np.array([1.5, 1.5, 0.])
    controller.mpc_nonliner(cur_state=cur_state, cur_control=cur_control, final_state=final_state)
    