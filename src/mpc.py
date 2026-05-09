import os
import copy

import casadi as ca
import matplotlib.pyplot as plt
import numpy as np

from cubic_spline_planner import CubicSpline2D

MAX_ITERATION = 20
DEFAULT_OUTPUT_GIF = os.path.join("assets", "images", "test.gif")


class MPC:
    def __init__(self, T, N):
        self.v_max = 0.5
        self.omega_max = np.pi / 2.0
        self.T = T
        self.N = N

    def shift_movement(self, cur_state, cur_control):
        f_value = self.f_np(cur_state, cur_control)
        cur_state = cur_state + self.T * f_value
        return cur_state

    def nonlinear_solver_generate(self, T, N):
        self.opti = ca.Opti()

        # control input
        self.opt_controls = self.opti.variable(N, 2)
        v = self.opt_controls[:, 0]
        omega = self.opt_controls[:, 1]

        # state
        self.opt_states = self.opti.variable(N + 1, 3)
        x = self.opt_states[:, 0]
        y = self.opt_states[:, 1]

        # differential-wheeled robot kinematics
        self.f = lambda x_, u_: ca.vertcat(
            *[u_[0] * np.cos(x_[2]), u_[0] * np.sin(x_[2]), u_[1]]
        )
        self.f_np = lambda x_, u_: np.array(
            [u_[0] * np.cos(x_[2]), u_[0] * np.sin(x_[2]), u_[1]]
        )

        self.opt_x0 = self.opti.parameter(3)  # init state
        self.opt_x_ref = self.opti.parameter(N + 1, 3)  # reference trajectory

        self.opti.subject_to(self.opt_states[0, :] == self.opt_x0.T)
        for i in range(N):
            x_next = (
                self.opt_states[i, :]
                + self.f(self.opt_states[i, :], self.opt_controls[i, :]).T * T
            )
            self.opti.subject_to(self.opt_states[i + 1, :] == x_next)

        Q = np.array([[20.0, 0.0, 0.0], [0.0, 20.0, 0.0], [0.0, 0.0, 0.1]])
        R = np.array([[0.5, 0.0], [0.05, 0.00]])

        obj = 0
        for i in range(N):
            obj = obj + ca.mtimes(
                [
                    (self.opt_states[i + 1, :] - self.opt_x_ref[i + 1, :]),
                    Q,
                    (self.opt_states[i + 1, :] - self.opt_x_ref[i + 1, :]).T,
                ]
            ) + ca.mtimes([self.opt_controls[i, :], R, self.opt_controls[i, :].T])

        self.opti.minimize(obj)

        self.opti.subject_to(self.opti.bounded(-2.0, x, 2.0))
        self.opti.subject_to(self.opti.bounded(-2.0, y, 2.0))
        self.opti.subject_to(self.opti.bounded(-self.v_max, v, self.v_max))
        self.opti.subject_to(self.opti.bounded(-self.omega_max, omega, self.omega_max))

        opts_setting = {
            "ipopt.max_iter": 200,
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.acceptable_tol": 1e-8,
            "ipopt.acceptable_obj_change_tol": 1e-6,
        }
        self.opti.solver("ipopt", opts_setting)

    def mpc(self, cur_state, cur_control, final_state, ref_state):
        self.nonlinear_solver_generate(T=self.T, N=self.N)
        self.opti.set_value(self.opt_x_ref, ref_state[0 : self.N + 1])

        pred_state = np.zeros((self.N + 1, 3))
        pred_control = np.zeros((self.N, 2))
        pred_control[0] = cur_control
        mpciter = 0

        while np.linalg.norm(cur_state[0:1] - final_state[0:1]) > 1e-2 and mpciter < 1:
            self.opti.set_value(self.opt_x0, cur_state)
            self.opti.set_initial(self.opt_controls, pred_control)
            self.opti.set_initial(self.opt_states, pred_state)
            sol = self.opti.solve()

            pred_control = sol.value(self.opt_controls)
            pred_state = sol.value(self.opt_states)
            pred_state = np.concatenate((pred_state[1:], pred_state[-1:]))
            pred_control = np.concatenate((pred_control[1:], pred_control[-1:]))
            mpciter += 1

        state_one_iteration = []
        for i in range(1):
            f_value = self.f_np(cur_state, pred_control[i])
            cur_state = cur_state + self.T * f_value
            state_one_iteration.append(cur_state)

        return state_one_iteration

    def path_generate(self, ref_path):
        x = ref_path[:, 0]
        y = ref_path[:, 1]
        ds = 0.02
        sp = CubicSpline2D(x, y)
        s = np.arange(0, sp.s[-1], ds)

        rx, ry, ryaw = [], [], []
        ref_path = []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(sp.calc_yaw(i_s))
            ref_path.append([ix, iy, sp.calc_yaw(i_s)])

        while len(ref_path) / 3 <= self.N + 1:
            ref_path.append([rx[-1], ry[-1], ryaw[-1]])

        return np.array(ref_path)

    def visualization(self, ref_path, actual_path, whether_gif=False, output_path=DEFAULT_OUTPUT_GIF):
        if not whether_gif:
            plt.plot(ref_path[:, 0], ref_path[:, 1], "-r", label="ref")
            plt.plot(actual_path[:, 0], actual_path[:, 1], "-b", label="mpc")
            plt.legend(loc="lower right")
            plt.show()
            return

        import matplotlib.animation as animation

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        fig = plt.figure()
        plt.plot(ref_path[:, 0], ref_path[:, 1], "-r", label="ref")
        ims = []
        for i in range(len(actual_path)):
            im = plt.plot(actual_path[0:i, 0], actual_path[0:i, 1], "ob", label="mpc")
            ims.append(im)

        ani = animation.ArtistAnimation(fig, ims, interval=10, repeat_delay=1)
        ani.save(output_path, writer="pillow")
        plt.show()


def main():
    controller = MPC(T=0.05, N=50)
    cur_state = np.array([-1.0, -1.0, 0.0])
    cur_control = np.array([0.0, 0.0])
    final_state = np.array([0.5, 0.0, 0.0])
    ref_path = np.array(
        [
            [-1.0, -1.0, 0.0],
            [0.0, -0.5, 0.0],
            [0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0],
        ]
    )

    next_state = copy.deepcopy(cur_state)
    ref_path = controller.path_generate(ref_path=ref_path)
    ref_path_tot = copy.deepcopy(ref_path)
    actual_path_tot = [cur_state]
    cnt = 0

    while np.linalg.norm(next_state[0:2] - final_state[0:2]) > 3e-2 and cnt <= 1000:
        ref_path = np.concatenate((ref_path[1:], ref_path[-1:]))
        ref_path[0] = next_state
        print(next_state, np.linalg.norm(next_state[0:2] - final_state[0:2]))
        state_one_iteration = controller.mpc(
            cur_state=next_state,
            cur_control=cur_control,
            final_state=final_state,
            ref_state=ref_path,
        )
        next_state = state_one_iteration[-1]
        actual_path_tot += state_one_iteration
        cnt += 1

    actual_path_tot = np.array(actual_path_tot)
    controller.visualization(ref_path=ref_path_tot, actual_path=actual_path_tot, whether_gif=True)


if __name__ == "__main__":
    main()
