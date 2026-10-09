# %% TSFS12 Hand-in exercise 3: Path following for autonomous vehicles

import numpy as np
import matplotlib.pyplot as plt
from vehiclecontrol import ControllerBase, SingleTrackModel, PurePursuitControllerBase
from splinepath import SplinePath
from scipy.linalg import solve_discrete_are

# Run if you want plots in external windows
# %matplotlib


# Run the ipython magic below to activate automated import of modules. Useful if you write code in external .py files.
# %load_ext autoreload
# %autoreload 2


# %% Make a simple controller and simulate vehicle


class MiniController(ControllerBase):
    def __init__(self):
        super().__init__()

    def u(self, t, w):
        a = 0.0
        if t < 10:
            u = [np.pi / 180 * 10, a]
        elif 10 <= t < 20:
            u = [-np.pi / 180 * 11, a]
        elif 20 <= t < 23:
            u = [-np.pi / 180 * 0, a]
        elif 23 <= t < 40:
            u = [-np.pi / 180 * 15, a]
        else:
            u = [-np.pi / 180 * 0, a]
        return u


opts = {"L": 2, "amax": np.inf, "amin": -np.inf, "steer_limit": np.pi / 3}

car = SingleTrackModel().set_attributes(opts)
car.Ts = 0.1
car.controller = MiniController()
w0 = [0, 0, 0, 2]
z0 = car.simulate(w0, T=40, dt=0.1, t0=0.0)
t, w, u = z0
M = 10
p = w[::M, 0:2]
nom_path = SplinePath(p)


s = np.linspace(0, nom_path.length, 100)

_, ax = plt.subplots(num=10, clear=True)
ax.plot(nom_path.x(s), nom_path.y(s))
ax.plot(p[:, 0], p[:, 1], "rx")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("Path from simple controller")
ax.axis("square")

fig, ax = plt.subplots(1, 2, num=11, clear=True, layout="constrained")
ax[0].plot(t, u[:, 0] * 180 / np.pi)
ax[0].set_xlabel("t [s]")
ax[0].set_ylabel("steer [deg]")
ax[0].set_title("Steer")

ax[1].plot(t, u[:, 1])
ax[1].set_xlabel("t [s]")
ax[1].set_ylabel("acceleration [m/s^2]")
ax[1].set_title("Acceleration")


# %% Pure pursuit controller


class PurePursuitController(PurePursuitControllerBase):
    def __init__(
        self,
        l: float,
        L: float,
        path: SplinePath = None,
        goal_tol: float = 1.0,
        pursuit_point_fig=None,
    ):
        """
        Create pure-pursuit controller object

        Input arguments:
          l -- Prediction horizon
          L -- wheel-base
          path -- SplinePath-object with the path to follow
          goal_tol -- Goal stopping tolerance (default: 1)
          pursuit_point_fig -- Figure object to illustrate pursuit-point selection (default: None)
        """
        super().__init__(pursuit_point_fig)
        self.plan = path
        self.l = l
        self.L = L
        self.goal_tol = goal_tol

    def pursuit_point(self, p_car):
        """Return pure-pursuit given the position of the car.

        Input:
          p_car - Car position in global coordinates

        Return:
          pursuit point in global coordinates
        """
        # p_car - position of vehicle

        path_points = self.plan.path  # Points on the path
        l = self.l  # Pure-pursuit look-ahead

        if not hasattr(self, 'search_index'):
            self.search_index = 0

        while self.search_index < len(path_points) - 1:
            candidate_point = path_points[self.search_index]
            dist = np.linalg.norm(candidate_point - p_car)
            if dist >= l:
                break
            self.search_index += 1

        p_purepursuit = path_points[self.search_index]
        return p_purepursuit

    def pure_pursuit_control(self, dp, theta):
        """Compute pure-pursuit steer angle.

        Input:
          dp - Vector from position of car to pursuit point
          theta - heading of vehicle

        Output:
          return steer angle
        """

        heading = np.array([np.cos(theta), np.sin(theta)])
        cross_track = heading[0] * dp[1] - heading[1] * dp[0]
        delta = np.arctan2(2 * self.L * cross_track, dp.dot(dp))
        return delta

    def u(self, t, w):
        """Compute control action

        Input:
          t - current time
          w - current state w = (x, y, theta, v)

        Output:
          return (delta, acc) where delta is steer angle and acc acceleration
        """
        x, y, theta, v = w
        p_car = np.array([x, y])

        p_purepursuit = self.pursuit_point(p_car)
        dp = p_purepursuit - p_car
        delta = self.pure_pursuit_control(dp, theta)
        acc = 0.0

        self._pursuit_plot(p_car, p_purepursuit)  # Included for animation purposes.

        return np.array([delta, acc])

    def run(self, t, w):
        # Function that returns true until goal is reached
        p_goal = self.plan.path[-1, :]
        p_car = w[0:2]
        dp = p_car - p_goal
        dist = dp.dot(dp)

        return dist > self.goal_tol**2


# %%# Assertions

# A few tests on your implementation. Note that passing these tests doesn't imply that your solution is correct but do not submit a solution if your solution doesn't pass these tests. First, test the ```pure_pursuit_control``` function

pp_controller = PurePursuitController(l=4, L=car.L, path=nom_path, goal_tol=0.25)
assert (
    abs(pp_controller.pure_pursuit_control(np.array([1.0, 1.0]), 10 * np.pi / 180) - 1.01840) < 1e-3
)
assert (
    abs(pp_controller.pure_pursuit_control(np.array([1.0, 1.0]), 30 * np.pi / 180) - 0.6319) < 1e-3
)
assert (
    abs(pp_controller.pure_pursuit_control(np.array([1.0, 1.0]), -30 * np.pi / 180) - 1.2199) < 1e-3
)


# To ensure that the pursuit-point selection works, run a simulation with pure-pursuit illustration turned on. Requires plotting in external window.

s = np.linspace(0, nom_path.length, 200)
fig, ax = plt.subplots(num=99, clear=True)
ax.plot(nom_path.x(s), nom_path.y(s), "b", lw=0.5)
ax.plot(nom_path.path[:, 0], nom_path.path[:, 1], "rx", markersize=3)

car = SingleTrackModel()
car.controller = PurePursuitController(
    l=2, L=car.L, path=nom_path, goal_tol=0.25, pursuit_point_fig=fig
)

#w0 = [0, 1, np.pi / 2 * 0.9, 2]
w0 = [0, 1, np.pi / 2 * 0.9, 2]
z_pp = car.simulate(w0, T=80, dt=0.1, t0=0.0)


# %%# Simulate controller

# Simulate controller without visualization

car = SingleTrackModel()
pp_controller = PurePursuitController(l=4, L=car.L, path=nom_path)
car.controller = pp_controller

w0 = [0, 1, np.pi / 2 * 0.9, 2]  # Sample starting state
z_pp = car.simulate(w0, T=80, dt=0.1, t0=0.0)


# %% State feedback controller based on the linearized path

# Implement linear and non-linear state feedback control.


class StateFeedbackController(ControllerBase):
    def __init__(self, K: float, L: float, path: SplinePath = None, goal_tol: float = 1.0):
        super().__init__()
        self.plan = path
        self.K = K
        self.goal_tol = goal_tol
        self.d = []
        self.L = L
        self.s0 = 0

    def heading_error(self, theta, s):
        """Compute theta error
        Inputs
            theta - current heading angle
            s - projection point on path

        Outputs
            theta_e - heading error angle
        """
      
        h_s, _ = self.plan.heading(s)
        h = np.array([np.cos(theta), np.sin(theta)])
        # sin(theta_e) = (h_s x h)_z, cos(theta_e) = h_s . h  (no angle wrapping needed)
        return np.arctan2(h_s[0] * h[1] - h_s[1] * h[0], h_s @ h)

    def u(self, t, w):
        x, y, theta, v = w
        p_car = w[0:2]

        # Compute d and theta_e errors. Use the SplinePath method project
        # and the obj.heading_error() function you've written above

        # YOUR CODE HERE
        s, d = self.plan.project(p_car, self.s0)
        self.s0 = s
        theta_e = self.heading_error(theta, s)

        u0 = self.plan.c(s)                          
        u_curv = u0 - self.K @ np.array([d, theta_e])
        delta = np.arctan(self.L * u_curv)

        acc = 0.0
        return np.array([delta, acc])

    def run(self, t, w):
        p_goal = self.plan.path[-1, :]
        p_car = w[0:2]
        dp = p_car - p_goal
        dist = np.sqrt(dp.dot(dp))

        return dist > self.goal_tol**2

# %% Nonlinear state feedback controller
#5.8
def sinc_series(theta):
    theta2 = theta ** 2
    return 1 - theta2 / 6 + theta2**2 / 120 - theta2**3 / 5040
  
class NonlinearStateFeedbackController(ControllerBase):
    def __init__(self, K: float, L: float, path: SplinePath = None, goal_tol: float = 1.0):
        super().__init__()
        self.plan = path
        self.K = K
        self.goal_tol = goal_tol
        self.d = []
        self.L = L
        self.s0 = 0

    def heading_error(self, theta, s):
        h_s, _ = self.plan.heading(s)
        h = np.array([np.cos(theta), np.sin(theta)])
        err = np.arctan2(h_s[0] * h[1] - h_s[1] * h[0], h_s @ h)
        return err 
      
    def u(self, t, w):
        x, y, theta, v = w
        p_car = w[0:2]

        # Compute d and theta_e errors. Use the SplinePath method project
        # and the obj.heading_error() function you've written above

        # YOUR CODE HERE
        s, d = self.plan.project(p_car, self.s0)
        self.s0 = s
        theta_e = self.heading_error(theta, s)

        u0 = self.plan.c(s)
        sinc_theta_e = sinc_series(theta_e)
        u_curv = u0 - self.K @ np.array([sinc_theta_e * d, theta_e])
        delta = np.arctan(self.L * u_curv)

        acc = 0.0
        return np.array([delta, acc])

    def run(self, t, w):
        p_goal = self.plan.path[-1, :]
        p_car = w[0:2]
        dp = p_car - p_goal
        dist = np.sqrt(dp.dot(dp))

        return dist > self.goal_tol**2


# %% Design LQR gain and simulate state-feedback controller

Ts = 0.1
v0 = 2.0                                    

A = np.array([[1, v0 * Ts], [0, 1]])
B = np.array([[0], [v0 * Ts]])

#Error and steer weigths
Q = np.diag([1.0, 1.0])                    
R = np.array([[1.0]])                      

P = solve_discrete_are(A, B, Q, R)
K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
K = K.flatten()                             

car = SingleTrackModel()
sf_controller = StateFeedbackController(K=K, L=car.L, path=nom_path)
car.controller = sf_controller

#5.7
w0 = [-5, 10,np.pi / 2 * 0.9, 2]
z_sf = car.simulate(w0, T=80, dt=Ts, t0=0.0)

t_sf, w_sf, u_sf = z_sf
s_plot = np.linspace(0, nom_path.length, 200)
fig, ax = plt.subplots(num=50, clear=True)
ax.plot(nom_path.x(s_plot), nom_path.y(s_plot), "b", lw=1, label="path")
ax.plot(w_sf[:, 0], w_sf[:, 1], "k", lw=1, label="LQ state-feedback")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.axis("equal")
ax.legend()

#5.8
car = SingleTrackModel()
nl_controller = NonlinearStateFeedbackController(K=K, L=car.L, path=nom_path)
car.controller = nl_controller
w0 = [-5, 10, np.pi / 2 * 0.9, 2]
z_nl = car.simulate(w0, T=80, dt=Ts, t0=0.0)
t_nl, w_nl, u_nl = z_nl
s_plot = np.linspace(0, nom_path.length, 200)
fig, ax = plt.subplots(num=70, clear=True)
ax.plot(nom_path.x(s_plot), nom_path.y(s_plot), "b", lw=1, label="path")
ax.plot(w_nl[:, 0], w_nl[:, 1], "g", lw=1, label="nonlinear state-feedback")
ax.plot(w0[0], w0[1], "ko", label="start")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.axis("equal")
ax.legend()


# %% Sweep Q and R weights and compare
def design_and_simulate(Q, R, v0=2.0, Ts=0.1, w0=None, T=80):
    if w0 is None:
        w0 = [0, 1, np.pi / 2 * 0.9, 2]

    A = np.array([[1, v0 * Ts], [0, 1]])
    B = np.array([[0], [v0 * Ts]])

    P = solve_discrete_are(A, B, Q, R)
    K = (np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)).flatten()

    car = SingleTrackModel()
    car.controller = StateFeedbackController(K=K, L=car.L, path=nom_path)
    t, w, u = car.simulate(w0, T=T, dt=Ts, t0=0.0)
    d = nom_path.path_error(w[:, 0:2])

    return {"K": K, "t": t, "w": w, "u": u, "d": d}


def plot_sweep(results, labels, fignum, title):
    fig, ax = plt.subplots(1, 2, num=fignum, clear=True, figsize=(11, 4), layout="constrained")
    for res, label in zip(results, labels):
        ax[0].plot(res["t"], res["d"], label=label)
        ax[1].plot(res["t"], res["u"][:, 0] * 180 / np.pi, label=label)

    ax[0].set_xlabel("t [s]")
    ax[0].set_ylabel("d [m]")
    ax[0].set_title(f"{title} -- path error")
    ax[0].legend()

    ax[1].axhline(60, color="r", ls="--", lw=0.7)
    ax[1].axhline(-60, color="r", ls="--", lw=0.7)
    ax[1].set_xlabel("t [s]")
    ax[1].set_ylabel("steer [deg]")
    ax[1].set_title(f"{title} -- steering command (dashed = actuator limit)")
    ax[1].legend()


# Sweep Q (distance-error weight), R fixed
q_values = [1, 10, 100]
q_results = [design_and_simulate(Q=np.diag([q, 1.0]), R=np.array([[1.0]])) for q in q_values]
plot_sweep(q_results, [f"Q=diag({q},1)" for q in q_values], fignum=60, title="Q sweep (R=1)")

# Sweep R (control-effort weight), Q fixed
r_values = [0.1, 1, 10]
r_results = [design_and_simulate(Q=np.diag([1.0, 1.0]), R=np.array([[r]])) for r in r_values]
plot_sweep(r_results, [f"R={r}" for r in r_values], fignum=61, title="R sweep (Q=diag(1,1))")


# %% Compare all three controllers (Exercise 5.9)

w0 = [0, 1, np.pi / 2 * 0.9, 2]

car_pp = SingleTrackModel()
car_pp.controller = PurePursuitController(l=2, L=car_pp.L, path=nom_path)
t_pp, w_pp, u_pp = car_pp.simulate(w0, T=80, dt=Ts, t0=0.0)
d_pp = nom_path.path_error(w_pp[:, 0:2])

car_lin = SingleTrackModel()
car_lin.controller = StateFeedbackController(K=K, L=car_lin.L, path=nom_path)
t_lin, w_lin, u_lin = car_lin.simulate(w0, T=80, dt=Ts, t0=0.0)
d_lin = nom_path.path_error(w_lin[:, 0:2])

car_nl = SingleTrackModel()
car_nl.controller = NonlinearStateFeedbackController(K=K, L=car_nl.L, path=nom_path)
t_nl, w_nl, u_nl = car_nl.simulate(w0, T=80, dt=Ts, t0=0.0)
d_nl = nom_path.path_error(w_nl[:, 0:2])

s_plot = np.linspace(0, nom_path.length, 200)

# Trajectories
fig, ax = plt.subplots(num=80, clear=True)
ax.plot(nom_path.x(s_plot), nom_path.y(s_plot), "b", lw=1, label="path")
ax.plot(w_pp[:, 0], w_pp[:, 1], "r", lw=1, label="pure pursuit")
ax.plot(w_lin[:, 0], w_lin[:, 1], "g", lw=1, label="linear SF")
ax.plot(w_nl[:, 0], w_nl[:, 1], "m", lw=1, label="nonlinear SF")
ax.legend(); ax.axis("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")

# Control signals
fig, ax = plt.subplots(num=81, clear=True)
ax.plot(t_pp, u_pp[:, 0] * 180 / np.pi, "r", label="pure pursuit")
ax.plot(t_lin, u_lin[:, 0] * 180 / np.pi, "g", label="linear SF")
ax.plot(t_nl, u_nl[:, 0] * 180 / np.pi, "m", label="nonlinear SF")
ax.set_xlabel("t [s]"); ax.set_ylabel("steer [deg]"); ax.legend()

# Path error via path_error()
fig, ax = plt.subplots(num=82, clear=True)
ax.plot(t_pp, d_pp, "r", label="pure pursuit")
ax.plot(t_lin, d_lin, "g", label="linear SF")
ax.plot(t_nl, d_nl, "m", label="nonlinear SF")
ax.set_xlabel("t [s]"); ax.set_ylabel("d [m]"); ax.legend()




# %%
plt.show()
