import numpy as np
import casadi as ca

# Constants
dt = 0.2
N = 10

n_states = 3
n_controls = 2
n_obs = 3  # maximum number of obstacles

# Symbolic variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(x, y, theta)

v = ca.SX.sym('v')
omega = ca.SX.sym('omega')
controls = ca.vertcat(v, omega)

# Robot dynamics
rhs = ca.vertcat(
    v * ca.cos(theta),
    v * ca.sin(theta),
    omega
)
f = ca.Function('f', [states, controls], [rhs])

# MPC optimization variables
U = ca.SX.sym('U', n_controls, N)
X = ca.SX.sym('X', n_states, N + 1)

# P = [x, y, theta, x_target, y_target, x_obs1, y_obs1, ..., x_obs3, y_obs3]
P = ca.SX.sym('P', n_states + 2 + 2 * n_obs)

Q = ca.diag([10, 10, 1])  # state cost
R = ca.diag([0.5, 0.05])  # control cost

cost = 0
X[:, 0] = P[0:3]

for k in range(N):
    st = X[:, k]
    con = U[:, k]
    st_next = X[:, k] + dt * f(st, con)
    X[:, k + 1] = st_next

    dx = st[0] - P[3]  # target x
    dy = st[1] - P[4]  # target y
    cost += Q[0, 0]*dx**2 + Q[1, 1]*dy**2 + Q[2, 2]*st[2]**2
    cost += R[0, 0]*con[0]**2 + R[1, 1]*con[1]**2

    # Obstacle avoidance cost
    for i in range(n_obs):
        obs_x = P[5 + 2*i]
        obs_y = P[5 + 2*i + 1]
        dist_sq = (st[0] - obs_x)**2 + (st[1] - obs_y)**2
        cost += 20 / (dist_sq + 1e-2)  # avoid divide-by-zero

# No constraints added to g (can be extended later for hard constraints)
g = []

OPT_variables = U.reshape((-1, 1))
nlp_prob = {'f': cost, 'x': OPT_variables, 'p': P, 'g': ca.vertcat(*g)}

opts = {
    'ipopt.print_level': 0,
    'print_time': 0,
    'ipopt.max_iter': 100,
    'ipopt.tol': 1e-4,
}

solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

# Control bounds
lbx = np.tile(np.array([0.0, -1.0]), N)  # v >= 0, omega in [-1, 1]
ubx = np.tile(np.array([1.0, 1.0]), N)

# Main MPC call
def run_mpc(robot_state, target_pos, obstacles):
    """
    robot_state: [x, y, theta]
    target_pos: [x_target, y_target]
    obstacles: list of [x_obs, y_obs] (max 3 obstacles supported)
    """
    # Pad obstacles if fewer than 3
    obstacles = obstacles[:n_obs]
    while len(obstacles) < n_obs:
        obstacles.append([10.0, 10.0])  # place far away

    flat_obs = np.array(obstacles).flatten()
    p_val = np.concatenate((robot_state, target_pos, flat_obs))

    u0 = np.zeros((n_controls * N, 1))

    sol = solver(x0=u0, p=p_val, lbx=lbx, ubx=ubx)
    u_opt = sol['x'].full().reshape(n_controls, N)
    v = u_opt[0, 0]
    omega = u_opt[1, 0]
    return float(v), float(omega)
