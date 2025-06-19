import numpy as np
import casadi as ca

# Constants
dt = 0.2
N = 7
n_states = 3
n_controls = 2
n_obs = 3

robot_radius = 0.03        # approximate radius of your robot footprint (meters)
cylinder_radius = 0.2     # radius of cylinder obstacles (meters)
safety_margin = 0.2      # extra margin to be safe

# minimum distance = robot radius + obstacle radius + safety margin
obstacle_min_dist = robot_radius + cylinder_radius + safety_margin

safe_distance = 0.4        # Nearly touch the goal

# Symbolic variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(x, y, theta)

v = ca.SX.sym('v')
omega = ca.SX.sym('omega')
controls = ca.vertcat(v, omega)

# Dynamics model
rhs = ca.vertcat(
    v * ca.cos(theta),
    v * ca.sin(theta),
    omega
)
f = ca.Function('f', [states, controls], [rhs])

# Decision variables
U = ca.SX.sym('U', n_controls, N)
X = ca.SX.sym('X', n_states, N + 1)

# Parameters: [x0, y0, theta0, tx, ty, obs1_x, obs1_y, ..., obsN_x, obsN_y]
P = ca.SX.sym('P', n_states + 2 + 2 * n_obs)

# Cost weights
Q = ca.diag([20, 20, 2])
R = ca.diag([0.8, 0.1])
cost = 0
g = []

# Initial state
X[:, 0] = P[0:3]

for k in range(N):
    st = X[:, k]
    con = U[:, k]
    st_next = X[:, k] + dt * f(st, con)
    X[:, k + 1] = st_next

    # Target position
    tx = P[3]
    ty = P[4]

    dx = st[0] - tx
    dy = st[1] - ty
    dist_to_target = ca.sqrt(dx**2 + dy**2)
    dist_error = dist_to_target - safe_distance
    cost += Q[0, 0] * dist_error**2

    # Only penalize heading when near goal (within 1m)
    fade_heading = ca.fmax(0.0, 1.0 - dist_to_target / 0.1)
    desired_theta = ca.atan2(ty - st[1], tx - st[0])
    heading_error = ca.atan2(ca.sin(st[2] - desired_theta), ca.cos(st[2] - desired_theta))
    cost += Q[2, 2] * heading_error**2 * fade_heading

    # Control effort
    cost += R[0, 0] * con[0]**2 + R[1, 1] * con[1]**2

    # Damp velocity near goal
    vel_damping = ca.exp(-2.0 * dist_to_target)
    cost += vel_damping * (2.0 * con[0]**2 + 0.5 * con[1]**2)

    # Obstacle avoidance
    for i in range(n_obs):
        obs_x = P[5 + 2 * i]
        obs_y = P[5 + 2 * i + 1]
        dist_sq = (st[0] - obs_x)**2 + (st[1] - obs_y)**2
        # Soft penalty
        cost += 10.0 / (dist_sq + 1e-2)
        # HARD constraint: keep at least obstacle_min_dist away
        g.append(dist_sq - obstacle_min_dist**2)

# Terminal cost to enforce convergence
terminal_dist = ca.sqrt((X[0, -1] - tx)**2 + (X[1, -1] - ty)**2)
cost += 30 * terminal_dist**2

# NLP setup
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
lbx = np.tile(np.array([-0.5, -2.0]), N)
ubx = np.tile(np.array([1.2, 2.0]), N)

# Constraint bounds (obstacle distances must be ≥ min)
lbg = np.zeros(N * n_obs)
ubg = np.full(N * n_obs, ca.inf)

def run_mpc(robot_state, target_pos, obstacles):
    # Stop if already near goal
    dist_to_goal = np.linalg.norm(np.array(robot_state[:2]) - np.array(target_pos))
    if dist_to_goal <= safe_distance:
        return 0.0, 0.0  # stop moving

    # Otherwise run MPC as normal
    obstacles = obstacles[:n_obs]
    while len(obstacles) < n_obs:
        obstacles.append([10.0, 10.0])  # fill with dummy far away obstacles

    flat_obs = np.array(obstacles).flatten()
    p_val = np.concatenate((robot_state, target_pos, flat_obs))
    u0 = np.zeros((n_controls * N, 1))

    sol = solver(x0=u0, p=p_val, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
    u_opt = sol['x'].full().reshape(n_controls, N)

    return float(u_opt[0, 0]), float(u_opt[1, 0])
