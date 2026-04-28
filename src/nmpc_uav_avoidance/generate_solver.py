#!/usr/bin/env python3
"""
OpEn NMPC Solver Generator — C Bindings Version (Dynamic Obstacle)
===================================================================
Parameter vector layout (total = 138):
  [0:8]     x0 = [px,py,pz, vx,vy,vz, phi,theta]
  [8:11]    u_prev = [T, phi_ref, theta_ref]_{k-1}
  [11:131]  p_ref = 40 reference positions (3×40=120)
  [131:134] p_obs = [px_obs, py_obs, pz_obs]   ← dynamic obstacle position
  [134:137] v_obs = [vx_obs, vy_obs, vz_obs]   ← obstacle velocity (NEW)
  [137]     r_obs
"""

import casadi as cs
import opengen as og
import numpy as np
import os

N = 40
Ts = 0.05
nx = 8
nu = 3

tau_phi = 0.5
tau_theta = 0.5
K_phi = 1.0
K_theta = 1.0
Ax, Ay, Az = 0.1, 0.1, 0.2
g = 9.81

Qx = [5.0, 5.0, 5.0, 3.0, 3.0, 3.0, 8.0, 8.0]
Qu = [5.0, 10.0, 10.0]
QdU = [5.0, 12.0, 12.0]

u_ref_vals = [g, 0.0, 0.0]

u_min = [5.0, -0.35, -0.35]
u_max = [13.5, 0.35, 0.35]

DELTA_PHI_MAX   = 0.08
DELTA_THETA_MAX = 0.08
R_S_MAX         = 0.5

# [0:8] x0, [8:11] u_prev, [11:131] p_ref×40,
# [131:134] p_obs, [134:137] v_obs, [137] r_obs
N_PARAMS = 8 + 3 + 3 * N + 3 + 3 + 1  # = 138
n_z = nu * N  # = 120


def uav_dynamics_euler(x, u):
    px, py, pz = x[0], x[1], x[2]
    vx, vy, vz = x[3], x[4], x[5]
    phi, theta = x[6], x[7]

    T_thrust    = u[0]
    phi_ref_u   = u[1]
    theta_ref_u = u[2]

    ax = T_thrust * cs.cos(phi) * cs.sin(theta)
    ay = -T_thrust * cs.sin(phi)
    az = T_thrust * cs.cos(phi) * cs.cos(theta) - g

    px_n    = px + Ts * vx
    py_n    = py + Ts * vy
    pz_n    = pz + Ts * vz
    vx_n    = vx + Ts * (ax - Ax * vx)
    vy_n    = vy + Ts * (ay - Ay * vy)
    vz_n    = vz + Ts * (az - Az * vz)
    phi_n   = phi   + Ts / tau_phi   * (K_phi   * phi_ref_u   - phi)
    theta_n = theta + Ts / tau_theta * (K_theta * theta_ref_u - theta)

    return cs.vertcat(px_n, py_n, pz_n, vx_n, vy_n, vz_n, phi_n, theta_n)


def build_problem():
    z = cs.SX.sym('z', n_z)
    p = cs.SX.sym('p', N_PARAMS)

    x0     = p[0:8]
    u_prev = p[8:11]

    p_ref_start = 11
    p_obs_start = p_ref_start + 3 * N   # 131
    v_obs_start = p_obs_start + 3        # 134  (NEW)
    r_obs_idx   = v_obs_start + 3        # 137  (NEW)

    p_obs = p[p_obs_start : p_obs_start + 3]   # obstacle position at t=0
    v_obs = p[v_obs_start : v_obs_start + 3]   # obstacle velocity (constant)
    r_obs = p[r_obs_idx]                        # obstacle radius

    u_ref = cs.vertcat(*u_ref_vals)

    cost        = 0.0
    constraints = []
    x_k         = x0

    for j in range(N):
        uj     = z[j * nu : (j + 1) * nu]
        uj_prev = u_prev if j == 0 else z[(j - 1) * nu : j * nu]

        # Predict obstacle position at future time step j
        # p_obs_j = p_obs + j * Ts * v_obs
        p_obs_j = p_obs + j * Ts * v_obs

        # Reference for this timestep
        p_ref_j = p[p_ref_start + j * 3 : p_ref_start + j * 3 + 3]
        x_ref_j = cs.vertcat(p_ref_j, cs.SX.zeros(5, 1))

        # State cost — trajectory tracking
        dx = x_ref_j - x_k
        for i in range(nx):
            cost += Qx[i] * dx[i] ** 2

        # Input cost
        du_ref = u_ref - uj
        for i in range(nu):
            cost += Qu[i] * du_ref[i] ** 2

        # Input-rate cost
        du = uj - uj_prev
        for i in range(nu):
            cost += QdU[i] * du[i] ** 2

        # Spherical obstacle — use predicted position
        r_s_j   = R_S_MAX * j / N
        dp      = x_k[0:3] - p_obs_j
        dist_sq = cs.dot(dp, dp)
        h_obs   = cs.fmax(0.0, (r_obs + r_s_j) ** 2 - dist_sq)
        constraints.append(h_obs)

        # Input-rate constraints
        constraints.append(cs.fmax(0.0, uj_prev[1] - uj[1] - DELTA_PHI_MAX))
        constraints.append(cs.fmax(0.0, uj[1] - uj_prev[1] - DELTA_PHI_MAX))
        constraints.append(cs.fmax(0.0, uj_prev[2] - uj[2] - DELTA_THETA_MAX))
        constraints.append(cs.fmax(0.0, uj[2] - uj_prev[2] - DELTA_THETA_MAX))

        x_k = uav_dynamics_euler(x_k, uj)

    # Terminal cost
    p_ref_N = p[p_ref_start + (N - 1) * 3 : p_ref_start + (N - 1) * 3 + 3]
    x_ref_N = cs.vertcat(p_ref_N, cs.SX.zeros(5, 1))
    dx = x_ref_N - x_k
    for i in range(nx):
        cost += Qx[i] * dx[i] ** 2

    # Terminal obstacle constraint — obstacle at t = N*Ts
    p_obs_N = p_obs + N * Ts * v_obs
    r_s_N   = R_S_MAX
    dp      = x_k[0:3] - p_obs_N
    dist_sq = cs.dot(dp, dp)
    h_obs_N = cs.fmax(0.0, (r_obs + r_s_N) ** 2 - dist_sq)
    constraints.append(h_obs_N)

    c = cs.vertcat(*constraints)
    return z, p, cost, c


def generate():
    z, p, cost, c = build_problem()

    bounds = og.constraints.Rectangle(u_min * N, u_max * N)

    problem = (
        og.builder.Problem(z, p, cost)
        .with_penalty_constraints(c)
        .with_constraints(bounds)
    )

    meta = og.config.OptimizerMeta().with_optimizer_name("nmpc_uav_obstacle")

    build_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "nmpc_solver")

    build_config = (
        og.config.BuildConfiguration()
        .with_build_directory(build_dir)
        .with_build_mode("release")
        .with_build_c_bindings()
    )

    solver_config = (
        og.config.SolverConfiguration()
        .with_tolerance(1e-4)
        .with_max_outer_iterations(4)
        .with_max_inner_iterations(500)
        .with_penalty_weight_update_factor(10.0)
        .with_initial_penalty(1.0)
    )

    builder = og.builder.OpEnOptimizerBuilder(
        problem, meta, build_config, solver_config
    )
    builder.build()

    print("\n[OK] Solver compiled with C bindings (dynamic obstacle).")
    print(f"     Parameters : {N_PARAMS}  (138)")
    print(f"     Decision variables: {n_z}")
    print(f"     [131:134] p_obs, [134:137] v_obs, [137] r_obs")
    print()


if __name__ == "__main__":
    generate()