import numpy as np

# =======================
# 1) DH PARAMETERS (FILL THESE!)
# =======================

# Standard DH: a, alpha, d, theta
# Replace the numbers below with YOUR robot's values (in meters & radians).
# Here I put placeholders as an example of shape only.
# DH_PARAMS = [
#     #  a,     alpha,    d,      theta_offset
#     [0.0,      0.0,     0.445,  0.0],   # Link 1
#     [0.45,     np.pi/2, 0.0,    0.0],   # Link 2
#     [0.44,     0.0,     0.0,    0.0],   # Link 3
#     [0.25,    np.pi/2,  0.0,    0.0],   # Link 4
#     [0.0,    -np.pi/2,  0.520,  0.0],   # Link 5
#     [0.0,     0.0,      1.217,  0.0],   # Link 6 (tool)
# ]

DH_PARAMS = [
    #  a,     alpha,    d,      theta_offset
    [0.04,     np.pi/2,  0.33,    0.0],   # Link 1
    [0.445,    0.0,      0.0,     np.pi/2],   # Link 2
    [0.04,     np.pi/2,  0.0,     0.0],   # Link 3
    [0.0,      np.pi/2,  0.44,    0.0],   # Link 4
    [0.0,      -np.pi/2,  0.0,     0.0],   # Link 5
    [0.0,      0.0,      0.08,    0.0],   # Link 6 (tool)
]


def dh_transform(a, alpha, d, theta):
    """Single standard DH transform."""
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,     sa,       ca,      d],
        [0.0,    0.0,      0.0,    1.0]
    ])


def forward_kinematics(joint_angles):
    """
    joint_angles: iterable of 6 joint values [q1..q6] in radians.
    Returns: 4x4 homogeneous transform of end-effector (T_0_6).
    """
    assert len(joint_angles) == 6, "Need 6 joint angles"

    T = np.eye(4)
    for i, q in enumerate(joint_angles):
        a, alpha, d, theta_offset = DH_PARAMS[i]
        theta = q + theta_offset
        A_i = dh_transform(a, alpha, d, theta)
        T = T @ A_i

    return T


def pose_from_T(T):
    """
    Extract position (x,y,z) and RPY from a homogeneous transform.
    """
    pos = T[0:3, 3]
    R = T[0:3, 0:3]
    # Simple RPY from rotation matrix (ZYX)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
    roll = np.arctan2(R[2, 1], R[2, 2])
    return pos, np.array([roll, pitch, yaw])


def numerical_jacobian(q, eps=1e-6):
    """
    Numerical Jacobian of end-effector position & orientation (RPY) wrt joints.
    Returns 6x6 Jacobian.
    """
    J = np.zeros((6, 6))
    T0 = forward_kinematics(q)
    p0, rpy0 = pose_from_T(T0)
    y0 = np.concatenate([p0, rpy0])

    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        T_eps = forward_kinematics(q + dq)
        p_eps, rpy_eps = pose_from_T(T_eps)
        y_eps = np.concatenate([p_eps, rpy_eps])
        J[:, i] = (y_eps - y0) / eps

    return J


def inverse_kinematics(target_pos, target_rpy,
                       q_init=None,
                       max_iters=200,
                       tol=1e-4,
                       damping=1e-3):
    """
    Numerical IK: solve for joint angles that reach target pose.

    target_pos: np.array([x, y, z])
    target_rpy: np.array([roll, pitch, yaw])
    q_init: initial guess (np.array(6,)), default zeros
    Returns: (q_solution, success_flag)
    """
    if q_init is None:
        q = np.zeros(6)
    else:
        q = q_init.copy()

    for _ in range(max_iters):
        T = forward_kinematics(q)
        p, rpy = pose_from_T(T)
        y = np.concatenate([p, rpy])

        y_target = np.concatenate([target_pos, target_rpy])
        err = y_target - y

        if np.linalg.norm(err) < tol:
            return q, True

        J = numerical_jacobian(q)
        JT = J.T
        # Damped least-squares: (J^T J + λ^2 I)^-1 J^T e
        lambda_I = (damping**2) * np.eye(6)
        dq = JT @ np.linalg.inv(J @ JT + lambda_I) @ err

        q = q + dq

    return q, False

def inverse_kinematics_position_only(target_pos,
                                     q_init=None,
                                     max_iters=200,
                                     tol=1e-4,
                                     damping=1e-3):
    """
    Numerical IK that matches only the end-effector POSITION (x,y,z).
    Orientation is ignored. This is often more robust if the DH model
    is approximate or the desired orientation is hard to reach.

    target_pos: np.array([x, y, z])
    q_init: initial guess (np.array(6,))
    Returns: (q_solution, success_flag)
    """
    if q_init is None:
        q = np.zeros(6)
    else:
        q = q_init.copy()

    for _ in range(max_iters):
        T = forward_kinematics(q)
        p, rpy = pose_from_T(T)

        err_pos = target_pos - p  # 3D position error

        if np.linalg.norm(err_pos) < tol:
            return q, True

        # Full numerical Jacobian (6x6)
        J = numerical_jacobian(q)
        # Position part only: top 3 rows (∂x, ∂y, ∂z / ∂q)
        J_pos = J[0:3, :]  # 3x6
        JT_pos = J_pos.T   # 6x3

        # Damped least-squares on position only:
        # dq = J_pos^T (J_pos J_pos^T + λ^2 I)^-1 err_pos
        lambda_I = (damping**2) * np.eye(3)
        dq = JT_pos @ np.linalg.inv(J_pos @ JT_pos + lambda_I) @ err_pos

        q = q + dq

    return q, False



# if __name__ == "__main__":
#     # Simple FK test
#     import numpy as np

#     # Example joint config in radians
#     q_test = np.deg2rad([0, 0, 0, 0, 0, 0])
#     T = forward_kinematics(q_test)
#     pos, rpy = pose_from_T(T)

#     print("q_test (deg):", [0, 0, 0, 0, 0, 0])
#     print("End-effector position:", pos)
#     print("End-effector RPY (rad):", rpy)

if __name__ == "__main__":
    import numpy as np

    # 1) Test FK at zero
    q_zero = np.deg2rad([0, 0, 0, 0, 0, 0])
    T_zero = forward_kinematics(q_zero)
    pos_zero, rpy_zero = pose_from_T(T_zero)
    print("=== FK test at q = 0 ===")
    print("End-effector position:", pos_zero)
    print("End-effector RPY (rad):", rpy_zero)
    print()

    # 2) Pick a random-ish joint configuration
    q_test = np.deg2rad([10, 20, -30, 40, -20, 10])
    T = forward_kinematics(q_test)
    pos, rpy = pose_from_T(T)
    print("=== FK at q_test ===")
    print("q_test (rad):", q_test)
    print("FK position:", pos)
    print("FK rpy:", rpy)
    print()

    # 3) IK to recover q_test (consistency check)
    q_sol, ok = inverse_kinematics(pos, rpy, q_init=np.zeros(6))
    print("=== IK result ===")
    print("IK success:", ok)
    print("q_sol:", q_sol)
    print("difference (q_sol - q_test):", q_sol - q_test)
