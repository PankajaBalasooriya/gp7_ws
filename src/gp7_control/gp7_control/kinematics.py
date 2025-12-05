import numpy as np

# =======================
# 1) DH PARAMETERS (FILL THESE!)
# =======================

# Standard DH: a, alpha, d, theta
# Replace the numbers below with YOUR robot's values (in meters & radians).
# Here I put placeholders as an example of shape only.
DH_PARAMS = [
    #  a,     alpha,    d,      theta_offset
    [0.04,     np.pi/2,  0.33,    0.0],   # Link 1
    [0.445,    0.0,      0.0,     np.pi/2],   # Link 2
    [0.04,     np.pi/2,  0.0,     0.0],   # Link 3
    [0.0,     -np.pi/2,  0.44,    0.0],   # Link 4
    [0.0,      np.pi/2,  0.0,     0.0],   # Link 5
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

def numerical_jacobian_pos(q, eps=1e-6):
    """
    Numerical Jacobian of end-effector position wrt joints.
    Returns 3x6 Jacobian.
    """
    J = np.zeros((3, 6))
    T0 = forward_kinematics(q)
    p0, _ = pose_from_T(T0)

    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        T_eps = forward_kinematics(q + dq)
        p_eps, _ = pose_from_T(T_eps)
        J[:, i] = (p_eps - p0) / eps

    return J


def inverse_kinematics_pos(target_pos,
                           q_init=None,
                           max_iters=200,
                           tol=1e-4,
                           damping=1e-3):
    """
    Numerical IK only for position (x,y,z).
    target_pos: np.array([x, y, z])
    q_init: np.array(6,), initial guess
    Returns: (q_solution, success_flag)
    """
    if q_init is None:
        q = np.zeros(6)
    else:
        q = q_init.copy()

    for _ in range(max_iters):
        T = forward_kinematics(q)
        p, _ = pose_from_T(T)

        err = target_pos - p  # 3D position error

        if np.linalg.norm(err) < tol:
            return q, True

        J = numerical_jacobian_pos(q)
        JT = J.T
        lambda_I = (damping**2) * np.eye(3)
        dq = JT @ np.linalg.inv(J @ JT + lambda_I) @ err

        q = q + dq

    return q, False
