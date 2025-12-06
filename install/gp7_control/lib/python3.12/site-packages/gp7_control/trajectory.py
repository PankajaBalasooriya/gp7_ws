import numpy as np


def trapezoidal_1d(q0, qf, vmax, amax, dt):
    """
    1D trapezoidal profile for a single joint.
    Returns arrays of positions over time.
    """

    dq = qf - q0
    sign = np.sign(dq) if dq != 0 else 1.0
    dq_abs = abs(dq)

    # Time to accelerate to vmax
    t_acc = vmax / amax
    d_acc = 0.5 * amax * t_acc**2

    if 2 * d_acc >= dq_abs:
        # Triangle profile (never reaches vmax)
        t_acc = np.sqrt(dq_abs / amax)
        t_flat = 0.0
    else:
        # Proper trapezoid
        d_flat = dq_abs - 2 * d_acc
        t_flat = d_flat / vmax

    t_total = 2 * t_acc + t_flat
    t = np.arange(0, t_total + dt, dt)

    q = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti < t_acc:
            # Acceleration phase
            q[i] = q0 + sign * 0.5 * amax * ti**2
        elif ti < t_acc + t_flat:
            # Constant velocity
            q[i] = q0 + sign * (d_acc + vmax * (ti - t_acc))
        else:
            # Deceleration
            td = ti - (t_acc + t_flat)
            q[i] = qf - sign * 0.5 * amax * (t_total - ti)**2

    return t, q


def trapezoidal_multi(q_start, q_goal, vmax, amax, dt):
    """
    Multi-joint trapezoidal profile.
    q_start, q_goal: np.array of shape (n_joints,)
    vmax, amax: scalars (same for all joints for simplicity)
    Returns:
      t: time array
      q_traj: shape (len(t), n_joints)
    """
    q_start = np.array(q_start)
    q_goal = np.array(q_goal)
    n = len(q_start)

    # Generate each joint separately, then stretch shorter ones to match max time
    profiles = []
    max_len = 0
    max_t = 0.0

    for i in range(n):
        t_i, q_i = trapezoidal_1d(q_start[i], q_goal[i], vmax, amax, dt)
        profiles.append((t_i, q_i))
        if t_i[-1] > max_t:
            max_t = t_i[-1]
            max_len = len(t_i)

    t = np.arange(0, max_t + dt, dt)
    q_all = np.zeros((len(t), n))

    # Simple re-sampling of each joint to common time vector
    for j in range(n):
        t_i, q_i = profiles[j]
        q_all[:, j] = np.interp(t, t_i, q_i)

    return t, q_all
