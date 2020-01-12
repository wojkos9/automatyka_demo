def control_system(t0, tmax, m=750, b=180.9, y0=0, v0=0, target=100, v_opt=3.5, f_max=3000, use_r_h=True):
    global dt, g
    n = int((tmax-t0) // dt)
    if use_r_h:
        r_h = y0
    else:
        r_h = target
        v_opt = 0.0

    rhs = [r_h, r_h + v_opt*dt]
    t = t0
    f_add = m*g
    t0, t1 = 0, dt
    y1 = y0 + v0 * dt
    ts, ys = [t0, t1], [y0, y1]
    fs = [0, 0]
    vs = [v0, v0]
    pid = PID(50, 0.05, 50)
    for _ in range(n):
        t += dt
        if use_r_h:
            if r_h < target:
                r_h += v_opt*dt
                if r_h >= target: use_r_h = False
            if r_h > target:
                r_h -= v_opt*dt
                if r_h <= target: use_r_h = False
        rhs += [r_h]
        e = r_h - y1
        f = pid(e, dt)
        l = max(min(f, f_max) + f_add, 0) - m*g
        y = (m*(2*y1 - y0) + l*(dt**2) - b*abs(y1-y0)*(y1-y0)) / m  # drag b is bidirectional
        y = max(y, 0)
        ts += [t]
        ys += [y]
        y0 = y1
        y1 = y

    return ts, ys, rhs, fs, vs
