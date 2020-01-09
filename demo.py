import matplotlib.pyplot as plt


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.e_sum = 0
        self.last_e = 0

    def __call__(self, e, dt):
        self.e_sum += e*dt
        v = self.Kp*e + self.Ki*self.e_sum + self.Kd * (e - self.last_e) / dt
        self.last_e = e
        return v


dt = 0.01
g = 9.81


def control_system(t0, tmax, m=750, b=180.9, y0=0, v0=0, target=100, pp=(50, 0.05, 50), use_r_h=True, mt=(0, 0)):
    global dt, g
    n = int((tmax-t0) // dt)
    r_h = 0  # reference height
    t = t0
    f_add = m*g
    t0, t1 = 0, dt
    y1 = y0 + v0 * dt
    ts, ys = [t0, t1], [y0, y1]
    v_opt = 3.5
    rhs = [0, v_opt*dt]
    f_max = 3000
    pid = PID(*pp)
    for _ in range(n):
        t += dt
        if t < mt[1] <= t+dt:
            print(t, ":", m, mt[0])
            m = mt[0]

        if use_r_h:
            if r_h < target:
                r_h += v_opt*dt
        else:
            r_h = target
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

    return ts, ys, rhs


def plot(t0, tmax, **kwargs):
    ts1, ys1, rhs = control_system(t0, tmax, **kwargs)

    le = ", ".join([k+"="+str(kwargs[k]) for k in kwargs.keys()])
    plt.xlabel("t")
    plt.ylabel("h(t)")
    plt.plot(ts1, rhs, ":")
    plt.plot(ts1, ys1, label=le)
    plt.legend()


plot(0, 200, m=750)
plot(0, 200, m=7500)
plt.show()


plot(0, 200, target=25)
plot(0, 200, target=100)
plot(0, 200, target=500)
plt.show()

plot(0, 200, b=180.9)
plot(0, 200, b=0)
plt.show()

plot(0, 1000, target=2000, use_r_h=True)
plot(0, 1000, target=2000, use_r_h=False)
plt.show()
