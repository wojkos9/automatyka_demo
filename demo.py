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


def plot1(t0, tmax, **kwargs):
    ts, ys, rhs, fs, vs = control_system(t0, tmax, **kwargs)
    plt.title("Wysokość balonu w funkcji czasu")
    plt.xlabel("t [s]")
    plt.ylabel("y(t) [m]")
    plt.plot(ts, rhs, ":", label="wysokość optymalna")
    plt.plot(ts, ys, label="wysokość rzecziwysta")
    plt.legend()


def plot(t0, tmax, **kwargs):
    ts1, ys1, rhs = control_system(t0, tmax, **kwargs)

    le = ", ".join([k+"="+str(kwargs[k]) for k in kwargs.keys()])
    plt.xlabel("t")
    plt.ylabel("h(t)")
    plt.plot(ts1, rhs, ":")
    plt.plot(ts1, ys1, label=le)
    plt.legend()


plot1(0, 200, m=7500, y0=150)
plt.show()


exit(0)

#----------------------------------------------------------------------------

plt.title("Mała masa balonu vs duża")
plot(0, 200, m=750)
plot(0, 200, m=7500)
plt.show()

plt.title("Różne docelowe wysokości")
plot(0, 200, target=25)
plot(0, 200, target=100)
plot(0, 200, target=500)
plt.show()

plt.title("Opór powietrza vs brak oporu")
plot(0, 200, b=180.9)
plot(0, 200, b=0)
plt.show()

plt.title("Zmienna wartość docelowa vs stała")
plot(0, 1000, target=2000, use_r_h=True)
plot(0, 1000, target=2000, use_r_h=False)
plt.show()
