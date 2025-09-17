"""
MIT License, Kemal Ficici, 2018
github.com/kemfic
"""
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class process(object):
    process_response = 0

    def __init__(self, const_shift=0, noise=False):
        self.constant_shift = const_shift/512.0
        self.noise = noise

    def update(self, controller_response, current_state):
        self.process_response = self.process_response*0.96 - 0.01*controller_response + self.constant_shift +self.noise*((np.random.rand()-0.5)/100)
        self.process_response = min(1, self.process_response)
        self.process_response = max(-1, self.process_response)
        return current_state + self.process_response

def error(current_state, set_state):
    return current_state - set_state
    
class Plant(object):
    
    def __init__(self, control, d_t=0.1, t_max=60, set_steady=False, set_shift=False, set_sin=False, noise=False):
        self.delta_t = d_t
        self.t_max = t_max
        self.t = np.arange(0, self.t_max, self.delta_t)
        if set_sin:
            self.set_states = np.sin(self.t*4*np.pi/self.t_max)
        elif set_steady:
            self.set_states = np.zeros_like(self.t)
        else:
            self.set_states = np.zeros_like(self.t)
            self.set_states[200:400] = 1
        
        self.cur_process = process(set_shift, noise)        
        self.controller = control

    def simulate(self, plot=True):
        states = np.zeros_like(self.t)
        states[0] = -1
        errors = np.zeros_like(self.t)
        errors[0] = error(states[0], self.set_states[0])

        for i in range(1, len(self.t)):
            controller_response = self.controller.update(errors[i-1], delta_t=self.delta_t)
            states[i] = self.cur_process.update(controller_response, states[i-1])
            errors[i] = error(states[i], self.set_states[i])

        errortotal = np.sum(np.abs(errors))

        # store for later use
        self.states = states
        self.errors = errors
        self.errortotal = errortotal

        if plot:
            plt.plot(states, color='blue', label='environment state')
            plt.plot(self.set_states, color='red', label='set (desired) state')
            plt.plot([], color='none', label='total error: ' + str(errortotal)[:9])
            plt.xlabel('Time')
            plt.ylabel('State')  # was 'Error'
            plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
            plt.show()

        # also return arrays for convenience
        return self.t, states, errors, self.set_states


class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0,
                 u_min=None, u_max=None,          # output saturation
                 i_min=None, i_max=None,          # integrator clamp
                 d_alpha=0.0,                     # 0..1 derivative low-pass (0 = no filter)
                 reverse=True):                   # reverse action (good for your plant sign)
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.u_min = u_min
        self.u_max = u_max
        self.i_min = i_min
        self.i_max = i_max
        self.d_alpha = float(d_alpha)
        self.dir = -1.0 if reverse else 1.0  # reverse action flips sign of output
        self.reset()

    def reset(self):
        self._integ = 0.0
        self._prev_err = 0.0
        self._prev_d = 0.0
        self._init = False

    def update(self, err, delta_t=0.01):
        dt = float(delta_t) if delta_t is not None else 0.01

        # P
        P = self.kp * err

        # I (tentative integrate then clamp)
        integ_new = self._integ + err * dt
        if self.i_min is not None: integ_new = max(self.i_min, integ_new)
        if self.i_max is not None: integ_new = min(self.i_max, integ_new)
        I = self.ki * integ_new

        # D on error + optional filtering
        if not self._init:
            de = 0.0
            self._init = True
        else:
            de = (err - self._prev_err) / dt
        D_raw = self.kd * de
        D = self.d_alpha * self._prev_d + (1.0 - self.d_alpha) * D_raw

        # Unsaturated control (apply direct/reverse action)
        u_unsat = self.dir * (P + I + D)

        # Saturate
        u = u_unsat
        if self.u_min is not None: u = max(self.u_min, u)
        if self.u_max is not None: u = min(self.u_max, u)

        # Simple anti-windup: only commit integrator if not saturating further in same direction
        saturated_high = (self.u_max is not None) and (u_unsat > self.u_max)
        saturated_low  = (self.u_min is not None) and (u_unsat < self.u_min)
        if (saturated_high and self.dir*(P + D) >= 0 and err > 0):
            pass  # don't update integ (would push further into sat)
        elif (saturated_low and self.dir*(P + D) <= 0 and err < 0):
            pass
        else:
            self._integ = integ_new  # accept new integral

        # Save for next step
        self._prev_err = err
        self._prev_d = D
        return u


# Gains and limits — your process uses -0.01 * controller_response, so scale u generously
pid = PIDController(
    kp=80.0, ki=20.0, kd=5.0,
    u_min=-200.0, u_max=200.0,   # output saturation
    i_min=-10.0, i_max=10.0,     # integrator clamp
    d_alpha=0.2,                 # derivative filter (0..1)
    reverse=True                 # reverse action matches your plant sign
)

env = Plant(control=pid, d_t=0.1, t_max=60, set_steady=False, set_shift=False, set_sin=False, noise=False)
env.simulate()

if __name__ == "__main__":
    pid = PIDController(
        kp=80.0, ki=20.0, kd=5.0,
        u_min=-200.0, u_max=200.0,
        i_min=-10.0, i_max=10.0,
        d_alpha=0.2, reverse=True
    )
    env = Plant(control=pid, d_t=0.1, t_max=60, set_steady=False, set_shift=False, set_sin=False, noise=False)

    # run the sim and plot
    t, states, errors, set_states = env.simulate(plot=False)

    plt.figure()
    plt.plot(t, states, label='environment state')
    plt.plot(t, set_states, label='set (desired) state')
    plt.xlabel('Time'); plt.ylabel('State'); plt.legend(); plt.grid(True)

    plt.figure()
    plt.plot(t, errors, color='purple', label='tracking error')
    plt.xlabel('Time'); plt.ylabel('Error'); plt.legend(); plt.grid(True)
    plt.show()
