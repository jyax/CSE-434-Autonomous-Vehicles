'''
    Very simple PID controller
    Has option to input state-rate rather than calculating this numerically

    Daniel Morris, 2022, 2023
'''

class pid_controller():
    def __init__(self, kp, ki, kd):
        ''' Can optionally require a state rate input
            This avoids latencies from numerical calculation of the derivative
        '''
        self.kp = kp
        self.ki = ki
        self.kd = kd       
        self.previous_time_sec = None
        self.previous_error = 0.
        self.previous_target = None
        self.I_error = 0

    def update_control(self, target, state, current_time_sec):
        ''' Will calculate derivative numerically '''

        current_error = target - state

        if self.previous_time_sec:
            dt = current_time_sec - self.previous_time_sec
        else:
            dt = 0
        
        # Numerical integration
        self.I_error +=  (self.previous_error + current_error) * dt / 2

        if self.previous_error and dt > 0:        
            # Numerical differentiation
            D_error = ( current_error - self.previous_error ) / dt
        else:
            D_error = 0
        
        self.previous_time_sec = current_time_sec
        self.previous_error = current_error
        self.previous_target = target

        u = self.kp * current_error + self.ki * self.I_error + self.kd * D_error

        return (u, current_error, self.I_error, D_error)

    def update_control_with_rate(self, target, state, state_rate, current_time_sec):
        ''' Uses state rate as part of the derivative '''

        current_error = target - state

        if self.previous_time_sec:
            dt = current_time_sec - self.previous_time_sec
        else:
            dt = 0

        # Numerical integration       
        self.I_error +=  (self.previous_error + current_error) * dt / 2

        # Use state_rate instead of differencing state -- can be more stable
        if self.previous_target and dt > 0:
            D_error = (target - self.previous_target) / dt - state_rate
        else:
            D_error = -state_rate
        
        self.previous_time_sec = current_time_sec
        self.previous_error = current_error
        self.previous_target = target

        u = self.kp * current_error + self.ki * self.I_error + self.kd * D_error

        return (u, current_error, self.I_error, D_error)


