import numpy as np


# DONE Part 3: Comment the code explaining each part
class kalman_filter:
    # DONE Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P = P
        self.Q = Q
        self.R = R
        self.x = x
        self.dt = dt
        
    # DONE Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):
        self.A = self.jacobian_A() # Obtain current A matrix through linearization of motion model
        self.C = self.jacobian_H() # Obtain current C matrix through linearization of observation model
        
        self.motion_model()
        
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # DONE Part 3: Replace the matrices with Jacobians where needed
    def update(self, z):

        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.R
        S = np.array(S, dtype=np.float64)
            
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        surprise_error= z - self.measurement_model()
        
        self.x=self.x + np.dot(kalman_gain, surprise_error)
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)
        
    # DONE: Part 3: Implement here the measurement model
    def measurement_model(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,# v
            w,# w
            vdot, # ax. NOTE: Assumed vdot is always aligned with x axis
            v * w, # ay. NOTE: Used centripetal accel for ay
        ])
        
    # DONE Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        
    # DONE
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y, th, w, v, vdot
            [1, 0, -v * np.sin(th) * dt, 0, np.cos(th) * dt, 0], # partial deriv of x w.r.t state variables
            [0, 1, v * np.cos(th) * dt, 0, np.sin(th) * dt,  0], # partial deriv of y w.r.t state variables
            [0, 0, 1, dt, 0, 0], # partial deriv of theta w.r.t state variables
            [0, 0, 0, 1, 0, 0], # partial deriv of omega w.r.t state variables
            [0, 0, 0, 0, 1, dt], # partial deriv of v w.r.t state variables
            [0, 0, 0, 0, 0, 1] # partial deriv of vdot w.r.t state variables
        ])
    
    # DONE Part 3: Implement here the jacobian of the H matrix (measurements)  
    def jacobian_H(self):
        x, y, th, w, v, vdot = self.x
        return np.array([
            #x, y, th, w, v, vdot
            [0, 0, 0, 0, 1, 0], # partial deriv of v w.r.t state variables
            [0, 0, 0, 1, 0, 0], # partial deriv of w w.r.t state variables
            [0, 0, 0, 0, 0, 1], # partial deriv of ax w.r.t state variables
            [0, 0, 0, v, w, 0], # partial deriv of ay w.r.t state variables
        ])
        
    # DONE Part 3: return the states here    
    def get_states(self):
        return self.x
