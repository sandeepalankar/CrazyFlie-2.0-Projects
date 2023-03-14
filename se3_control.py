import numpy as np
from scipy.spatial.transform import Rotation

class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2

        # STUDENT CODE HERE
        # Control gains.
#        self.Kd =  0.5 * np.diag([0.75,0.75,1])
#        self.Kp =  0.05 * np.diag([0.75,0.75,1])
#        self.KR = 0.2 * np.eye(3)
#        self.Kw = 0.64 * np.eye(3)
#        self.Kd = np.diag([5, 5, 2])
#        self.Kp = np.diag([0.5, 0.5, 1])
#        self.KR =   np.diag([1.5, 1.5, 1.5])
#        self.Kw =   np.diag([3, 3, 3])
#        self.Kd = np.diag(np.random.normal(size=[3]))
#         self.Kd = 3 * np.eye(3)
#         self.Kp = 5 * np.eye(3)
#         self.Kp = np.diag([5, 5, 5])
#         self.Kd = np.diag([5, 5, 5])
#         self.KR =   72 * np.eye(3)
#         self.Kw =   9 * np.eye(3)
#         self.Kp = np.diag([2, 2, 4])
#         self.Kd = np.diag([5, 5, 10])
#         self.KR = np.diag([150,150,150])
#         self.Kw = np.diag([5,5,5])
#         self.Kd = 4 * np.eye(3)
#         self.Kp = 8 * np.eye(3)
#         self.Kp = np.diag([5, 5, 10])
#         self.Kd = np.diag([5, 5, 10])
# hardwrae gains
        self.Kp = np.diag([8, 8, 8])
        self.Kd = np.diag([5, 5, 5])
        self.KR = 210 * np.eye(3)
        self.Kw = 15 * np.eye(3)
#speed =1m/s
        # self.Kd = np.diag([3, 3, 3])
        # self.Kp = np.diag([6, 6, 6])
        # self.KR = np.diag([75, 75, 150])
        # self.Kw = np.diag([10, 10, 12])

#        self.Kp = 0.1*np.diag(np.random.normal(size=[3]))
#        self.KR = 0.1*np.diag(np.random.normal(size=[3]))
#        self.Kw = 0.1*np.diag(np.random.normal(size=[3]))
        l = self.arm_length
        gamma = self.k_drag/self.k_thrust
        self.linear_Eq = np.array([[1,1,1,1],[0,l,0,-l],[-l,0,l,0],[gamma,-gamma,gamma,-gamma]])




    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))

        # STUDENT CODE HERE
        x_ddot_des = flat_output['x_ddot'] - self.Kd @ (state['v'] - flat_output['x_dot']) - self.Kp @ (state['x']-flat_output['x'])
        F_des = self.mass * x_ddot_des.reshape([3,1]) + np.array([0,0,self.mass * self.g]).reshape([3,1])

        #Use scipy to create transformation matrix
        R = Rotation.from_quat(state['q']).as_matrix()

        #Convert quaternion to intrinsic Euler Z-X-Y angles
        #euler = Rotation.from_quat(state['q']).as_euler('ZXY')

        b3 = R @ np.array([0,0,1]).reshape([3,1])

        u = np.zeros([4, 1])
        u[0] = b3.T @ F_des

        b3_des = F_des / np.linalg.norm(F_des)
        a_phi = np.array([np.cos(flat_output['yaw']),np.sin(flat_output['yaw']),0]).reshape([3,1])
        b2_des = np.cross(b3_des,a_phi,axis=0) / np.linalg.norm(np.cross(b3_des,a_phi,axis=0))
        R_des = np.hstack([np.cross(b2_des,b3_des,axis=0),b2_des,b3_des])
        S = (R_des.T @ R - R.T @ R_des) / 2
        e_R = np.array([S[2,1],S[0,2],S[1,0]])
        e_w = state['w']
        u[1:] = (self.inertia @(-self.KR @ e_R - self.Kw @ e_w)).reshape([3,1])
        F = np.linalg.inv(self.linear_Eq) @ u
        F[F < 0] = 0
        w = np.sqrt(F/self.k_thrust)

        cmd_motor_speeds[:] = w.reshape([4])
        cmd_thrust = F.sum()
        cmd_moment[:] = u[1:].reshape([3])
        cmd_q = Rotation.from_matrix(R_des).as_quat()







        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
        return control_input
