import numpy as np

class WaypointTraj(object):
    """

    """
    def __init__(self, points):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission. For a waypoint
        trajectory, the input argument is an array of 3D destination
        coordinates. You are free to choose the times of arrival and the path
        taken between the points in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Inputs:
            points, (N, 3) array of N waypoint coordinates in 3D
        """

        # STUDENT CODE HERE
        self.waypoints = points
        self.N = points.shape[0]
        self.velocity = 0.5
        self.I_norm = np.zeros((points.shape[0], 3))
        self.distance = np.zeros(points.shape[0])
        self.duration = np.zeros(points.shape[0])
        self.t_start = np.zeros(points.shape[0])

        for i in range(self.N - 1):
            numerator = points[i+1] - points[i]
            #equation (20):
            self.I_norm[i] = numerator/np.linalg.norm(numerator)
            #equation (21):
            self.distance[i] = np.linalg.norm(numerator)
            self.duration[i] = self.distance[i]/self.velocity
            #equation (22):
            self.t_start[i+1] = self.t_start[i] + self.duration[i]

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        count, step = 0, 0

        while (count <= self.N - 1) and (t >= self.t_start[count]):
            count += 1
        step = count - 1

        #equation (23):
        x_dot = self.velocity * self.I_norm[step]

        if t > self.t_start[-1]:
            x = self.waypoints[-1]
        else:
            #equation (24):
            x = self.waypoints[step] + self.velocity * self.I_norm[step] * (t - self.t_start[step])

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output
