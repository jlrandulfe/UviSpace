import time
import numpy as np
from pylab import *


class QuadCurveTracker():
    def __init__(self):
        # Look ahead parameters
        self.Smax = 0.500 # Maximum allowable lookahead distance
        self.rho = 0.15 # Tuning parameter 
        #
        self.Kappa = 5 # Quadratic coeficient
        # 
        self.alpha = 0.300 # Maximum speed
        # 
        self.path = None
        self.ref_point = 0
        self.dist_ahead = 0.350
        self.end_radius = 0.100

    def straight_line(self, point1, point2):
        """
		Calculates a set of intermediate points between the 2 opposite sides
		of a segment. The steps between the calculated points will be of
		about 1cm

		Parameters
		----------
		point1, point2 : numpy.ndarray of 2 elements
			cartesian 2-D coordinates of the 2 opposite sides of the segment
		"""
        x1, y1 = point1
        x2, y2 = point2
        dx, dy = np.abs(point2 - point1) * 100 # cm aprox.
        if dx > dy:
            x = np.linspace(x1, x2, dx+1)
            y = np.linspace(y1, y2, dx+1)
        else:
            x = np.linspace(x1, x2, dy+1)
            y = np.linspace(y1, y2, dy+1)
        return np.column_stack((x, y))

## Substituted by np.linalg.norm()
#    def distance(self, point1, point2):
#        dx, dy = point2 - point1
#        return np.sqrt(dx * dx + dy * dy)
        
    def append_point(self, point):
        """
		Defines the path as a set of straight segments, from the current
        location to the destination location.

		Parameters
		----------
		point : array_like object of 2 elements
			cartesian 2-D coordinates of the new destination point. Can be a 
			list a tuple or any object exposing the array interface

		Returns
		-------
		path : numpy.ndarray
			an array with a new calculated path, with the previous segments 
			and the new	one stacked together. Each segment is composed of 
			several pairs of intermediate cardinal points that the UGV must reach
		"""
        point_array = np.array(point)
        if self.path == None:
            self.path = np.array([point_array])
        else:
            new_segment = self.straight_line(self.path[-1], point_array)     
            self.path = np.vstack((self.path, new_segment))
        print 'New path:', self.path
        return self.path
    
    def find_next_point(self, point):
        """Finds the next reference point based on look ahead distance."""
        dist_ahead = self.Smax / (1 + (self.rho * self.Kappa))   
        while ( np.linalg.norm( point - self.path[self.ref_point] ) < dist_ahead):
            if self.ref_point < len(self.path) - 1:
                self.ref_point += 1
            else:
                break
        return self.path[self.ref_point]
    
    def run(self, robotX, robotY, robotO):
        cpoint = np.array([robotX, robotY])
        if self.path == None :
            speed, turn = 0, 0
        else:
            if np.linalg.norm( cpoint - self.path[-1] ) < self.end_radius:
                speed, turn = 0, 0
            else:
                ref_point = self.find_next_point(cpoint) # goal point
                print 'Ref Point:', ref_point
                delta_x, delta_y = (ref_point - cpoint) # in mm
                delta_o = np.arctan2(delta_y, delta_x) - robotO
                print 'Deltas:', delta_x, delta_y, delta_o
                e_x = delta_x * np.cos(robotO) + delta_y * np.sin(robotO)
                e_y = -delta_x * np.sin(robotO) + delta_y * np.cos(robotO)
                A = e_y / (e_x ** 2)
                K = np.sign(e_x) * self.alpha / (1 + np.abs(A))
                self.Kappa = np.abs(A)
                speed = np.round(K, 4) 
                turn = np.round(2 * A * K, 4)
        return speed, turn
        
    def tracking(self, robotX, robotY, robotO):
        cpoint = np.array([robotX, robotY])
        if self.path == None:
            v_linear, v_angular = 0, 0
        else:
            if self.distance(cpoint, self.path[-1]) < self.end_radius:
                v_linear, v_angular = 0, 0
            else:
                while (self.distance(cpoint, self.path[self.ref_point]) < self.dist_ahead):
                    if self.ref_point < len(self.path) - 1:
                        self.ref_point = self.ref_point + 1
                    else:
                        break
                ref_point = self.path[self.ref_point]
                print 'REF:', ref_point, 'CUR:', cpoint, robotO
                delta_o = 0
                if self.ref_point < len(self.path)-1: 
                    dx, dy = (self.path[self.ref_point+1] - self.path[self.ref_point])
                    delta_o = np.arctan2(dy, dx)
                # ---
                delta_x, delta_y = (ref_point - cpoint)                   
                e_x = delta_x * np.cos(robotO) + delta_y * np.sin(robotO)
                e_y = -delta_x * np.sin(robotO) + delta_y * np.cos(robotO)
                # Minimun error angle
                delta_o = np.arctan2(delta_y, delta_x)
                e_o = delta_o - robotO
                if e_o > np.pi:
                    e_o = e_o - 2 * np.pi
                if e_o < -np.pi:
                    e_o = e_o + 2 * np.pi
                # ---
                print 'Deltas:', delta_x, delta_y, delta_o
                print 'Errors:', e_x, e_y, e_o
                kvp = 0.6
                kwp = 0.3
                v_linear = kvp * e_x
                v_y = 0.3 * np.abs(e_y)
                v_o = kwp * np.abs(e_o)
                if v_linear > v_o:
                    v_linear = v_linear - v_o
                else:
                    v_linear = 0
                v_angular = kwp * e_o
        return self.speed_limits(v_linear, v_angular)
        
        
class SimulateTrackPath():
    """Uses a quadratic curve tracking algorithm to track the robot along the
    path made with set_path()."""
    def __init__(self, DT=0.1):
        self.DT = DT
        self.init()
        
    def init(self, x=0, y=0, theta=0):
        self.x, self.y, self.theta = x, y, theta
        self.QCTracker = QuadCurveTracker()
        
    def set_path(self, points):
        for point in points:
            self.path = self.QCTracker.append_point(point)
        self.track = []
    
    def current_location(self, v, w):
        self.theta = self.theta + w * self.DT
        if (self.theta) > np.pi:
            self.theta = self.theta - 2 * np.pi
        elif (self.theta) < -np.pi:
            self.theta = self.theta + 2 * np.pi
        dx = v * self.DT * np.cos(self.theta)
        dy = v * self.DT * np.sin(self.theta)
        self.x, self.y = self.x + dx, self.y + dy
        return self.x, self.y, self.theta
        
    def run(self, time):
        """Runs a simulation with a time duration."""
        track = []
        time_steps = np.linspace(0, time, int(time/self.DT))
        speeds = np.zeros((len(time_steps), 2))
        for k, t in enumerate(time_steps):
            v, w = self.QCTracker.tracking(self.x, self.y, self.theta)
            speeds[k] = ((1 + 0.7 * np.random.random()) * v, 
                         (1 + 0.4 * np.random.random()) * w)
            v, w = speeds[k-9]
            x, y, theta = self.current_location(v, w)
            print 'Time (s)', np.round(t, 2)
            print 'Location', x, y, theta           
            print 'Speeds', 'V (m/s):', v, 'W (rad/s):', w
            track.append([x, y, theta])
        self.track = np.array(track)
        
    def plot(self):
        """Plots the simulation results."""
        try:
            figure()
            grid()
            plot(self.path[:,0], self.path[:,1], 'b+')
            #plot(self.track[:,0], self.track[:,1], 'g+')    
            for x, y, o in self.track:
                arrow(x, y, np.cos(o) * 0.005, np.sin(o) * 0.005, width=0.0025, 
                      fc='r', lw=0.25)
            xlim(-1.800, 1.800)
            ylim(-1.400, 1.400)
            show()
        except:
            pass
                
            
    
if __name__ == '__main__':
    points = [[0.000, 0.200],
              [0.100, 0.400],
              [1.200, 0.733],
              [1.000, -1.000],
              [-0.980, -0.765],
              [-0.666, 0.555]]
    
    simulation = SimulateTrackPath()
    simulation.set_path(points)
    simulation.run(50)
    simulation.plot()
    
