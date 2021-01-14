"""**************************************************************************************
*  Project:      Longitude & Lateral controllers for Waypoint tracking of vehicle       *
*                                                                                       *
*  Files:        controller.py                                                          *
*  Description:  The file consists of 2 controllers namely PID and Pure Pursuit.        *
*                PID controller is responsible for longitudinal control of vehicle      *
*                while the Pure Pursuit controller controls the vehicle in lateral      *
*                direction.                                                             *
**************************************************************************************"""


#************************************ NAMESPACE**************************************#
import math

#***************************** GLOBAL VARIABLES *************************************#
Kp = 0.5    # Proportional gain
Kd = 0.1   # Derivative gain
Ki = 0.15    # Integral gain
dt = 0.1    # Time Interval
WB = 2.9    # Rear length of vehicle [m].

#***************************** FUNCTIONS DEFINITIONS*********************************#
class Controller:

    def __init__(self):
        """===================================================================================
        * Function:   __init__(): Initiazlies values for controller output
        * Arguments:  self
        * Returns:    N/A
        ==================================================================================="""
        self.Ld=[]          # Look ahead distances list (Lookahead_dist+state.vel*K)
        self.delta=[]       # Steering angle list from purepursuit output
        self.throttle=[]    # Throttle list from PID output  
        self.ind=[]         # Index list for tracking

        self.DTerm=0
        self.PTerm=0
        self.ITerm=0
        self.last_error = 0
    
    # def PID(self, desired, actual):
    #     """===================================================================================
    #     * Function:   PID(): Longitudnal controller that calculates car throttle for adjusting speed
    #     * Arguments:  self, desired, actual
    #     * Returns:    float
    #     ==================================================================================="""
    #
    #     error = desired - actual
    #     throttle = Kp*error + Kd*error/dt + Ki*error*dt #From Tuned PID values
    #
    #     # Add throttle history to list
    #     self.throttle.append(throttle)
    #
    #     # Return throttle to be added/subtracted to state.vel
    #     return throttle

    def PID(self,desired, actual):
        """===================================================================================
        * Function:   PID(): Longitudnal controller that calculates car throttle for adjusting speed
        * Arguments:  self, desired, actual
        * Returns:    float
        ==================================================================================="""

        error = desired - actual
        delta_error = error - self.last_error
        self.last_error=error

        self.PTerm = Kp * error

        self.ITerm += error * dt

        self.DTerm = 0.0
        self.DTerm = delta_error / dt

        throttle = self.PTerm + Kd*self.DTerm + Ki*self.ITerm

        # Add throttle history to list
        self.throttle.append(throttle)

        # Return throttle to be added/subtracted to state.vel
        return throttle

    def pure_pursuit(self, state, trajectory, pind):
        """===================================================================================
        * Function:   pure_pursuit(): Latitude controller that calculates steering angle for car
        * Arguments:  self, state, trajectory, pind
        * Returns:    float, int
        ==================================================================================="""
        # Get look ahead distance Ld & the corresponding waypoint index from TargetCourse class 
        ind, Ld = trajectory.search_target_index(state)

        if pind >= ind: # Dont exceed the index
            ind = pind

        if ind < len(trajectory.wp_x):
            target_wp_x = trajectory.wp_x[ind]
            target_wp_y = trajectory.wp_y[ind]
        else:  # Towards goal
            target_wp_x = trajectory.wp_x[-1]
            target_wp_y = trajectory.wp_y[-1]
            ind = len(trajectory.wp_x) - 1

        # Alpha Calculations
        angle_car_to_target = math.atan2(target_wp_y - state.y_rear, 
            target_wp_x - state.x_rear)

        alpha = angle_car_to_target - state.yaw

        # Steering angle calculation
        delta = math.atan2(2.0 * WB * math.sin(alpha), Ld)

        # Add the history to list
        self.Ld.append(Ld)
        self.delta.append(delta)
        self.ind.append(ind)

        # Return steering angle and the corresponding index
        return delta, ind
