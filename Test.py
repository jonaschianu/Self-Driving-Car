"""**************************************************************************************
*  Project:      Longitude & Lateral controllers for Waypoint tracking of vehicle       *
*                                                                                       *
*  Files:        Test.py                                                                *
*  Description:  The file consists of multiple plotting methods to offer insight into   *
*                controller performance                                                 *
**************************************************************************************"""

#************************************ NAMESPACE**************************************#
import matplotlib.pyplot as plt

#***************************** GLOBAL VARIABLES *************************************#

#***************************** FUNCTIONS DEFINITIONS*********************************#
class plots:
    def __init__(self):
        """===================================================================================
        * Function:   __init__(): initialize plots class
        * Arguments:  self
        * Returns:    N/A
        ==================================================================================="""

    @staticmethod
    def speed_profile(states):
        """===================================================================================
        * Function:   speed_profile(): Static method to setup speed vs time plot
        * Arguments:  states
        * Returns:    N/A
        ==================================================================================="""
        plt.figure()
        plt.plot(states.time_list, [iv * 3.6 for iv in states.vel_list], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.title("Speed Profile")
        plt.grid(True)

    @staticmethod
    def tracking_error(states,error):
        """===================================================================================
        * Function:   tracking_error(): Static method to setup error vs time plot
        * Arguments:  states, error
        * Returns:    N/A
        ==================================================================================="""
        plt.figure()
        plt.plot(states.time_list[:-1], error, "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Error")
        plt.title("Tracking Error")
        plt.grid(True)

    @staticmethod
    def look_distance(states):
        """===================================================================================
        * Function:   look_distance(): Static method to setup look ahead distance vs time plot
        * Arguments:  states
        * Returns:    N/A
        ==================================================================================="""
        plt.figure()
        plt.plot(states.time_list, states.Ld_list, "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Look Ahead Distance[m]")
        plt.title("Look Ahead Distance")
        plt.grid(True)

    @staticmethod
    def throttle(states):
        """===================================================================================
        * Function:   throttle(): Static method to setup vehicle throttle vs time plot
        * Arguments:  states
        * Returns:    N/A
        ==================================================================================="""
        plt.figure()
        plt.plot(states.time_list, states.throttle, "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.title("Throttle")
        plt.grid(True)

    @staticmethod
    def steering(states):
        """===================================================================================
        * Function:   steering(): Static method to setup steering angle vs time plot
        * Arguments:  states
        * Returns:    N/A
        ==================================================================================="""
        plt.figure()
        plt.plot(states.time_list, states.steering_list, "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Steering[rad]")
        plt.title("Steering Angle")
        plt.grid(True)
