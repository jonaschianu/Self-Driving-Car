"""**************************************************************************************
*  Project:      Longitude & Lateral controllers for Waypoint tracking of vehicle       *
*                                                                                       *
*  Files:        main.py                                                                *
*  Description:  The file contains the main body of code with imports from Test.py and  *
*                controller.py. Here the acquired waypoints (x,y coords and velocity)   *
*                are read from a text file and stored as an array. In a while loop,     *
*                the target waypoint index is calculated using TargetCourse() class,    *
*                and the waypoint at this index acts as reference targets for the PID   *
*                and Pure Pursuit controller which output steering and throttle values. *
*                The controller output is then used to update and store the state       *
*                variables for the vehicle over time, after which, live plotting is     *
*                performed. We generate additional plots for better insights of         *
*                simulation using Test.py() class for tracking error, steering angle,   *
*                throttle and velocity plots.                                           *
**************************************************************************************"""

#************************************ NAMESPACE**************************************#
import numpy as np
#import matplotlib.pyplot as plt
import pandas as pd
from controller import *
from Test import *

#***************************** GLOBAL VARIABLES *************************************#
dt = 0.1  # Sampling Time [s]
WB = 2.9  # Rear length of vehicle [m]. Also change in controller.py
lookahead_dist = 3.0  # [m] Initial Lookahead distance. For tuning
K_look = 0.1  # Lookahead gain new_lookahead = vel*K + lookahead_dist.

#***************************** FUNCTIONS DEFINITIONS*********************************#
def main():
    """===================================================================================
        * Function:   main(): main function block for code that manipulates waypoints, 
          receives controller output and does plotting
        * Arguments:  N/A
        * Returns:    N/A
        ==================================================================================="""

    # Stores waypoints from the text file
    waypoints = []
    # Stores tracking errors between target waypoint and actual vehicle position
    tracking_errors = []

    # Create a list of every waypoint set (x,y,velocity)
    line_list = [line.rstrip('\n') for line in open('waypoint_carla.txt')]

    # Creates a list of a lists of waypoint data
    for i in range(len(line_list)):
        line = line_list[i].split(',')

        for j in range(len(line)):
            line[j] = float(line[j])

        waypoints.append(line)

    # Creates a two-dimensional table with the columns representing x, y, velocity
    wp_table = pd.DataFrame(np.array(waypoints))

    wp_x = np.array(wp_table[0])  # Global x-coordinate of vehicle center [m]
    wp_y = np.array(wp_table[1])  # Global y-coordinate of vehicle center[m]
    target_speed = np.array(wp_table[2])  # Velocity[m/s]

    T = 300.0  # Max simulation time
    last_index = len(wp_x) - 1 # Max waypoint index
    time = 0
    throttle = 0
    steering = 0

    yaw=0
    # Calculation of initial yaw
    for i in range(len(wp_x)-1):
        if wp_x[i+1]!=wp_x[i] or wp_y[i+1]!=wp_y[i]:
            yaw=math.atan2( wp_y[i+1]-wp_y[i] ,  wp_x[i+1]-wp_x[i])
            break

    # Initial state of vehicle at the beginning of the simulation
    state = State(x=wp_x[0], y=wp_y[0], yaw=yaw) # Adjust yaw manually based on initial orientation

    # Initializing states
    states = States()
    controller = Controller()
    states.add(time, throttle, steering, state)
    target_course = TargetCourse(wp_x, wp_y)

    # Find the first target index with initial states, usually returns 0 index
    target_ind, _ = target_course.search_target_index(state)

    control = Controller()
    # Loop until time is less than set sim time value & ind lower than waypoint length
    while T >= time and last_index > target_ind:
        time += dt  # dt = 0.1sec
        
        # Calculate control output from PID & Pure_Pursuit class for target waypoint & vehicle state
        throttle = control.PID(target_speed[target_ind], state.vel)
        steering, target_ind= control.pure_pursuit(state, target_course, target_ind)

        # Update the steering angle and velocity from PID & Pure Pursuit controller
        state.update_state(throttle, steering)
        # Store the steering angle and velocity controller output history in a list 
        states.add(time, throttle, steering, state)

        # Sqrt sum of X and Y axis = tracking error
        off_track = np.hypot(wp_x[target_ind]-states.x_list[-1], wp_y[target_ind]-states.y_list[-1])
        tracking_errors.append(off_track)

        # Live plotting stuff! Commented out if not in use.
        # If show_animation: 
        plt.cla()
        # For stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        plot_arrow(state.wp_x, state.wp_y, state.yaw)
        plt.plot(wp_x, wp_y, ".r", label="course")
        plt.plot(states.x_list, states.y_list, "-b", label="trajectory")
        plt.plot(wp_x[target_ind], wp_y[target_ind], "xg", label="target")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[km/h]:" + str(state.vel * 3.6)[:4]) # m/s converted to km/hr
        plt.pause(0.001)

    # Other plots for analysis from the plots class in test.py. Commented out if not in use. 
    plots.speed_profile(states)
    plots.tracking_error(states, tracking_errors)
    plots.throttle(states)
    plots.steering(states)
    plt.show()

#***************************** FUNCTIONS DEFINITIONS*********************************#
class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        """===================================================================================
        * Function:   __init__(): Initiazlies values for vehicle state variables 
        * Arguments:  self, waypoint_x coord, waypoint_y coord, vehicle yaw
        * Returns:    N/A
        ==================================================================================="""
        self.wp_x = x          # Waypoint X
        self.wp_y = y          # Waypoint Y
        self.yaw = yaw      
        self.vel = 0           # Vehicle Velocity

        # Extracting X and Y coordinates of vehicle rear [m]
        self.x_rear = self.wp_x - ((WB / 2) * math.cos(self.yaw))
        self.y_rear = self.wp_y - ((WB / 2) * math.sin(self.yaw))

    def update_state(self, throttle, steer_angle):
        """===================================================================================
        * Function:   update_state(): Adjusted values for vehicle state variables 
        * Arguments:  self, throttle, steering angle
        * Returns:    N/A
        ==================================================================================="""
        # Adjusting state of vehicle as per controller output. dt = 0.1
        # Only velocity needed to run for Carla, rest are for updating vehicle location for ploting.
        self.vel += throttle * dt
        self.yaw += (self.vel / WB * math.tan(steer_angle)) * dt
        self.wp_x += self.vel * math.cos(self.yaw) * dt
        self.wp_y += self.vel * math.sin(self.yaw) * dt
        self.x_rear = self.wp_x - ((WB / 2) * math.cos(self.yaw))
        self.y_rear = self.wp_y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        """===================================================================================
        * Function:   calc_distance(): Calculates distance from rear of vehicle to target point
        * Arguments:  self, point_x, point_y
        * Returns:    float
        ==================================================================================="""
        # Get X and Y axis distances between rear wheel of car to the target waypoint location
        dx = self.x_rear- point_x
        dy = self.y_rear- point_y
        # Return the hypotenuse 
        return math.hypot(dx, dy)

#***************************** FUNCTIONS DEFINITIONS*********************************#
class States:

    def __init__(self):
        """===================================================================================
        * Function:   __init__(): Initiazlies list values for vehicle state variables 
        * Arguments:  self
        * Returns:    N/A
        ==================================================================================="""
        self.x_list = []
        self.y_list = []
        self.yaw_list = []
        self.vel_list = []
        self.time_list = []
        self.steering_list = []
        self.throttle = []

    def add(self, time, throttle, steering, state):
        """===================================================================================
        * Function:   add(): stores values for vehicle state as list from the controller output
        * Arguments:  self, time, throttle, steering, state
        * Returns:    N/A
        ==================================================================================="""
        self.x_list.append(state.wp_x)
        self.y_list.append(state.wp_y)
        self.yaw_list.append(state.yaw)
        self.vel_list.append(state.vel)
        self.time_list.append(time)
        self.steering_list.append(steering)
        self.throttle.append(throttle)

#***************************** FUNCTIONS DEFINITIONS*********************************#
class TargetCourse:

    def __init__(self, x, y):
        """===================================================================================
        * Function:   __init__(): Initiazlies waypoint coordinates for calculating target point
        * Arguments:  Waypoint X coord, Waypoint Y coord
        * Returns:    N/A
        ==================================================================================="""
        self.wp_x = x
        self.wp_y = y
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        """===================================================================================
        * Function:   search_target_index(): Determines the target waypoint at look ahead distance
        * Arguments:  Self, vehicle state class variables
        * Returns:    Waypoint index at look ahead distance, new look ahead dist based on velocity
        ==================================================================================="""
        
        # We do this if statement for the first time
        if self.old_nearest_point_index is None:
            # search nearest point index
            # Use the rear axle coordinate and iterate through all the pts
            # then find minimum hypotenuse, the lowest of which is our first index
            dx = [state.x_rear - icx for icx in self.wp_x]
            dy = [state.y_rear - icy for icy in self.wp_y]
            distance = np.hypot(dx, dy)
            ind = np.argmin(distance) # Index at which minimum distance occurs
            self.old_nearest_point_index = ind # Usually 0
        else:  
            # This condition used once we have established starting index
            ind = self.old_nearest_point_index
            # Hypotenuse distance from rear axle to first index
            distance_this_index = state.calc_distance(
                self.wp_x[ind], self.wp_y[ind])

            while True:
                # Iterate through next indexes and get hypotenuse dist from current rear axle pos
                # Keep looping until the further WP is at greater distance from car than current WP
                # i.e. Car on the right trajectory
                distance_next_index = state.calc_distance(self.wp_x[ind + 1], self.wp_y[ind + 1])
                
                if distance_this_index < distance_next_index:
                    break

                # i++ but shouldnt exceed length of waypoint 
                if (ind + 1) < len(self.wp_x):
                    ind = ind + 1

                distance_this_index = distance_next_index

            self.old_nearest_point_index = ind

        # Update dynamic look ahead distance based on velocity of vehicle (from PID)
        lookahead_new = K_look * state.vel + lookahead_dist

        # Loop until the index that matches the look ahead distance is found i.e. target waypoint
        while lookahead_new > state.calc_distance(self.wp_x[ind], self.wp_y[ind]):
            if (ind + 1) >= len(self.wp_x):
                break  # Do not exceed goal
            ind += 1

        return ind, lookahead_new

def plot_arrow(wp_x, wp_y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """===================================================================================
        * Function:   plot_arrow(): This draws an arrow from (x, y) to (x+dx, y+dy)
        * Arguments:  x, y, yaw, display: length, width, colors: fc="r", ec="k"
        * Returns:    N/A
        ==================================================================================="""

    if not isinstance(wp_x, float):
        print(wp_x)
        for ix, iy, iyaw in zip(wp_x, wp_y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(wp_x, wp_y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(wp_x, wp_y)

if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
