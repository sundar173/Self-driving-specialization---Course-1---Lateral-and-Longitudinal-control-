# -*- coding: utf-8 -*-
"""
Created on Tue May 26 22:01:07 2020

@author: Sundar
"""


#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed
        return min_idx

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('e_old', 0.0)
        self.vars.create_var('E', 0.0)
        self.vars.create_var('Kp',9)
        self.vars.create_var('Ki',0)
        self.vars.create_var('Kd',1)
        self.vars.create_var('t_old', 0.0)
        self.vars.create_var('Ks', 1.5)
        self.vars.create_var('i',0)
        self.vars.create_var('j',0)
        self.vars.create_var('damp',1)
        self.vars.create_var('Kdd',1)
        self.vars.create_var('L',4)




        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """
            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
              """
            t_step=t-self.vars.t_old  # Finding the time step to use in PID control law equation ( Time difference between current time and old time(time logged at each time step)
            e = v_desired - v         # error in vehicle velocity
            #print ("velocity" , v_desired, " ", v)
            self.vars.E = self.vars.E + e # Summation of errors at each timestep for 'I'part of PID
            e_dot = e - self.vars.e_old  # Derivative of errors : Current errors - Old error(At the end of each time step the current error is assigned to e_old to be used in next time step)

            # Acceleration is considered to be Throttle output bypassing the Low-level Controller
            #
            throttle_output = self.vars.Kp * e + self.vars.Ki * self.vars.E * t_step+ (self.vars.Kd * (e_dot))/t_step

            # To limit the throttle between 0 and 1
            #
            throttle_output = min(max(0,throttle_output),1)
        
            # Updating Velocity
#            print ("Throttle" , throttle_output)
            

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            # throttle_output = 0
            # brake_output    = 0

            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
                """
            # To find the desired yaw (i.e The yaw of the waypoints - Calculated by finding the slope of the successive way points
            yaw_desired = np.arctan2((waypoints[-1][1] - waypoints[0][1]), (waypoints[-1][0] - waypoints[0][0]))
            # To find the yaw_error by finding the difference of yaw from yaw_desired
            yaw_e= (yaw_desired-yaw)
            # To create a list(or array) to store the distances of the current position with all the waypoints(available in this instance)
            x_vec = [x + 1.5*np.cos(yaw)] * len(waypoints)  # to create a vector with as many elements as the number of waypoints.. For x
            y_vec = [y + 1.5*np.sin(yaw)] * len(waypoints)  # to create a vector with as many elements as the number of waypoints.. For y
            
            cross_ex = (np.asarray([g[0] for g in waypoints]) - np.asarray(x_vec))**2 # (x_waypoint-x)^2
            cross_ey = (np.asarray([g[1] for g in waypoints]) - np.asarray(y_vec))**2 # (y_waypoint-y)^2
            dist = np.sqrt(cross_ex + cross_ey)]
        # To find the  minimum distance out of all the distances in the array dist and that is the cross-track error
            cross_e = min(dist)
        # To find the slope of the line joining the vehicle's current position and the waypoint --- To check if the error is positive or negative
            # (i.e. to check if the vehicle is to the left or right of the desired trajectory)

            yaw_cross_track = np.arctan2((y + 1.5*np.sin(yaw))-waypoints[0][1], (x + 1.5*np.cos(yaw))-waypoints[0][0]) #find the slope of the line joining the vehicle's current position and the waypoint
            yaw_path_cross = yaw_desired - yaw_cross_track
#
            
            if yaw_path_cross > np.pi:
                yaw_path_cross -= 2 * np.pi
            if yaw_path_cross < - np.pi:
                yaw_path_cross += 2 * np.pi


            if yaw_path_cross > 0: ## To make sure the error has the right sign since cross_e =min(dist) just gives the magnitude
                cross_e = abs(cross_e)
            else:
                cross_e = - abs(cross_e)
#
            # Implementation of Stanley Controller
            steer_output= ((self.vars.damp*yaw_e))+np.arctan2(self.vars.Ks*cross_e,v)

            # l_d=self.vars.Kdd*v  ###### Pure pursuit Controller implementation
            # alpha_star=np.arctan2(waypoints[0][1]-y,waypoints[0][0]-x)
            # alpha=alpha_star-yaw
            # # print(alpha)
            # # if alpha > np.pi:
            # #     alpha -= 2 * np.pi
            # # if alpha < - np.pi:
            # #     alpha += 2 * np.pi
            # # if v<10:
            # #     l_d=5;
            # # elif v>10 and v<50:
            # #     l_d=25;
            # # # if alpha > 0:
            # # #     alpha = abs(alpha)0
            # # # else:
            # # #     alpha = - abs(alpha)
            # #
            # delta_ld=np.arctan2(2*np.sin(alpha)*self.vars.L,l_d)
            # steer_output=delta_ld
            # delta_

            # Update equation for velocityn using Forward Euler Approximation
            v+=throttle_output*t_step
                        

            # Change the steer output with the lateral controller. 
            #steer_output    = 0.35

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)

            # To make sure the steering outputs are within limits [-1.22,1.22]
            if steer_output > 1.22:
                steer_output = 1.22
            if steer_output < -1.22:
                steer_output = -1.22
                
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)"""
# The update of the variables to be used in the next time step
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.e_old = e
        self.vars.t_old = t
        self.vars.i=self.vars.i+1
