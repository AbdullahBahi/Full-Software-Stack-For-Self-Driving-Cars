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
        min_idx = 0
        min_dist = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                self._waypoints[i][0] - self._current_x,
                self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints) - 1:
            self._desired_speed = self._waypoints[min_idx][2]
            return min_idx
        else:
            self._desired_speed = self._waypoints[-1][2]
            x = -1
            return x

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
        min_index = self.update_desired_speed()
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

        # Throttle to engine torque
        a_0 = 400
        a_1 = 0.1
        a_2 = -0.0002

        # Gear ratio, effective radius, mass + inertia
        GR = 0.35
        r_e = 0.3
        J_e = 10
        m = 2000
        g = 9.81

        # Aerodynamic and friction coefficients
        c_a = 1.36
        c_r1 = 0.01

        #PID Gains
        k_p = 1 / 1.13
        k_i = k_p / 10

        #latteral controller gains
        k = 2
        k_s = 10

        self.vars.create_var('sum_integral', 0.0)
        self.vars.create_var('steer_output_old', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################

            # ====================================================
            # feed forward controller
            # ====================================================
            # calculate F_load and T_e respectively
            # F_load calculations
            f_aero = c_a * (v_desired ** 2)
            r_x = c_r1 * v_desired
            f_g = m * g * np.sin(0)
            f_load = f_aero + r_x + f_g

            # T_e calculation (assuming t_e = t_load)
            t_e = GR * r_e * f_load

            # calculate engine speed w_e
            w_e = v_desired / (GR * r_e)

            # now update throttle according to the updated engine speed w_e
            throttle_forward = t_e / (a_0 + (a_1 * w_e) + (a_2 * w_e ** 2))

            # ====================================================
            # feedback controller
            # ====================================================

            sample_time = 1 / 30                                                         # time step = 1 / FPS
            v_error = v_desired - v
            self.vars.sum_integral = self.vars.sum_integral + ( v_error * sample_time )  #integration term is turned into submission
            throttle_feedback = ( k_p * v_error ) + ( k_i * self.vars.sum_integral )

            throttle_output = throttle_feedback + throttle_forward
            brake_output    = 0

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################

            x_desired = waypoints[min_index][0]
            y_desired = waypoints[min_index][1]

            count = len(range(min_index, len(waypoints)))
            #########################################################
            # Calculation of heading error : psi = path_heading - yaw
            #########################################################
            if min_index != -1 :
                if count > 9 :
                    dx = waypoints[min_index+10][0] - x_desired
                    dy = waypoints[min_index+10][1] - y_desired
                else :
                    dx = x_desired - waypoints[min_index - 10][0]
                    dy = y_desired - waypoints[min_index - 10][1]

            else :
                dx = x_desired - waypoints[min_index - 10][0]
                dy = y_desired - waypoints[min_index - 10][1]

            path_heading = np.arctan2(dy,dx)

            heading_error = path_heading - yaw

            ############################################################
            # Calculation of cross track error : e = sqrt( dx^2 + dy^2 )
            ############################################################
            dx = x - x_desired
            dy = y - y_desired
            cross_track_error = np.sqrt((dx**2)+(dy**2))

            #the following codeblock finds out the vehicle position with respect to path (i.e. either on the left or on the right)
            if np.sin(path_heading) < 0 :
                if dx > 0:                                                                          # vehicle on right of the path
                    if cross_track_error < 0.1:
                        cross_track_error_term = 0
                    else:
                        cross_track_error_term = -np.arctan((k * cross_track_error) / (k_s + v))    # steer left
                else:                                                                               # vehicle on left of the path
                    if cross_track_error < 0.1:
                        cross_track_error_term = 0
                    else:
                        cross_track_error_term = np.arctan((k * cross_track_error) / (k_s + v))     # steer right
            else:
                if dx < 0:                                                                          # vehicle on right of the path
                    if cross_track_error < 0.1:
                        cross_track_error_term = 0
                    else:
                        cross_track_error_term = -np.arctan((k * cross_track_error) / (k_s + v))    # steer left
                else:                                                                               # vehicle on left of the path
                    if cross_track_error < 0.1:
                        cross_track_error_term = 0
                    else:
                        cross_track_error_term = np.arctan((k * cross_track_error) / (k_s + v))     # steer right

            steer_output = heading_error + cross_track_error_term

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

            print (yaw,"   ",path_heading,"   ",heading_error,"   ",cross_track_error_term,"   ",steer_output)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        self.vars.steer_output_old = steer_output






