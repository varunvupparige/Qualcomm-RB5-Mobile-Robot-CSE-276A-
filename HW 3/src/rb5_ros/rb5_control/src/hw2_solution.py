#!/usr/bin/env python
import sys
import roslib
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import numpy as np
import math
import tf
import tf2_ros
from tf.transformations import quaternion_matrix

"""
The class of the pid controller.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.02

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def getCurrentPos(l):
    """
    Given the tf listener, we consider the camera's z-axis is the header of the car
    """
    br = tf.TransformBroadcaster()
    result = None
    foundSolution = False

    for i in range(0, 9):
        camera_name = "camera_" + str(i)
        if l.frameExists(camera_name):
            try:
                now = rospy.Time()
                # wait for the transform ready from the map to the camera for 1 second.
                l.waitForTransform("map", camera_name, now, rospy.Duration(1.0))
                # extract the transform camera pose in the map coordinate.
                (trans, rot) = l.lookupTransform("map", camera_name, now)
                # convert the rotate matrix to theta angle in 2d
                matrix = quaternion_matrix(rot)
                angle = math.atan2(matrix[1][2], matrix[0][2])
                # this is not required, I just used this for debug in RVIZ
                br.sendTransform((trans[0], trans[1], 0), tf.transformations.quaternion_from_euler(0,0,angle), rospy.Time.now(), "base_link", "map")
                result = np.array([trans[0], trans[1], angle])
                foundSolution = True
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                print("meet error")
    listener.clear()
    return foundSolution, result


def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    """
    Convert the twist into the car coordinate
    """
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)

def ekf_predict(mu_t_1,sigma_t_1,u_t_1):
    
    mu_t = np.dot(F_pred, mu_t_1) + np.dot(G_pred, u_t_1)
    sigma_t = np.linalg.multi_dot([F_pred, sigma_t_1, np.transpose(F_pred)]) + W_pred

    return mu_t, sigma_t

def ekf_update(l,mu_t,sigma_t):
    """
    Given the tf listener, we consider the camera's z-axis is the header of the car
    """
    #br = tf.TransformBroadcaster()
    result = None
    foundSolution = False
    counter = np.zeros(8)

    for i in range(0, 8):
        #camera_name = "camera_" + str(i)
        marker_name = "marker" + str(i)
        print(i)
        if l.frameExists(marker_name):
            print(marker_name)
            try:
                now = rospy.Time()
                # wait for the transform ready from the map to the camera for 1 second.
                l.waitForTransform("camera", marker_name, now, rospy.Duration(1.0))
                # extract the transform camera pose in the map coordinate.
                (trans, rot) = l.lookupTransform("camera", marker_name, now)
                # convert the rotate matrix to theta angle in 2d
                matrix = quaternion_matrix(rot)
                theta = math.atan2(matrix[1][2], matrix[0][2])
                angle = math.atan2(trans[0],trans[1]) - theta
                #tag_pose = np.array([trans[0], trans[1], angle])
                z_t = np.array([np.sqrt(trans[0]**2 + trans[1]**2),angle])
                
                counter[i] += 1

                if counter[i] == 1:
                    mu_t[2*i+3] = mu_t[0] + z_t*math.cos(angle + mu_t[2])
                    mu_t[2*i+4] = mu_t[1] + z_t*math.sin(angle + mu_t[2])
                
                yeta_x = mu_t[2*i+3] - mu_t[0]
                yeta_y = mu_t[2*i+4] - mu_t[1]

                yeta = np.array([yeta_x,yeta_y])
                q = np.dot(np.transpose(yeta),yeta)
                z_cap_t = np.array([np.sqrt(q),math.atan2(yeta_y, yeta_x) - mu_t[2]])

                F_x_j = np.zeros([5,2*no_l + 3])
                F_x_j[:3][:3] = 1
                F_x_j[4][2*(2*i-2)-1] = 1 
                F_x_j[5][2*(2*i-2)] = 1

                Jacobian_H = np.array([[-np.sqrt(q)*yeta_x, -np.sqrt(q)*yeta_y, 0,np.sqrt(q)*yeta_x, np.sqrt(q)*yeta_y]
                                       [yeta_y, -yeta_x, -q, -yeta_y, yeta_x ]])
                H_t = 1/q * np.dot(Jacobian_H, F_x_j)

                K_gain = np.multi_dot([sigma_t,np.transpose(H_t),np.linalg.inv(np.multi_dot([H_t, sigma_t, np.transpose(H_t)]))])
                mu_t = mu_t + K_gain(z_t - z_cap_t)
                sigma_t = np.dot((np.eye(2*no_l+3) - np.dot(K_gain, H_t )), sigma_t)
        
                        
                foundSolution = True
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                print("meet error")
    listener.clear()
    return mu_t, sigma_t


if __name__ == "__main__":
    
    import time
    rospy.init_node("hw2")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    #kalman init parameters
    no_l = 8
    mu_t = np.zeros(2*no_l+3)
    sigma_t = np.zeros([2*no_l+3,2*no_l+3])
    np.fill_diagonal(sigma_t, 1e6)
    sigma_t[0][0] = 0
    sigma_t[1][1] = 0
    sigma_t[2][2] = 0

    F_pred = np.eye(2*no_l+3)

    G_pred = np.zeros([2*no_l+3,2*no_l+3])
    G_pred[0][0] = 1
    G_pred[1][1] = 1
    G_pred[2][2] = 1

    W_pred = np.zeros([2*no_l+3,2*no_l+3])
    # motion noise covariance for x,y,theta
    W_pred[0][0] = 0.4
    W_pred[1][1] = 0.4
    W_pred[2][2] = 0.3

    R_upd = np.zeros([2,2])
    R_upd[0][0] = 0.2
    R_upd[1][1] = 0.2

    listener = tf.TransformListener()

    waypoint = np.array([[0.0,0.0,0.0], 
                         [1.5,0.0,0.0]])

    # init pid controller
    pid = PIDcontroller(0.1,0.001,0.007)

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        
        u_t = np.zeros(2*no_l+3)
        u_t[0] = update_value[0]
        u_t[1] = update_value[1]
        u_t[2] = update_value[2]
        
        mu_pred, sigma_pred = ekf_predict(mu_t,sigma_t,u_t)
        mu_update, sigma_update = ekf_update(listener,mu_pred,sigma_pred)       

        estimated_state = np.array([mu_update[0],mu_update[1],mu_update[2]])

        #if found_state: # if the tag is detected, we can use it to update current state.
        current_state = estimated_state
        
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
          
            u_t[0] = update_value[0]
            u_t[1] = update_value[1]
            u_t[2] = update_value[2]

            mu_pred, sigma_pred = ekf_predict(mu_t,sigma_t,u_t)
            mu_update, sigma_update = ekf_update(listener,mu_pred,sigma_pred)       
            
            print("state vector after update \n")
            print(mu_update)
            
            
            #print("covariance matrix after update \n")
            #print(sigma_update)
            estimated_state = np.array([mu_update[0],mu_update[1],mu_update[2]])

            estimated_state = current_state

    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))



