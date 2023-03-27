#!/usr/bin/env python
#!/usr/bin/env python
import sys
import rospy
import numpy as np
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from april_detection.msg import AprilTagDetectionArray
from tf.transformations import euler_from_quaternion

class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.1

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
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)

twist_msg = Twist()

def callback(data):

    global twist_msg

    quat_orientation = data.detections[0].pose.orientation
    quat_orientation_array = [quat_orientation.x, quat_orientation.y, quat_orientation.z, quat_orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(quat_orientation_array)

    #rospy.loginfo("roll: %s, pitch: %s, yaw = %s", roll, pitch, yaw)
    twist_msg.linear.x = data.detections[0].pose.position.z
    twist_msg.linear.y = data.detections[0].pose.position.x
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = pitch
    #temp_state = np.array([data.detections[0].pose.position.z, data.detections[0].pose.position.x, pitch])

    #print("current state = ", current_state)

if __name__ == "__main__":
    
    rospy.init_node("key_joy")
    sub = rospy.Subscriber("/apriltag_detection_array", AprilTagDetectionArray, callback, queue_size = 1)
    pub = rospy.Publisher("/cris", Twist, queue_size = 1 )

    r = 0.025
    lx = 0.055
    ly = 0.07
    calibration = 50.0

    mark_3_loc = [1.54, 0, np.pi]
    mark_0_loc = [2.27, 0, np.pi]
    mark_8_loc = [2.6, 0, np.pi]
    tag_coord = [mark_3_loc, mark_3_loc, mark_0_loc, mark_8_loc, mark_8_loc]

    jacobian_matrix = np.array([[1, -1, -(lx + ly)],
                                [1, 1, (lx + ly)],
                                [1, 1, -(lx + ly)],
                                [1, -1, (lx + ly)]]) / r


    #print(current_state)

  
    waypoint = np.array([[0.0, 0.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [2.0, 0.0, 0.0],
                         [2.23, 0.0, 0.0]])
                    
    pid = PIDcontroller(0.02, 0.005, 0.005)

    current_state = np.array([0.0,0.0,0.0])
    ref_state = np.array([1.3,0.0,0.0])

    global temp_state
    temp_state = np.array([twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z])

    for wp in waypoint:

        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        #temp_state = np.array([twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z])
        current_state = ref_state - temp_state
        update_value = pid.update(current_state)

        # publish the twist
        pub.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)

        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            
            # calculate the current twist
            #temp_state = np.array([twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z])
            print("Temp_State = ", temp_state)
            current_state = ref_state - temp_state
            print("Current_State = ", current_state)
            update_value = pid.update(current_state)
            
            # publish the twist
            pub.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            print("\n counter")

            twist_cmd = genTwistMsg(coord(update_value, current_state))
            desired_twist = calibration*np.array([[twist_cmd.linear.x], [twist_cmd.linear.y], [twist_cmd.angular.z]])
            result = np.dot(jacobian_matrix, desired_twist)
            print(result)
            
            time.sleep(0.05)

        pid.itr += 1
        
        pub_twist.publish(genTwistMsg(np.array([0.0, 0.0, 0.0])))
        time.sleep(1)
        pub_twist.publish(genTwistMsg(np.array([0.0, 0.0, 0.1])))
        
        #openloop rotations
        if pid.itr == 1:
            timestep = 0
        
        elif pid.itr == 2:
            timestep = 1.58
        
        elif pid.pt_counter == 3:
            timestep = 2.85
        
        else:
            timestep = 2.45
        
        time.sleep(timestep)
        
        pub_twist.publish(genTwistMsg(np.array([0.0, 0.0, 0.0])))

    # stop the car and exit
    print("loop exiting and car stop!")
    pub.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
