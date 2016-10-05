#!/usr/bin/env python
# standard libraries
import glob
import sys
import numpy as np
# ROS libraries
import rospy
from geometry_msgs.msg import Twist
# local libraries
import speed_subscriber
import ser_message_protocol

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('base_controller', anonymous=True)
    rospy.Subscriber('/robot/cmd_vel', Twist, move_robot, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def move_robot(data, rho=0.065, L=0.150):
    v_RightWheel, v_LeftWheel = get_2WD_speeds(data.linear.x, data.angular.z)
    rospy.loginfo('I am sending R: {} L: {}'.format(v_RightWheel,
                                                      v_LeftWheel) )
    message.move([v_RightWheel, v_LeftWheel])

def get_2WD_speeds(vLinear, vRotation, minInput=-0.3, maxInput=0.3, 
                    minOutput=89, maxOutput=165, rho=0.065, L=0.150):
    """
    Obtains two speeds components, one for each side of the vehicle. It
    calculates the resulting value according to the output required 
    scale. This calculus responds to the dynamics system proposed on the 
    paper: http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=5674957

    Parameters
    ----------
    vLinear : int or float
        desired linear speed of the vehicle.

    vRotation : int or float
        desired rotatory speed of the vehicle.

    minInput : int or float
        input value for reverse linear direction at max speed.

    maxInput : int or float
        input value for direct linear direction at max speed.

    minOutput : int or float
        output value for minimum set point sent to the vehicle.

    maxOutput : int or float
        output value for maximum set point sent to the vehicle. 

    rho : float 
        Parameter of the dynamic model, which represents the vehicle's 
        wheels diameter, in meters.

    L : float
        Parameter of the dynamic model, which represents the distance 
        between the driving wheels of the vehicle.

    Returns
    -------
    v_R : 0 to 255 int
        output value for the right wheel.
        0 corresponds to max speed at reverse direction.
        255 corresponds to max speed at direct direction.
        127 corresponds to null speed.

    v_L : 0 to 255 int
        output value for the left wheel.
    """
    # Conversion of the linear speed range to the wheels angular speed.
    maxAngular = maxInput / rho
    minAngular = minInput / rho
    # Both terms are previosly calculated to reduce redundant operations.
    term1 = (1 / rho ) * vLinear
    term2 = (2 * rho * vRotation) / L
    # Calculates non-scaled raw values of the speeds.
    vR_raw = term1 + term2
    vL_raw = term1 - term2
    # Clips and scales the speed values to minOutput and maxOutput.
    v_clipped = np.clip([vR_raw, vL_raw], minAngular, maxAngular)
    v_num = (v_clipped - minAngular) * (maxOutput - minOutput)
    v_den = (maxAngular - minAngular)
    v_scaled = minOutput + v_num // v_den
    v_R, v_L = v_scaled.astype(int)
    return v_R, v_L

if __name__ == '__main__':
    # This exception prevents a crash when no device is connected to CPU
    baud = 57600
    timeout = 0.5
    try:
        port = glob.glob('/dev/ttyUSB*')[0]
    except IndexError:
        print 'It was not detected any serial port connected to PC'		
        sys.exit(1)
    # Create an object implementing serial messaging 
    message = ser_message_protocol.SerMesProtocol(port=port,
                                                  baudrate=baud, 
                                                  timeout=timeout)
    if message.ready():
        print "The board is ready"
        listener()
    else:
        print "Unable to stablish connection"



