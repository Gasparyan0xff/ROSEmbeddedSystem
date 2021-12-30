#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

pub_cmd_vel = None
MINICAR_MAX_LIN_VEL = 1
MINICAR_MAX_ANG_VEL = 2
UPDATE_VEL = 0.25
UPDATE_ANG = 1.1
MIN_RANGE = 0.01
MAX_RANGE = 2.35
RANGE_THRESHOLD = 1.1
FRONT_RANGE_THRESHOLD = 0.45
GAIN_ANG = 5
range_front1 = MAX_RANGE
range_front2 = MAX_RANGE

twist = Twist()

def update_velocity(angular, velocity):
    global twist
    if (velocity != None):
        target_linear_vel = MINICAR_MAX_LIN_VEL * velocity
        twist.linear.x = target_linear_vel
    if (angular != None):
        target_angular_vel = MINICAR_MAX_ANG_VEL * angular
        twist.angular.z = target_angular_vel
    return


def callback_ir_front0(data):
    global pub_cmd_vel
    range_value = data.range
    is_left = range_front1 <= RANGE_THRESHOLD
    is_right = range_front2 <= RANGE_THRESHOLD
    ### update linear velocity
    if (range_value > FRONT_RANGE_THRESHOLD and not is_left and not is_right):
        update_velocity(None, UPDATE_VEL)
    else:
        update_velocity(None, 0)

    ### update angular velocity
    if (is_left and is_right):
        if (range_front1 < range_front2):
            update_velocity(GAIN_ANG*UPDATE_ANG, None)
        else:
            update_velocity(-GAIN_ANG*UPDATE_ANG, None)
    elif(is_left and range_front2 > RANGE_THRESHOLD):
        update_velocity(UPDATE_ANG, None)
    elif(is_right and range_front1 > RANGE_THRESHOLD):
        update_velocity(-UPDATE_ANG, None)
    else:
        update_velocity(0, None)

    ### publish result velocity
    pub_cmd_vel.publish(twist)


def callback_ir_front1(data):
    global range_front1
    range_front1 = data.range
    
def callback_ir_front2(data):
    global range_front2
    range_front2 = data.range
    

def callback_cmd_vel(data):
    rospy.loginfo('Robot linear velocity is a %s', data.linear)
    rospy.loginfo('Robot angular velocity is a %s', data.angular)
    

def run_ir_sensor_reader():
    global pub_cmd_vel
    rospy.init_node('ir_sensors_control', anonymous=True)
    pub_cmd_vel = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=10) 
    rospy.Subscriber('/myrobot/sensor/front_0', Range, callback_ir_front0)
    rospy.Subscriber('/myrobot/sensor/front_1', Range, callback_ir_front1)
    rospy.Subscriber('/myrobot/sensor/front_2', Range, callback_ir_front2)

    #rospy.Subscriber('/controller/cmd_vel', Twist, callback_cmd_vel)
    rospy.spin()
    return

def init_velocity():
    global twist
    twist.linear.x = 0.0
    twist.linear.y = 0.0 
    twist.linear.z = 0.0
    twist.angular.x = 0.0 
    twist.angular.y = 0.0
    twist.angular.z = 0.0


# Точка входа в программу
if __name__ == '__main__':
    try:
        init_velocity()
        run_ir_sensor_reader()
    except rospy.ROSInterruptException as err:
        print('Error! %s' % err)
        pass

