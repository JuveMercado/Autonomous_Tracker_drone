#!/usr/bin/env python2.7
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
odom = Odometry()
odom_my = Odometry()
rospy.init_node('Follower', anonymous=True)

def callback(data):
    global odom
    odom = data

def callback_owm(data):
    global odom_my
    odom_my = data

def copy_path():
    #OvR = rospy.Publisher()
    global odom, odom_my, rad
    pubLand1_ = rospy.Publisher('/q2/real/land', Empty,  queue_size=10)
    pubTakeoff1_ = rospy.Publisher('/q2/real/takeoff', Empty,  queue_size=10)
    pubCommandPilot1_ = rospy.Publisher('/q2/real/cmd_vel', Twist,  queue_size=10)
    CommandPilot1_ = Twist()
    #Orienta = Pose()
    msgEmpty = Empty()
    ori = 0.0
    ext = False
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo("x {0} y {1} z {2}".format(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z))

        #CommandPilot1_.linear.x = 0.0
        #CommandPilot1_.linear.y = 0.0
        #CommandPilot1_.linear.z = 0.0
        #CommandPilot1_.angular.z = 0.0

        if odom.twist.twist.linear.x != 0 and odom.twist.twist.linear.y != 0 and odom.twist.twist.linear.z != 0 and ext != True:
            while pubTakeoff1_.get_num_connections() < 1:
                rospy.loginfo("Start takeoff")
                rospy.sleep(0.1)
            pubTakeoff1_.publish(msgEmpty)
            rospy.sleep(5)
            ext = True
        elif odom.twist.twist.linear.x == 0 and odom.twist.twist.linear.y == 0 and odom.twist.twist.linear.z == 0 and ext == True:
            pubLand1_.publish(msgEmpty)
            rospy.sleep(5)
            ext = False
        
        if odom.twist.twist.linear.x > 0.1:
            CommandPilot1_.linear.x = 0.3
        elif  odom.twist.twist.linear.x < -0.1:
            CommandPilot1_.linear.x = -0.3
        else:
            CommandPilot1_.linear.x = 0.0

        if  odom.twist.twist.linear.y > 0.1 :
            CommandPilot1_.linear.y = 0.3
        elif odom.twist.twist.linear.y < -0.1:
            CommandPilot1_.linear.y = -0.3
        else:
            CommandPilot1_.linear.y = 0.0

        if  odom.twist.twist.linear.z > 0.1 :
            CommandPilot1_.linear.z = 1
        elif odom.twist.twist.linear.z < -0.1:
            CommandPilot1_.linear.z = -1
        else:
            CommandPilot1_.linear.z = 0.0
        
        '''
        if odom.pose.pose.orientation.w+0.05 > ori:
            CommandPilot1_.angular.z = 0.1
        elif odom.pose.pose.orientation.w-0.05 < ori:
            CommandPilot1_.angular.z = -0.1
        else:
            CommandPilot1_.angular.z = 0.0
        '''
        

            #rospy.sleep(0.1)
        pubCommandPilot1_.publish(CommandPilot1_)
        

            


if __name__ == '__main__':
    try:
        rospy.Subscriber("/q1/real/odom", Odometry, callback)
        rospy.Subscriber("/q2/real/odom", Odometry, callback_owm)
        copy_path()
        rospy.spin()    
    except rospy.ROSInterruptException as e:
        pass