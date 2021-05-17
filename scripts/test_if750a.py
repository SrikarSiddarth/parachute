#!/usr/bin/env python
from __future__ import print_function
# rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD'"
# rosservice call /mavros/cmd/arming "value: true"
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget
from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import SetMode, CommandBool
from mavros import command
from mavros.utils import *

class TestParachute():
    def __init__(self):
        rospy.init_node('TestParachute',anonymous=True)

        self.curr_pose = PoseStamped()
        self.des_pose = PoseStamped()
        self.vel = Twist()
        self.des_pose.pose.position.x = 0
        self.des_pose.pose.position.y = 0
        self.des_pose.pose.position.z = 0
        self.des_pose.pose.orientation.x = 0
        self.des_pose.pose.orientation.y = 0
        self.des_pose.pose.orientation.z = 0
        self.des_pose.pose.orientation.w = 0
        self.arm = True
        self.cmd_mode = 'OFFBOARD'
        self.is_ready_to_fly = False
        self.state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_callback)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)

        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 10)
        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.trig_pub = rospy.Publisher('/parachute_trigger', Bool, queue_size=10)

        self.controller()
        # rospy.spin()

    def state_callback(self, msg):
        self.set_mode()
        if (self.arm == True and msg.armed == False) or (self.arm == False and msg.armed == True):
            self.set_arm(self.arm)


    def set_arm(self,flag):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            armService(flag)
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)

    def set_mode(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            isModeChanged = flightModeService(custom_mode=self.cmd_mode)
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    def do_kill(self):
        try:
            ret = command.long(command=400, param2=21196)
        except rospy.ServiceException as ex:
            fault(ex)

        if not ret.success:
            fault("Request failed. Check mavros logs")

        print_if("Command result:", ret.result)
        return ret

    def pose_callback(self,msg):
        self.curr_pose = msg

    def controller(self):
        r = rospy.Rate(10)
        x = 0
        while not rospy.is_shutdown():
            self.des_pose.pose.position.z = 10

            if x == 0:

                if self.curr_pose.pose.position.z>9:
                    # self.des_pose.pose.position.x = 10
                    # if self.curr_pose.pose.position.x>8:
                    #     self.arm=False
                    #     self.cmd_mode = 'AUTO.LAND'
                    #     self.set_offboard_mode()
                    # self.do_kill()
                    x = 1
                else:
                    self.pose_pub.publish(self.des_pose)
            elif x ==1:
                # self.vel.linear.y = -100
                # self.vel_pub.publish(self.vel)

                self.cmd_mode = 'MANUAL'
                self.set_mode()
                # self.des_pose.pose.orientation.z = 0.7
                self.trig_pub.publish(1)

                x = 2

            self.pose_pub.publish(self.des_pose)
            r.sleep()



if __name__=='__main__':
    TestParachute()