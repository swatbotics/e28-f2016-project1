#!/usr/bin/env python
import roslib; roslib.load_manifest('lab1b')
import rospy
import random
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)

# change actions every 0.5s
ACTION_DURATION = rospy.Duration(0.5)

# minimum duration of safety stop
STOP_DURATION = rospy.Duration(1.0)

# define a class to handle our simple controller
class Controller:

    # initialize our controller
    def __init__(self):

        # initialize our ROS node
        rospy.init_node('starter')

        # set up publisher for commanded velocity
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                                           Twist)

        # set up subscriber for sensor state for bumpers/cliffs
        rospy.Subscriber('/mobile_base/sensors/core',
                         SensorState, self.sensor_callback)

        # set up timer to update our random action every so often
        rospy.Timer(ACTION_DURATION,
                    self.update_random_action)

        # set up our trivial 'state machine' controller
        rospy.Timer(CONTROL_PERIOD,
                    self.control_callback)

        # record whether we should stop for safety
        self.should_stop = 0
        self.time_of_stop = rospy.get_rostime() - STOP_DURATION

        # set up random wander
        self.state = 'wander'
        self.update_random_action()

    # called when sensor msgs received - just copy sensor readings to
    # class member variables
    def sensor_callback(self, msg):

        if msg.bumper & SensorState.BUMPER_LEFT:
            rospy.loginfo('***LEFT BUMPER***')
        if msg.bumper & SensorState.BUMPER_CENTRE:
            rospy.loginfo('***MIDDLE BUMPER***')
        if msg.bumper & SensorState.BUMPER_RIGHT:
            rospy.loginfo('***RIGHT BUMPER***')
        if msg.cliff:
            rospy.loginfo('***CLIFF***')

        if msg.bumper or msg.cliff:
            self.should_stop = True
            self.time_of_stop = rospy.get_rostime()
        else:
            self.should_stop = False


    # called periodically to update our current random action
    def update_random_action(self, timer_event=None):
        self.random_action = Twist()
        self.random_action.linear.x = random.uniform(0.1, 0.2)
        self.random_action.angular.z = random.uniform(-1.0, 1.0)

    # called periodically to do top-level coordination of behaviors
    def control_callback(self, timer_event=None):

        # initialize vel to 0, 0
        cmd_vel = Twist()

        time_since_stop = rospy.get_rostime() - self.time_of_stop

        if self.should_stop or time_since_stop < STOP_DURATION:
            rospy.loginfo('stopped')
        elif self.state == 'wander':
            cmd_vel = self.random_action
        else:
            rospy.logerror('invalid state {0}'.format(self.state))
                
        self.cmd_vel_pub.publish(cmd_vel)

    # called by main function below (after init)
    def run(self):
        
        # timers and callbacks are already set up, so just spin
        rospy.spin()

        # if spin returns we were interrupted by Ctrl+C or shutdown
        rospy.loginfo('goodbye')


# main function
if __name__ == '__main__':
    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
    
