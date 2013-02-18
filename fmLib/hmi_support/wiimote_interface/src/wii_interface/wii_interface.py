#!/usr/bin/env python
import rospy
import math
from msgs.msg import StringStamped
from sensor_msgs.msg import Joy,JoyFeedback,JoyFeedbackArray
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
import smach
import smach_ros

class WiiInterface():
    """
        Wiimote interface.
        Button B is deadman button
        Button 1 enters automode
        Button 2 exits automode
        Rumble feedback upon under voltage
        When not in automode, remote controlled driving is active.
    """
    def __init__(self):
        # Setup parameters
        self.automode = False
        self.deadman = Bool(False)
        self.linear = 0
        self.angular = 0
        self.next_state_change = rospy.Time.now() + rospy.Duration(1)
        self.rumble_on = False
        self.warning = False
        self.pitch = [0 , 0 , 0 , 0 , 0 , 0 , 0, 0 , 0 , 0]
        self.roll = [0 , 0 , 0 , 0 , 0 , 0 , 0, 0 , 0 , 0]
        self.ptr = 0
        self.twist = TwistStamped()
        self.fb = JoyFeedbackArray( array=[JoyFeedback( type=JoyFeedback.TYPE_LED, intensity=0, id=0 ), 
                                           JoyFeedback( type=JoyFeedback.TYPE_LED, intensity=0, id=1 ),
                                           JoyFeedback( type=JoyFeedback.TYPE_LED, intensity=0, id=2 ), 
                                           JoyFeedback( type=JoyFeedback.TYPE_LED, intensity=0, id=3 ), 
                                           JoyFeedback( type=JoyFeedback.TYPE_RUMBLE, intensity=0, id=0 )] ) 

        # Get parameters
        self.max_linear_velocity = rospy.get_param("~max_linear_velocity",2)
        self.max_angular_velocity = rospy.get_param("~max_angular_velocity",1)
        self.deadband = rospy.get_param("~deadband",0.1)
        self.publish_frequency = rospy.get_param("~publish_frequency",5)    
        
        # Get topic names
        self.deadman_topic = rospy.get_param("~deadman_topic",'deadman')
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic",'cmd_vel')
        self.feedback_topic = rospy.get_param("~feedback_topic",'joy/set_feedback') 
        self.joy_topic = rospy.get_param("~joy_topic",'joy')
        self.status_topic = rospy.get_param("~status_topic",'status')     

        # Setup topics
        self.deadman_pub = rospy.Publisher(self.deadman_topic, Bool)
        self.twist_pub = rospy.Publisher(self.cmd_vel_topic, TwistStamped)
        self.fb_pub = rospy.Publisher(self.feedback_topic, JoyFeedbackArray)
        self.joy_sub = rospy.Subscriber(self.joy_topic, Joy, self.onJoy )
        self.status_sub = rospy.Subscriber(self.status_topic, StringStamped , self.onStatus)
        
    def onJoy(self,msg):
        """
            Callback method handling Joy messages
        """
        # Handle automode buttons
        if msg.buttons[0] == 1 :
            self.automode = True
        if msg.buttons[1] == 1 :
            self.automode = False
            
        # Handle deadman button
        if msg.buttons[3] == 1 :
            self.deadman = True
        else :
            self.deadman = False      
             
        # Generate pitch and roll and save value in list
        self.pitch[self.ptr] = math.atan2(msg.axes[1], math.sqrt(math.pow(msg.axes[0], 2) + math.pow(msg.axes[2], 2))) / 1.57;
        self.roll[self.ptr] = math.atan2(msg.axes[0], math.sqrt(math.pow(msg.axes[1], 2) + math.pow(msg.axes[2], 2))) / 1.57;
        self.ptr = self.ptr + 1
        if self.ptr > 9 :
            self.ptr = 0

    def onStatus(self,msg):
        """
            Callback method to translate status string into user feedback
        """
        if "under voltage" in msg.data :
            self.warning = True
        else:
            self.warning = False
                        
    def publishCmdVel(self): 
        """
            Method to average filter and publish twist from wiimote input
        """
        # Calculate average of the ten latest messages
        self.linear = (sum(self.pitch)/len(self.pitch) * self.max_linear_velocity)
        self.angular = -(sum(self.roll)/len(self.roll) * self.max_angular_velocity)
        
        # Implement deadband
        if self.angular < self.deadband and self.angular > -self.deadband :
            self.angular = 0;
        elif self.angular < 0 :
            self.angular = self.angular + self.deadband
        elif self.angular > 0 :
            self.angular = self.angular - self.deadband 
        
        # Publish twist message                   
        self.twist.header.stamp = rospy.Time.now()
        self.twist.twist.linear.x = self.linear
        self.twist.twist.angular.z = self.angular
        self.twist_pub.publish(self.twist)
        
    def publishDeadman(self):
        """
            Method to publish Bool message on deadman topic
        """
        self.deadman_pub.publish(self.deadman)
    
    def publishFeedback(self):
        """
            Method to generate and publish wiimote feedback message.
            array[0-3] are leds (set .intensity to 1 for on or 0 for off)
            array[4] is rumble (set .intensity to 1 for on or 0 for off)
        """
        # Handle periodic rumble feedback
        if self.warning or self.rumble_on :
            if rospy.Time.now() > self.next_state_change :
                if self.rumble_on :
                    self.next_state_change = rospy.Time.now() + rospy.Duration(1)
                    self.fb.array[4].intensity = 0
                    self.rumble_on = False
                else :
                    self.next_state_change = rospy.Time.now() + rospy.Duration(0.5)
                    self.fb.array[4].intensity = 1
                    self.rumble_on = True
                    
        # Publish feedback message            
        self.fb_pub.publish(self.fb)   
    
     
class remoteControlState(smach.State):
    """
        Implements remote controlled operation as a smach state
        Usage example:
            smach.StateMachine.add('REMOTE_CONTROL', remoteControlState(self.hmi), transitions={'enterAutomode':'AUTO_MODE'})
    """
    def __init__(self,hmi):
        smach.State.__init__(self, outcomes=['enterAutomode','preempted'])
        # Instantiate HMI
        self.hmi = hmi

    def execute(self, userdata):
        self.r = rospy.Rate(self.hmi.publish_frequency)
        while not self.hmi.automode and not rospy.is_shutdown():
            # Publish topics
            self.hmi.publishDeadman()
            self.hmi.publishFeedback()
            self.hmi.publishCmdVel()
            # Spin
            self.r.sleep()
        return 'enterAutomode'
        
class interfaceState(smach.State):
    """
        Implements HMI as smach state to be used in concurrence with autonomous behaviours
        Usage example:
            smach.Concurrence.add('HMI', interfaceState(self.hmi))
    """
    def __init__(self,hmi):
        smach.State.__init__(self, outcomes=['succeeded','preempted'])
        # Instantiate HMI
        self.hmi = hmi
        self.r = rospy.Rate(self.hmi.publish_frequency)
        
    def execute(self, userdata):
        while not rospy.is_shutdown():         
            if self.hmi.automode :
                # Publish topics
                self.hmi.publishDeadman()
                self.hmi.publishFeedback()
                self.r.sleep()
            else :
                break
        return 'preempted'