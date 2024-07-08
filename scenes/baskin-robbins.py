#!/usr/bin/env python3

import rospy
import smach
import smach_ros

from smach import StateMachine
from smach_ros import ServiceState, ActionServerWrapper, SimpleActionState, IntrospectionServer
from smach_ros.util import set_preempt_handler

from actionlib import *
from actionlib_msgs.msg import *
import random

def rand_for_baskin(cur_num):
    if cur_num == 29:
        return random.randint(1,2)
    if cur_num == 30:
        return 1
    return random.randint(1,3)

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
        						 outcomes=['keep_going', 'game_over'],
        						 input_keys=['foo_value_in'],
        						 output_keys=['foo_value_out'])

    def execute(self, userdata):
    	if userdata.foo_value_in < 31:
    		rand_num = rand_for_baskin(userdata.foo_value_in)
    		nums = ' '.join([str(userdata.foo_value_in + i) for i in range(0,rand_num)])
    		rospy.loginfo('Foo said ' + nums)
    		userdata.foo_value_out = userdata.foo_value_in + rand_num
    		return 'keep_going'
    	else:
            rospy.loginfo('Foo lost the game')
            return 'game_over'

# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
        						outcomes=['keep_going', 'game_over'],
        						input_keys=['bar_value_in'],
        						output_keys=['bar_value_out'])

    def execute(self, userdata):
        if userdata.bar_value_in < 31:
        	rand_num = rand_for_baskin(userdata.bar_value_in)
        	nums = ' '.join([str(userdata.bar_value_in + i) for i in range(0,rand_num)])
        	rospy.loginfo('Bar said ' + nums)
        	userdata.bar_value_out = userdata.bar_value_in + rand_num
        	return 'keep_going'
        else:
            rospy.loginfo('Bar lost the game')
            return 'game_over'

# define state Bas
class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['keep_going', 'game_over'],
        							input_keys=['bas_value_in'],
        							output_keys=['bas_value_out'])

    def execute(self, userdata):
        if userdata.bas_value_in < 31:
        	rand_num = rand_for_baskin(userdata.bas_value_in)
        	nums = ' '.join([str(userdata.bas_value_in + i) for i in range(0,rand_num)])
        	rospy.loginfo('Bas said ' + nums)
        	userdata.bas_value_out = userdata.bas_value_in + rand_num
        	return 'keep_going'
        else:
            rospy.loginfo('Bas lost the game')
            return 'game_over'

def main():
    rospy.init_node('state_machine_ice_cream')

    # Create the top level SMACH state machine
    sm = smach.StateMachine(outcomes=['game_over'])
    sm.userdata.value = 1

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'keep_going':'BAR', 
                                            'game_over':'game_over'},
                               remapping={'foo_value_in':'value', 
                                          'foo_value_out':'value'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'keep_going':'BAS', 
                                            'game_over':'game_over'},
                               remapping={'bar_value_in':'value', 
                                          'bar_value_out':'value'})
        smach.StateMachine.add('BAS', Bas(), 
                               transitions={'keep_going':'FOO', 
                                            'game_over':'game_over'},
                               remapping={'bas_value_in':'value', 
                                          'bas_value_out':'value'})

    sis = smach_ros.IntrospectionServer('introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
