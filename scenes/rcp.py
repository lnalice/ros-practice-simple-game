#!/usr/bin/env python3

import rospy
import smach

from smach import *
from smach_ros import *

from actionlib import *
import random

rcp_list = ['rock', 'scissors', 'paper']


def rcp_game(hand1, hand2):
    if hand1 == hand2:
        return 'same'
    if hand1 == 'rock' and hand2 == 'paper':
        return 'winner_p2'
    if hand1 == 'paper' and hand2 == 'rock':
        return 'winner_p1'
    if rcp_list.index(hand1) < rcp_list.index(hand2):
        return 'winner_p1'
    else:
        return 'winner_p2'


class Judge(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['winner_p1', 'winner_p2', 'same'],
                             input_keys=['p1', 'p2', 'round'],
                             output_keys=['p1', 'p2', 'round'])

    def execute(self, ud):
        if 'round' not in ud:
            ud.round = 0
        ud.round += 1
        rospy.loginfo("[[    ROUND %s    ]]", str(ud.round))

        while ud.p1 is None or ud.p2 is None:
            rospy.sleep(0.1)

        rospy.loginfo("[ JUDGE ] Player1 is ... %s", str(ud.p1))
        rospy.loginfo("[ JUDGE ] Player2 is ... %s", str(ud.p2))

        result = rcp_game(str(ud.p1), str(ud.p2))
        ud.p1 = None
        ud.p2 = None

        return result


class Player1(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'],
                             output_keys=['p1'])

    def execute(self, ud):
        choice = rcp_list[random.randint(0, 2)]
        ud.p1 = choice
        rospy.loginfo("[PLAYER1] %s", choice)
        return 'done'


class Player2(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['done'],
                             output_keys=['p2'])

    def execute(self, ud):
        choice = rcp_list[random.randint(0, 2)]
        ud.p2 = choice
        rospy.loginfo("[PLAYER2] %s", str(choice))
        return 'done'


def main():
    rospy.init_node('simple_rcp')
    sm = smach.StateMachine(['winner_p1', 'winner_p2'])
    sm.userdata['p1'] = None
    sm.userdata['p2'] = None
    with sm:
        game = smach.Concurrence(['same', 'winner_p1', 'winner_p2'],
                                 default_outcome='same',
                                 outcome_map={'same': {'JUDGE': 'same'},
                                              'winner_p1': {'JUDGE': 'winner_p1'},
                                              'winner_p2': {'JUDGE': 'winner_p2'}})
        with game:
            smach.Concurrence.add('PLAYER1', Player1())
            smach.Concurrence.add('PLAYER2', Player2())
            smach.Concurrence.add('JUDGE', Judge())

        smach.StateMachine.add('GAME', game,
                               transitions={'same': 'GAME',
                                            'winner_p1': 'winner_p1',
                                            'winner_p2': 'winner_p2'})

    sis = IntrospectionServer('simple_rcp', sm, '/GAME')
    sis.start()

    outcome = sm.execute()
    rospy.loginfo("Who is winner? THE PLAYER \"%s\"", outcome[-1])

    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
