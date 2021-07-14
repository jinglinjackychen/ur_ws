#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from geometry_msgs.msg import Pose

# define state vx300s_move
class vx300s_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.pub = rospy.Publisher('vx300s_move', Pose, queue_size=1)
        self.vx300s_pose = Pose()
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state vx300s_move')
        if self.counter < 3:
            self.counter += 1
            self.pub.publish(self.vx300s_pose)
            time.sleep(2)
            return 'outcome1'
        else:
            return 'outcome2'

# define state ur5_move
class ur5_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.pub = rospy.Publisher('ur5_move', Pose, queue_size=1)
        self.ur5_pose = Pose()
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state ur5_move')
        if self.counter < 3:
            self.counter += 1
            self.pub.publish(self.ur5_pose)
            time.sleep(2)
            return 'outcome1'
        else:
            return 'outcome2'

# define state Init
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['init'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Init')
        time.sleep(1)
        return 'init'

def main():
    rospy.init_node('concurrence_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['End'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('Init', Init(), transitions={'init':'CON'})

        # Create the sub SMACH state machine
        sm_con = smach.Concurrence(outcomes=['continue','out'],
                                   default_outcome='continue',
                                   outcome_map={'out':
                                       { 'vx300s_move':'outcome2',
                                         'ur5_move':'outcome2'}})

        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('vx300s_move', vx300s_move())
            smach.Concurrence.add('ur5_move', ur5_move())

        smach.StateMachine.add('CON', sm_con, transitions={'continue':'CON', 'out':'End'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm_top, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()