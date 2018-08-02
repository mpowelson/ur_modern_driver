#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
import numpy as np
import matplotlib.pyplot as plt
import pickle

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [2.617581844329834, -1.8571932951556605, -1.0866864363299769, 2.909508228302002, -1.2964380423175257, -0.7648981253253382]

Q2 = [3.407627582550049, -1.542525593434469, -0.3759873549090784, 3.1521449089050293, -1.7933877150165003, 0.14257782697677612]   #Path 1 t=(0,2,3,4)
Q2 = [1.4444080591201782, -3.3402093092547815, 1.2740988731384277, 2.7444076538085938, -0.9104979673968714, -0.7680652777301233]       #Path2 t=(0,2,3,5)
Q3 = [2.617581844329834, -1.8571932951556605, -1.0866864363299769, 2.909508228302002, -1.2964380423175257, -0.7648981253253382]


    
client = None

def move1():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    deltas = np.empty([0,6])
    data = np.empty([0,2])
    try:
        for i in range(50):
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            t = i*1./100.
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(5.0)),
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(5. + t))
                ]
            client.send_goal(g)
            client.wait_for_result()
            time.sleep(0.5)
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            delta = (np.array(joints_pos) - np.array(Q3))*180/3.14159
            avg = np.mean(np.abs(delta))
            print(avg)
            print(delta)
            deltas = np.vstack([deltas, delta])
            data = np.vstack((data, np.hstack((t,avg))))


        fileObject = open('Path2.pkl','wb')
        pickle.dump([deltas,data],fileObject)
        fileObject.close()
        plt.plot(data[:,0],data[:,1],'b*')
        print(data[:,0])
        print(data[:,1])
        plt.axis([0, .5, 0, 1])
        plt.ylabel('mean joint error (degrees)')
        plt.xlabel('extra time (s)')
        plt.title('Path2')
        plt.show()

    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise


def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print "This program makes the robot move between the following three poses:"
        print str([Q1[i] for i in xrange(0,6)])
        print str([Q2[i] for i in xrange(0,6)])
        print str([Q3[i] for i in xrange(0,6)])
        print "Please make sure that your robot can move freely between these poses before proceeding!"
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):
            move1()
            #move_repeated()
            #move_disordered()
#            move_interrupt()
        else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
