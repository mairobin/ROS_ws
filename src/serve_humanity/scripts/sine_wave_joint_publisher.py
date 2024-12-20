#!/usr/bin/env python3

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def sine_wave_publisher():
    rospy.init_node('sine_wave_joint_publisher', anonymous=True)
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        msg = JointTrajectory()
        msg.joint_names = joint_names

        # Set joint positions to sine wave values
        point = JointTrajectoryPoint()
        point.positions = [math.sin(elapsed) for _ in joint_names]
        point.time_from_start = rospy.Duration(1.0)
        msg.points = [point]

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        sine_wave_publisher()
    except rospy.ROSInterruptException:
        pass
		

