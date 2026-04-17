#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


def linear_interpolation(q0, q1, T, t):
    """Linear interpolation between q0 and q1
    """
    s = min(1.0, max(0.0, t / T))
    return (q1 - q0)*s + q0


class Interpolator:
    def __init__(self, 
                control_freq=50, 
                target_freq=3, 
                control_mode="impedance",
                max_target_jump=20./180.*np.pi):
        
        self.control_period = 1./control_freq
        self.target_period = 1./target_freq
        self.max_target_jump = max_target_jump
        self.max_target_delay = 2*self.target_period

        # setup the trajectory message
        trajectory_pos = JointTrajectory()
        point_pos = JointTrajectoryPoint()
        point_pos.time_from_start = rospy.Time(0)
        trajectory_pos.points.append(point_pos)
        trajectory_pos.joint_names = []

        trajectory_imp = JointTrajectory()
        point_imp = JointTrajectoryPoint()
        point_imp.time_from_start = rospy.Time(0)
        trajectory_imp.points.append(point_imp)
        trajectory_imp.joint_names = []

        self.trajectory_pos = trajectory_pos
        self.trajectory_imp = trajectory_imp

        self.msg_name = None
        self.head_names = [
            "head/joint_1","head/joint_2","head/joint_3",
        ]
        self.left_arm_names = [
            "left_arm/joint_1","left_arm/joint_2","left_arm/joint_3","left_arm/joint_4","left_arm/joint_5","left_arm/joint_6","left_arm/joint_7",
        ]
        self.right_arm_names = [
            "right_arm/joint_1","right_arm/joint_2","right_arm/joint_3","right_arm/joint_4","right_arm/joint_5","right_arm/joint_6","right_arm/joint_7",
        ]
        self.torso_names = [
            "torso/joint_1","torso/joint_2","torso/joint_3",
        ]
        
        # tsukakoshi
        self.pos_left_arm_names = ["left_arm/joint_1","left_arm/joint_2","left_arm/joint_3","left_arm/joint_4","left_arm/joint_5","left_arm/joint_6"]
        self.pos_right_arm_names = ["right_arm/joint_1","right_arm/joint_2","right_arm/joint_3","right_arm/joint_4","right_arm/joint_5","right_arm/joint_6"]
        self.imp_left_arm_names = ["left_arm/joint_7"]
        self.imp_right_arm_names = ["right_arm/joint_7"]
        self.pos_both_arms = self.pos_left_arm_names + self.pos_right_arm_names
        self.imp_both_arms = self.imp_left_arm_names + self.imp_right_arm_names
    
        self.both_arm_names   = self.left_arm_names + self.right_arm_names
        self.head_torso_names = self.head_names + self.torso_names
        self.head_arm_names   = self.head_names + self.both_arm_names
        self.arm_torso_names  = self.both_arm_names + self.torso_names
        self.all_names = self.head_arm_names + self.torso_names
        

        trajectory_pos.header.stamp = rospy.Time.now()
        trajectory_imp.header.stamp = rospy.Time.now()

        # setup states
        self.has_valid_target = False
        self.target_update_time=None
        self.cmd = None
        self.prev_target = None

        # low freq target input from network
        self.target_sub = rospy.Subscriber("/joint_predictions", JointState, self.targetCallback)

        # setup the online effort controller client
        if control_mode == "impedance":
            self.cmd_pub = rospy.Publisher("/torobo/online_joint_impedance_controller/command", JointTrajectory, queue_size=1)        
        else:
            self.cmd_pub = rospy.Publisher("/torobo/online_joint_trajectory_controller/command", JointTrajectory, queue_size=1)
            
        TOPIC_NAME_position = '/torobo/online_joint_upperarm_trajectory_controller/command'
        TOPIC_NAME_impedance = '/torobo/online_joint_lowerarm_impedance_controller/command'
        self.pub_pos = rospy.Publisher(TOPIC_NAME_position, JointTrajectory, queue_size=1)
        self.pub_imp = rospy.Publisher(TOPIC_NAME_impedance, JointTrajectory, queue_size=1)                
    
    def run(self):
        rate = rospy.Rate(1./self.control_period)

        self.target_update_time=None
        self.target=None
        self.has_valid_target = False

        rospy.logwarn("OnlineExecutor.run(): Starting execution")
        while not rospy.is_shutdown():
            
            if not self.has_valid_target:
                # just keep the current state
                rospy.logwarn_throttle(1.0, "OnlineExecutor.run(): Wait for first target")

            if self.has_valid_target:
                # interpolation between prev_target and next target
                t = (rospy.Time.now() - self.target_update_time).to_sec()
                self.cmd = linear_interpolation(self.prev_target, self.target, self.target_period, t)

                """
                # check if communicaton is broken
                if t > self.max_target_delay:
                    self.has_valid_target = False
                    rospy.logwarn("OnlineExecutor.run(): Interpolation stopped, wait for valid command")
                """
                
                # send the target to robot
                self.command(self.cmd)

            rate.sleep()
        
        rospy.logwarn("OnlineExecutor.run(): Finished execution")    


    def command(self, cmd):
        if not np.isnan(cmd).any():
            # 14個のcmdを分割
            # import ipdb;ipdb.set_trace()
            pos_cmd = np.concatenate([cmd[:6],cmd[7:13]], axis=0)  # 1-6, 1-6
            imp_cmd = [cmd[6], cmd[13]]  # 7, 7

            # ポジション用
            self.trajectory_pos.header.stamp = rospy.Time.now()
            self.trajectory_pos.points[0].positions = pos_cmd
            self.trajectory_pos.points[0].time_from_start = rospy.Duration(self.control_period)
            self.trajectory_pos.joint_names = self.pos_both_arms  # 12個

            # インピーダンス用
            self.trajectory_imp.header.stamp = rospy.Time.now()
            self.trajectory_imp.points[0].positions = imp_cmd
            self.trajectory_imp.points[0].time_from_start = rospy.Duration(self.control_period)
            self.trajectory_imp.joint_names = self.imp_both_arms  # 2個
            
            # rospy.loginfo(f"trajectory_pos.joint_names: {self.trajectory_pos.joint_names}, positions: {self.trajectory_pos.points[0].positions}")
            # rospy.loginfo(f"trajectory_imp.joint_names: {self.trajectory_imp.joint_names}, positions: {self.trajectory_imp.points[0].positions}")

            self.pub_pos.publish(self.trajectory_pos)
            self.pub_imp.publish(self.trajectory_imp)
            

    def targetCallback(self, msg):
        """
        next joint target callback from DL model
        stores next target, current time, previous target
        """

        # extract the state form message
        self.msg_name = msg.name
        target = np.array(msg.position) 

        if self.cmd is not None:
            # savety first, check last command against new target pose
            if np.max(np.abs(self.cmd - target)) > self.max_target_jump:
                self.has_valid_target = False
                idx = np.argmax(np.abs(self.cmd - target))
                rospy.logerr_throttle(1.0, "OnlineExecutor.targetCallback(): Jump in cmd[{:d}]={:f} to target[{:d}]={:f} > {:f}".format(
                    idx, self.cmd[idx], idx, target[idx], self.max_target_jump))
                return
        else:
            # initialization
            rospy.logwarn("OnlineExecutor.run(): Recieved first data")    
            self.cmd = target

        # store target and last target
        self.target = target
        self.prev_target = np.copy(self.cmd)
        self.target_update_time = rospy.Time.now()

        # target was good
        self.has_valid_target = True

def main(control_freq, target_freq, control_mode):
    executor = Interpolator(
        control_freq=control_freq,
        target_freq=target_freq, 
        control_mode=control_mode,
        max_target_jump=50./180.*np.pi)

    executor.run()

if __name__ == '__main__':
    try:
        rospy.init_node('interpolator_node', anonymous=True)
        control_freq = rospy.get_param('interpolator_node/control_freq')
        target_freq = rospy.get_param('interpolator_node/target_freq')
        control_mode = rospy.get_param('interpolator_node/control_mode')
        main(control_freq, target_freq, control_mode)
    except rospy.ROSInterruptException:
        pass
