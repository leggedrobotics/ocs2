#!/usr/bin/env python

import rospy
import tf
import time
from ocs2_msgs.msg import mpc_target_trajectories, mpc_state, mpc_input
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage

max_iteration = 10

class MobileManipulatorNode:
    def __init__(self):
        rospy.init_node('mobile_manipulator_franka_control_node', anonymous=True)
        self.publisher = rospy.Publisher('/mobile_manipulator_mpc_target', mpc_target_trajectories, queue_size=10)
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(0.5)  # 1 Hz

        # 初始化目标位置列表
        self.target_positions = [
            # [0.307, -0.261, 0.608, 0.55, 0.342, -0.402, 0.647],
            # [-0.297, 0.043, -0.033, 0.697, -0.265, -0.585, 0.316],
            # [0.247, 0.41, 0.023, 0.0, 0.263, -0.0, 0.965],
            # [0.496, -0.185, 0.127, 0.0, 0.263, -0.0, 0.965],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            [0.495, 0.61, 0.667, 0.0, 0.0, 0.0, 1.0],
            [0.675, 0.157, 0.339, 0.0, 0.0, 0.0, 1.0]

        ]
        
        self.current_target_index = 0

    def create_mpc_state(self, data):
        state = mpc_state()
        state.value = data
        return state

    def create_mpc_input(self, data):
        input = mpc_input()
        input.value = data
        return input
    
    def publish_target(self, target_position):
        # create and publish message
        input_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg = mpc_target_trajectories()
        current_time = rospy.Time.now().to_sec()
        msg.timeTrajectory = [current_time]
        msg.stateTrajectory = [self.create_mpc_state(target_position)]
        msg.inputTrajectory = [self.create_mpc_state(input_data)]
        rospy.loginfo("Publishing target trajectories: " + str(msg))
        self.publisher.publish(msg)

    def get_hand_position(self):
        try:
            # wait tf transform
            self.tf_listener.waitForTransform('/world', '/panda_leftfinger', rospy.Time(), rospy.Duration(6))
            (trans, rot) = self.tf_listener.lookupTransform('/world', '/panda_leftfinger', rospy.Time(0))
            return trans
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"TF error: {e}")
            return None

    def check_position(self, target_position, current_position, tolerance=0.05):
        # To check position if arrived within tolerance
        for t, c in zip(target_position, current_position):
            if abs(t - c) > tolerance:
                return False
        return True


    def run(self):
        iteration = 0
        start_time = time.time()
        while not rospy.is_shutdown():
            if self.current_target_index < len(self.target_positions):
                # Get current position
                target_position = self.target_positions[self.current_target_index]

                # publish target position
                self.publish_target(target_position)
                hand_position = self.get_hand_position()
                if hand_position:
                    rospy.loginfo(f"Current franka hand position: {hand_position}")

                    # check position if in tolerance
                    if self.check_position(target_position, hand_position):
                        rospy.loginfo("Position arrived within tolerance, move to next one")
                        # next position
                        self.current_target_index += 1
                        start_time = time.time()
                    else:
                        rospy.logwarn("Position does not match within tolerance.")
                        # To check if spend too much time on this movement
                        if time.time() - start_time > 600:
                            rospy.logerr("Timeout: Position check exceeded 600 seconds, please check if there is any error")
                            rospy.signal_shutdown("Timeout")

            else:
                rospy.loginfo("complete iteration " + str(iteration))
                iteration += 1
                self.current_target_index = 0
                if iteration > max_iteration:
                    rospy.signal_shutdown("Target iteration reach")

            self.rate.sleep()

if __name__ == '__main__':
    node = MobileManipulatorNode()
    node.run()

