#!/usr/bin/env python3
import subprocess
import time
import rospy
import math
from tf2_msgs.msg import TFMessage
from functools import partial


target_distance_1 = (1.0, 1.0, 0.0, 0.0)  # movement distance (x, y, z, yaw)
target_distance_2 = (-1.0, -1.0, 0.0, 0.0)  
initial_translation = (0, 0, 0)
translation = None
tolerance = 0.01 
max_iteration = 10


def set_gait():
    command = 'trot' + '\n'
    process = subprocess.Popen(
        ['rosrun', 'ocs2_legged_robot_ros', 'legged_robot_gait_command'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    time.sleep(2)

    process.stdin.write(command)
    process.stdin.flush()

    output = process.stdout.readline()
    print(output.strip())

def run_legged_robot_target(target_distance):
    try:
        process = subprocess.Popen(
            ['rosrun', 'ocs2_legged_robot_ros', 'legged_robot_target'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        # wait process to start
        time.sleep(1)

        # Input XYZ and Yaw displacements
        input_target = ' '.join(map(str, target_distance))
        # print(input_target)
        process.stdin.write(input_target + '\n')
        process.stdin.flush()

        # To wait command published
        while True:
            output = process.stdout.readline()
            if output:
                # print(output.strip() + '\n')
                if "The following command is published" in output:
                    rospy.loginfo("Detected the target command published. Input: " + input_target)
                    break

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        process.terminate()


class TFListener:
    def __init__(self, target_distance):
        self.target_distance = target_distance
        self.node_name = 'base_frame_listener'
        self.subscriber = None
        self.iteration = 1
        
    def start(self):
        rospy.init_node(self.node_name, anonymous=True)
        self.subscriber = rospy.Subscriber('/tf', TFMessage, partial(self.callback, target_distance=self.target_distance))
        rospy.spin()

    def stop(self):
        if self.subscriber:
            self.subscriber.unregister()
            self.subscriber = None
        rospy.signal_shutdown('Listener stopped')

    def calculate_distance(self, trans1, trans2):
        """Calculate two translations distance"""
        dx = trans2[0] - trans1[0]
        dy = trans2[1] - trans1[1]
        return math.sqrt(dx**2 + dy**2)
    
    def callback(self, data, target_distance):
        global translation, initial_translation
        for transform in data.transforms:
            if transform.child_frame_id == "base":
                translation = (
                    transform.transform.translation.x, 
                    transform.transform.translation.y, 
                    transform.transform.translation.z
                )
                
                # calculate two translations distance
                distance = self.calculate_distance(initial_translation, translation)
                target_distance_xy = math.sqrt(target_distance[0]**2 + target_distance[1]**2)
                
                if abs(distance - target_distance_xy) <= tolerance:
                    
                    rospy.loginfo("Target reached within tolerance. Translation: x=%f, y=%f, z=%f", 
                            translation[0], 
                            translation[1], 
                            translation[2])
                    initial_translation = translation  # update initial translation for next movement
                    rospy.loginfo('movement ' + str(self.iteration))
                    if self.iteration % 2 == 0:
                        target_distance = target_distance_1
                    else:
                        target_distance = target_distance_2
                    run_legged_robot_target(target_distance)
                    self.iteration += 1
                    if self.iteration > max_iteration:
                        rospy.signal_shutdown("Target reached")


def main():
    # set gait to trot and wait a while to take effect
    set_gait()
    time.sleep(10)
    run_legged_robot_target(target_distance_1)
    listener = TFListener(target_distance_1)
    listener.start()


if __name__ == "__main__":
    main()
    
