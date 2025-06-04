#!/usr/bin/env python3
import json
import crtk
import os
import sys
import time
import copy
import signal
import numpy as np
import rospy
import rosbag
import numpy
import PyKDL
import argparse
import subprocess
import cisstRobotPython

dynamic_path = os.path.abspath(__file__+"/../../")
# dynamic_path = os.path.abspath(__file__+"/../")  # code_gc folder
# print(dynamic_path)
sys.path.append(dynamic_path)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# from code_gc import calculate_mtxG

def actuator2joint(actuator_value):
    joint_value = copy.deepcopy(actuator_value[0:5])
    joint_value.extend(actuator_value[-2] - actuator_value[-1] / 2.)
    joint_value.extend(actuator_value[-2] + actuator_value[-1] / 2.)
    return joint_value

def joint2actuator(joint_value):
    actuator_value = copy.deepcopy(joint_value[0:5])
    actuator_value.extend((joint_value[-2] + joint_value[-1]) / 2.)
    actuator_value.extend(- joint_value[-2] + joint_value[-1])
    return actuator_value

# helper function to kill the rosbag process
def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    print(ps_output)
    ps_output = ps_output.decode('utf-8')
    print(ps_output.split("\n")[:-1])
    for pid_str in ps_output.split("\n")[:-1]:
        os.kill(int(pid_str), signal.SIGINT)
    p.terminate()


# simplified arm class to replay motion, better performance than
# dvrk.psm since we're only subscribing to topics we need
class arm_custom:

    # simplified jaw class to close gripper
    class __Jaw:
        def __init__(self, ral, expected_interval, operating_state_instance):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval, operating_state_instance)
            self.__crtk_utils.add_move_jp()
            self.__crtk_utils.add_servo_jp()
            self.__crtk_utils.add_servo_jf()
            
    class __MeasuredServoCf:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_jacobian()

    def __init__(self, ral, device_namespace, expected_interval):
        # ROS initialization
        if not rospy.get_node_uri():
            # rospy.init_node('simplified_arm_class', anonymous = False, log_level = rospy.WARN)
            rospy.init_node('si_arm_class', anonymous=False, log_level=rospy.WARN)
        # populate this class with all the ROS topics we need
        self.__ral = ral.create_child(device_namespace)
        self.__crtk_utils = crtk.utils(self, self.__ral, expected_interval)
        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_servo_jp()
        self.__crtk_utils.add_servo_jf()
        self.__crtk_utils.add_move_jp()
        self.__crtk_utils.add_measured_js()
        self.__crtk_utils.add_setpoint_js()
        self.__crtk_utils.add_measured_cp()
        self.__crtk_utils.add_setpoint_cp()
        # self.__crtk_utils.add_jacobian()

        
        self.spatial = self.__MeasuredServoCf(self.__ral.create_child('spatial'), expected_interval)
        
        jaw_ral = self.ral().create_child('/jaw')
        self.jaw = self.__Jaw(jaw_ral, expected_interval,
                              operating_state_instance = self)
                              
    def ral(self):
        return self.__ral
    
    def check_connections(self, timeout = 5.0):
        self.__ral.check_connections(timeout)

class TestGC:
    def __init__(self, ral, arm):
        self.ral = ral
        self.arm = arm
        self.ff_tau_topic = '/PSM1/pid_feed_forward/servo_jf'
        self.pub_ff_tau = rospy.Publisher(self.ff_tau_topic, JointState, queue_size=1)


    def main(self, feq):
        rate = rospy.Rate(feq)

        r = cisstRobotPython.robManipulator()
        result = r.LoadRobot('~/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob')
        print("result: ", result)
        print(r.links.size())
        return 
        # print(b)
        
        # arm.servo_jp(desired_js)
        # arm.move_jp(desired_js1).wait()
        # arm.servo_jp(desired_js1)
        print('Moved to the desired position')
        while not rospy.is_shutdown():
            # ff_state = JointState()
            # ff_state.header = Header()
            # ff_state.header.stamp = rospy.Time.now()
            # ff_state.name = ['yaw', 'pitch', 'insertion', 'disc_1', 'disc_2', 'disc_3', 'disc_4']
            # arm.servo_jp(np.array([0, 0, 0.12, 0, 0, 0]))
            # arm.servo_jp(desired_js)
            # print(arm.setpoint_jp())
            # jp = self.arm.measured_jp()
            # joint_value = copy.deepcopy(jp[0:3].tolist())
            # joint_value.extend([0, 0, 0, 0])
            # cal_tau = calculate_mtxG(joint_value)
            # #### Attempt Feed Forward
            # ff_jf = copy.deepcopy(cal_tau[0:3])
            # ff_jf.extend([0.0, 0.0, 0.0, 0.0])
            # ff_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # ff_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # ff_state.effort = ff_jf
            # # print(ff_jf)
            # self.pub_ff_tau.publish(ff_state)
            rate.sleep()


if __name__ == '__main__':
    opt_traj_folder = os.path.join(dynamic_path, 'test')
    # ---------------------------------------------
    # ros setup
    # ---------------------------------------------

    # strip ros arguments
    argv = crtk.ral.parse_argv(sys.argv)

    # ---------------------------------------------
    # parse arguments
    # ---------------------------------------------
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, default='PSM1',
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help='arm name corresponding to ROS topics without namespace.')
    # parser.add_argument('-f', '--file_json', type=str, default=os.path.join(opt_traj_folder, 'psm_si_gc.json'),
    #                     help='json file containing the trajectory to replay.')
    # parser.add_argument('-c', '--config', type=str, required=False, default='1',
    #                     help='index of your current configuration, if you changed SUJ position, please use another index. '
    #                          'e.g., \"-c 1\" for configuration 1 and \"-c 2\" for for configuration 1')

    args = parser.parse_args(argv[1:])  # skip argv[0], script name
    # args = parser.parse_args()

    # ros init node
    ral = crtk.ral('dvrk_si_gc_test')

    # create arm
    arm = arm_custom(ral, device_namespace=args.arm, expected_interval=0.01)

    arm.check_connections()

    # make sure the arm is powered
    print('-- Enabling arm')
    if not arm.enable(10):
        sys.exit('-- Failed to enable within 10 seconds')

    print('-- Homing arm')
    if not arm.home(10):
        sys.exit('-- Failed to home within 10 seconds')

    # ---------------------------------------------
    # start playing trajectory and data collection
    # ---------------------------------------------
    # play trajectory
    input('---> Press \"Enter\" to start')


    # main play process
    cls_gc = TestGC(ral, arm)
    cls_gc.main(100)