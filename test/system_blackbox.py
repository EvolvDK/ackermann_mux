#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# ackermann_mux: system_blackbox.py
#
# Copyright (c) 2020 PAL Robotics S.L. All rights reserved.
# Copyright (c) 2024 Elouarn de Kerros. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# Authors:
#   * Siegfried-A. Gevatter
#   * de Kerros Elouarn

import unittest

import rospy
import time

from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

from rate_publishers import RatePublishers, TimeoutManager


def ackermann(speed=0.0, steering=0.0, acceleration=0.0, jerk=0.0):
    """Return an AckermannDriveStamped for the given speed, steering angle, acceleration and jerk."""
    a = AckermannDriveStamped()
    a.drive.speed = speed
    a.drive.steering_angle = steering
    a.drive.acceleration = acceleration
    a.drive.jerk = jerk
    return a


class TestAckermannMux(unittest.TestCase):

    # Maximum time (in seconds) that it may take for a message
    # to be received by the target node.
    MESSAGE_TIMEOUT = 0.3

    # Value (in seconds) >= the highest topic/lock timeout.
    TOPIC_TIMEOUT = 1.0

    @classmethod
    def setUpClass(cls):
        cls._publishers = RatePublishers()
        cls._vel1 = cls._publishers.add_topic('vel_1', AckermannDriveStamped)
        cls._vel2 = cls._publishers.add_topic('vel_2', AckermannDriveStamped)
        cls._vel3 = cls._publishers.add_topic('vel_3', AckermannDriveStamped)

        cls._lock1 = cls._publishers.add_topic('lock_1', Bool)
        cls._lock2 = cls._publishers.add_topic('lock_2', Bool)

        cls._timeout_manager = TimeoutManager()
        cls._timeout_manager.add(cls._publishers)
        cls._timeout_manager.spin_thread()

    def tearDown(self):
        # Reset all topics.
        ackermann_msg = ackermann(0, 0)
        unlock = Bool(False)

        self._vel1.pub(ackermann_msg)
        self._vel2.pub(ackermann_msg)
        self._vel3.pub(ackermann_msg)

        self._lock1.pub(unlock)
        self._lock2.pub(unlock)

        # Wait for previously published messages to time out,
        # since we aren't restarting ackermann_mux.
        #
        # This sleeping time must be higher than any of the
        # timeouts in system_test_config.yaml.
        #
        # TODO(artivis) use rate once available
        time.sleep(self.MESSAGE_TIMEOUT + self.TOPIC_TIMEOUT)

    @classmethod
    def _vel_cmd(cls):
        # TODO(artivis) use rate once available
        time.sleep(cls.MESSAGE_TIMEOUT)
        # TODO wait_for_msg-like functionnality not yet available
        # https://github.com/ros2/rclcpp/issues/520
        return rospy.wait_for_message('ackermann_cmd', AckermannDriveStamped,
                                      timeout=cls.MESSAGE_TIMEOUT)

    def test_empty(self):
        try:
            self._vel_cmd()
            self.fail('ackermann_mux should not be publishing without any input')
        except rospy.ROSException:
            pass

    def test_basic(self):
        a = ackermann(2.0, 0.0, 1.0, 0.5)  # speed, steering, acceleration, jerk
        self._vel1.pub(a, rate=5)
        self.assertEqual(a, self._vel_cmd())

    def test_basic_with_priorities(self):
        a1 = ackermann(2.0)
        a2 = ackermann(0, 1.0)

        # Publish ackermann from input1 @ 3Hz, it should be used.
        self._vel1.pub(a1, rate=5)
        self.assertEqual(a1, self._vel_cmd())

        # Publish ackermann from input3, it should have priority
        # over the one from input1.
        self._vel3.pub(a2, rate=10)
        self.assertEqual(a2, self._vel_cmd())

        # Stop publishing input 3 and wait for it to timeout.
        # Speed should fall back to input 1.
        self._vel3.stop()
        rospy.sleep(0.5)  # input is 0.3 in .yaml file
        self.assertEqual(a1, self._vel_cmd())

# TODO: test limits, test timeouts, etc.


if __name__ == '__main__':
    import rostest
    PKG_NAME = 'ackermann_mux'
    TEST_NAME = '%s_system_blackbox_test' % PKG_NAME
    rospy.init_node(TEST_NAME)
    rostest.rosrun(PKG_NAME, TEST_NAME, TestAckermannMux)
