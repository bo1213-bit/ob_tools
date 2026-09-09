#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Control the external Orbbec hardware-trigger frequency for ROS1.

This is the ROS equivalent of the C++ helper that writes
/sys/kernel/debug/gpio_trigger/framerate.
"""

from __future__ import print_function

import os
import time

import rospy


class TriggerController(object):
    def __init__(self):
        self.trigger_node = rospy.get_param(
            "~trigger_node", "/sys/kernel/debug/gpio_trigger/framerate"
        )
        self.trigger_hz = int(rospy.get_param("~trigger_hz", 30))
        self.start_delay_sec = float(rospy.get_param("~start_delay_sec", 2.0))
        self.stop_on_shutdown = bool(rospy.get_param("~stop_on_shutdown", False))

    def write_framerate(self, hz):
        try:
            with open(self.trigger_node, "w") as f:
                f.write("{}\n".format(int(hz)))
        except IOError as exc:
            rospy.logerr(
                "cannot write trigger node %s: %s. Run as root/sudo or fix permissions.",
                self.trigger_node,
                exc,
            )
            raise
        rospy.loginfo("external trigger framerate set to %d Hz", int(hz))

    def start(self):
        rospy.loginfo("disable external trigger before camera streams settle")
        self.write_framerate(0)

        if self.start_delay_sec > 0.0:
            rospy.loginfo("wait %.3f sec before enabling external trigger", self.start_delay_sec)
            deadline = time.time() + self.start_delay_sec
            while not rospy.is_shutdown() and time.time() < deadline:
                time.sleep(min(0.05, max(0.0, deadline - time.time())))

        if rospy.is_shutdown():
            return

        if self.trigger_hz > 0:
            self.write_framerate(self.trigger_hz)
        else:
            rospy.logwarn("trigger_hz <= 0, external trigger remains disabled")

    def shutdown(self):
        if not self.stop_on_shutdown:
            rospy.logwarn(
                "leaving trigger state unchanged on shutdown. "
                "This mirrors the C++ safety rule: stop camera streams before disabling trigger."
            )
            return

        try:
            self.write_framerate(0)
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logwarn("failed to disable external trigger on shutdown: %s", exc)


def main():
    rospy.init_node("ob_trigger_controller")
    controller = TriggerController()
    rospy.on_shutdown(controller.shutdown)
    controller.start()
    rospy.spin()


if __name__ == "__main__":
    main()
