#!/usr/bin/env python3
"""Offline checks: never connect to a ROS master or command a robot."""
import importlib.util
from pathlib import Path
import unittest

from geometry_msgs.msg import Twist

spec = importlib.util.spec_from_file_location(
    'selector', Path(__file__).resolve().parents[1] / 'scripts' / 'command_selector.py')
selector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(selector)


class GateTests(unittest.TestCase):
    def setUp(self):
        self.gate = selector.CommandGate()
        self.command = Twist()
        self.command.linear.x = 0.2

    def test_starts_stopped_and_ignores_unselected_source(self):
        self.gate.receive('shared', self.command, 0)
        self.assertEqual(self.gate.output(0).linear.x, 0)
        self.gate.switch('maneuver', 1)
        self.gate.receive('shared', self.command, 1.3)
        self.assertEqual(self.gate.output(1.4).linear.x, 0)

    def test_handover_stops_and_discards_old_commands(self):
        self.gate.switch('shared', 0)
        self.gate.receive('shared', self.command, 0.1)
        self.assertEqual(self.gate.output(0.1).linear.x, 0)
        self.assertEqual(self.gate.output(0.3).linear.x, 0.2)
        self.gate.switch('maneuver', 0.4)
        self.gate.switch('shared', 0.5)
        self.assertEqual(self.gate.output(1).linear.x, 0)

    def test_command_timeout(self):
        self.gate.switch('shared', 0)
        self.gate.receive('shared', self.command, 0.3)
        self.assertEqual(self.gate.output(0.4).linear.x, 0.2)
        self.assertEqual(self.gate.output(2.4).linear.x, 0)

    def test_lease_expires_even_with_fresh_navigation_commands(self):
        self.gate.switch('maneuver', 0)
        self.gate.receive('maneuver', self.command, 2.1)
        self.assertEqual(self.gate.output(2.1).linear.x, 0)
        self.assertEqual(self.gate.mode, 'stopped')

    def test_heartbeat_keeps_lease_and_invalid_commands_are_rejected(self):
        self.gate.switch('maneuver', 0)
        self.gate.heartbeat_at = 2
        self.gate.receive('maneuver', self.command, 2)
        self.assertEqual(self.gate.output(2.2).linear.x, 0.2)
        self.gate.switch('maneuver', 3)
        self.command.angular.z = float('nan')
        self.gate.receive('maneuver', self.command, 3.3)
        self.assertEqual(self.gate.output(3.4).linear.x, 0)


if __name__ == '__main__':
    unittest.main()
