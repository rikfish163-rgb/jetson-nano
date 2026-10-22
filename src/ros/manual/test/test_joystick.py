"""Pure input checks; importing ROS types does not start a node."""
import imp
import os
import unittest


class JoystickTests(unittest.TestCase):
    def setUp(self):
        path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'joyop.py')
        self.c = imp.load_source('joystick_under_test', path).JoystickCommand()

    def test_deadman_is_required(self):
        self.c.observe([0,1,0,-1], [0]*8, 1)
        self.assertFalse(self.c.output(1, 0)['enabled'])
        buttons = [0]*8
        buttons[4] = 1
        self.c.observe([0,1,0,-1], buttons, 2)
        self.assertEqual(self.c.output(2, 0)['speed_raw'], 12)
        self.assertEqual(self.c.output(2, 0)['steering_raw'], -22)

    def test_deadman_release_and_disconnect_hold_stop(self):
        self.c.enabled = True
        self.c.observe([0,1,0,-1], [0]*8, 2)
        self.assertTrue(self.c.output(2, 0)['enabled'])
        self.assertEqual(self.c.output(2, 0)['speed_raw'], 0)
        self.c.command = (12,22)
        self.assertEqual(self.c.output(3, 0)['speed_raw'], 0)
        self.assertTrue(self.c.output(3, 0)['enabled'])

    def test_release_button_returns_ownership_only_without_deadman(self):
        buttons = [0]*8
        buttons[7] = 1
        self.c.enabled = True
        self.c.observe([0,0,0,0], buttons, 1)
        self.assertFalse(self.c.output(1, 0)['enabled'])

    def test_bad_axes_and_missing_buttons_stop(self):
        for axes, buttons in (([], []), ([0,float('nan'),0,0],[0]*8)):
            self.c.observe(axes, buttons, 1)
            self.assertTrue(self.c.output(1, 0)['enabled'])
            self.assertEqual(self.c.output(1, 0)['speed_raw'], 0)


if __name__ == '__main__':
    unittest.main()
