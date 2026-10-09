"""The supervised chassis probe must satisfy the live bridge stamp contract."""
import ast
import json
import os
import unittest
from test_keyboard_control_bridge import KeyboardControlBridgeTest, _FakePublisher


class BenchStampTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        KeyboardControlBridgeTest.setUpClass()

    @classmethod
    def tearDownClass(cls):
        KeyboardControlBridgeTest.tearDownClass()

    def test_probe_turn_and_stop_pass_live_bridge_validation(self):
        # Execute the actual send function without loading the CLI or ROS nodes.
        path = os.path.join(os.path.dirname(__file__), '../motion/bench_command.py')
        with open(path) as stream:
            source = ast.parse(stream.read())
        function = next(node for node in source.body
                        if isinstance(node, ast.FunctionDef) and node.name == 'send')
        module = KeyboardControlBridgeTest.module
        publisher = _FakePublisher()
        namespace = dict(pub=publisher, seq=[255], json=json,
                         String=module.String, rospy=module.rospy)
        exec(compile(ast.Module(body=[function]), path, 'exec'), namespace)
        bridge = module.AckermannControlBridge()
        bridge.require_stamp = True
        for speed,steer in ((0,-3),(0,-15),(0,-22),(0,22),(0,0)):
            # Each outgoing message must use the current clock, including stop.
            previous = module.rospy.Time.now().to_sec()
            module.rospy.Time.now = lambda t=previous+.05: type('Stamp', (),
                dict(to_sec=lambda self:t))()
            namespace['send'](speed,steer)
            msg = publisher.messages[-1]
            bridge.control_callback(msg)
            self.assertEqual(bridge.current_command(), (speed,steer))
            self.assertFalse(bridge.timeout_active)
        self.assertEqual(json.loads(publisher.messages[0].data)['seq'],255)
        self.assertEqual(json.loads(publisher.messages[1].data)['seq'],0)
