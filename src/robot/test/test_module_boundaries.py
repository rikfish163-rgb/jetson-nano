"""Module contracts: detached data, explicit calls, and central application."""
import copy
import unittest
from test_core import CONFIG
from robot.master.runtime import ModuleRuntime
from robot.master.runtime import ModuleContext
from robot.master.state_machine import initial_state
from robot.master.controller import Controller


class ModuleBoundariesTest(unittest.TestCase):
    def state(self):
        state = initial_state(copy.deepcopy(CONFIG))
        self.addCleanup(state['executor'].shutdown)
        return state

    def test_lane_returns_state_without_changing_caller(self):
        state = self.state()
        state.update(state='LANE', lane_stamp=1., lane_confidence=1.,
                     lane=[(.2, 0.), (.6, 0.)], pending=None)
        before = list(state['lane'])
        result = ModuleRuntime().execute('lane', 'lane_command', state, 1.)
        self.assertGreater(result.value[0], 0)
        self.assertIsNone(state['gap_origin'])
        self.assertEqual(result.updates['gap_origin'], (0., 0., 0.))
        self.assertEqual(state['lane'], before)

    def test_observation_updates_are_detached(self):
        state = self.state()
        result = ModuleRuntime().execute('camera', 'observe_lane', state,
                                         [(.3, .1), (.6, .1)], 1., 2.)
        self.assertEqual(state['lane'], [])
        self.assertEqual(len(result.updates['lane']), 2)

    def test_controller_applies_results_and_has_no_behavior_bases(self):
        c = Controller(copy.deepcopy(CONFIG))
        self.addCleanup(c.close)
        c.observe_lane([(.3, .1), (.6, .1)], 1., 2.)
        self.assertEqual(len(c.lane), 2)
        self.assertEqual(Controller.__bases__, (object,))

    def test_unknown_operation_is_rejected(self):
        with self.assertRaises(ValueError):
            ModuleRuntime().execute('lane', 'tick', self.state(), 1.)

    def test_module_has_no_controller_reference(self):
        state = self.state()
        ctx = ModuleContext(state, ('pose',), (), lambda *args: None)
        self.assertEqual(ctx.pose, (0., 0., 0.))
        with self.assertRaises(AttributeError):
            unused = ctx.pending
        with self.assertRaises(AttributeError):
            ctx.pending = 'UTURN'
        with self.assertRaises(ValueError):
            ctx.call('mission', 'tick', 1.)

    def test_input_config_changes_are_not_committed(self):
        state = self.state()
        result = ModuleRuntime().execute('motion', 'stop', state, 'fixture')
        self.assertNotIn('cfg', result.updates)
        self.assertEqual(state['reason'], 'startup')
        self.assertEqual(result.updates['reason'], 'fixture')
