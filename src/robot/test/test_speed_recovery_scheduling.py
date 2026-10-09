"""Exercise production scheduling with fake clocks; no ROS nodes or motion."""
import ast
import json
import os
import threading
import unittest
from robot.common.contracts import decode

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Box(object):
    def __init__(self, **values):
        self.__dict__.update(values)


def load_methods(path, methods, scope, functions=()):
    tree = ast.parse(open(path).read())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Node')
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    tree.body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in functions]+[cls]
    exec(compile(tree,path,'exec'),scope)
    return scope['Node'].__new__(scope['Node'])


class SpeedRecoverySchedulingTests(unittest.TestCase):
    def run_worker(self, hint, transition=False):
        clock = Box(value=10.)
        calls = []
        output = []
        scope = dict(json=json,decode=decode,String=lambda data:data,
            time=Box(time=lambda:clock.value),cv2=Box(error=RuntimeError),
            CvBridgeError=RuntimeError,
            rospy=Box(Time=Box(now=lambda:Box(to_sec=lambda:clock.value)),
                      is_shutdown=lambda:False,logwarn_throttle=lambda *args:None),
            validate_scene=lambda *args:None,both_boundary_curves=lambda *args:{})
        node = load_methods(os.path.join(ROOT,'parking/timed_vision_node.py'),
                            ('work','status'),scope,('processing_interval',))
        node.cfg = dict(ground_timeout=1.25)
        node.options = dict(curve_min_curvature=.1,curve_min_turn_deg=10.)
        node.lock = threading.Lock()
        node.mission_hint = hint
        refresh = hint is not None and hint["stamp"] == 10.
        node.bridge = Box(imgmsg_to_cv2=lambda *args:object())
        def pair(*args):
            stamp = clock.value
            return Box(header=Box(stamp=Box(to_sec=lambda:stamp))),{}
        node.buffer = Box(newest=pair)
        def observe(*args):
            calls.append(clock.value)
            return 0,None
        node.vision = Box(set_lane_observation=lambda *args:None,observe=observe)
        node.output = Box(publish=output.append)
        node.debug = Box(get_num_connections=lambda:0)
        def wait(dt):
            clock.value += dt
            if refresh:
                node.mission_hint['stamp'] = clock.value
            if transition and clock.value >= 10.12:
                node.status(Box(data=json.dumps(dict(stamp=clock.value,state='LANE',pending='PARKING'))))
        node.stop = Box(is_set=lambda:clock.value>=11.05,wait=wait)
        node.work()
        self.assertEqual(len(output),len(calls))
        return calls

    def test_idle_keeps_warm_at_low_rate(self):
        calls = self.run_worker(dict(stamp=10.,state='LANE'))
        self.assertEqual(len(calls),2)

    def test_active_parking_retains_processing_rate(self):
        self.assertGreaterEqual(len(self.run_worker(dict(stamp=10.,state='TIMED_PARKING'))),5)

    def test_pending_and_queued_parking_warm_before_dispatch(self):
        for key in ('action','pending','next_direction'):
            self.assertGreaterEqual(len(self.run_worker(dict(stamp=10.,state='LANE',**{key:'PARKING'}))),5)

    def test_missing_or_delayed_status_never_blocks_sensor(self):
        for hint in (None,dict(stamp=8.,state='LANE'),dict(stamp=12.,state='LANE')):
            self.assertGreaterEqual(len(self.run_worker(hint)),5)

    def test_parking_request_interrupts_idle_rate_promptly(self):
        calls = self.run_worker(dict(stamp=10.,state='LANE'),True)
        self.assertGreaterEqual(len(calls),5)
        self.assertLessEqual(calls[1],10.34)

    def controller(self,active):
        self.decoded = 0
        self.observed = []
        def parse(*args):
            self.decoded += 1
            return decode(*args)
        scope = dict(decode=parse,rospy=Box(Time=Box(now=lambda:Box(to_sec=lambda:10.)),
                    logwarn_throttle=lambda *args:None))
        node = load_methods(os.path.join(ROOT,'master/controller_node.py'),('timed_parking_scene',),scope)
        node.lock = threading.RLock()
        node.cfg = dict(ground_timeout=1.25)
        node.core = Box(timed_parking=object() if active else None,
                        observe_timed_parking=lambda data,now:self.observed.append((data,now)))
        return node

    def test_unused_parking_observation_skips_decode_and_state_copy(self):
        node = self.controller(False)
        node.timed_parking_scene(Box(data='invalid unused payload'))
        self.assertEqual(self.decoded,0)
        self.assertEqual(self.observed,[])

    def test_active_parking_still_receives_fresh_visual(self):
        node = self.controller(True)
        node.timed_parking_scene(Box(data=json.dumps(dict(stamp=9.9,lines=2))))
        self.assertEqual(self.decoded,1)
        self.assertEqual(len(self.observed),1)
        self.assertEqual(self.observed[0][0]['lines'],2)

    def test_active_parking_still_rejects_stale_visual(self):
        node = self.controller(True)
        node.timed_parking_scene(Box(data=json.dumps(dict(stamp=8.,lines=2))))
        self.assertEqual(self.decoded,1)
        self.assertEqual(self.observed,[])


if __name__ == '__main__':
    unittest.main()
