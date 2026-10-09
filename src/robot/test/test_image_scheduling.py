"""Exercise scheduling without starting nodes or opening cameras."""
import imp
import json
import os
import sys
import threading
import unittest

try:
    import rospy as real_ros
    from sensor_msgs.msg import Image
    import numpy as np
except ImportError:
    real_ros = None

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))


class Clock:
    value = 10.0

    @classmethod
    def now(cls):
        return real_ros.Time.from_sec(cls.value)


class Ros:
    Time = Clock

    @staticmethod
    def logwarn_throttle(*args):
        pass


class Pub:
    def __init__(self):
        self.messages = []

    def publish(self,msg):
        self.messages.append(json.loads(msg.data))

    def get_num_connections(self):
        return 0


@unittest.skipIf(real_ros is None,'requires sourced ROS Python')
class SchedulingTests(unittest.TestCase):
    def image(self,stamp):
        msg = Image()
        msg.header.stamp = real_ros.Time.from_sec(stamp)
        return msg

    def test_old_lane_and_rear_nodes_take_latest_once_and_drop_stale_future(self):
        base = os.path.join(os.path.dirname(ROOT),'ros','camera','scripts')
        lane = imp.load_source('scheduling_lane',os.path.join(base,'camera_yihan_web.py'))
        rear = imp.load_source('scheduling_rear',os.path.join(base,'rear_bev_node.py'))
        lane.rospy = rear.rospy = Ros
        lane.processing_hz = 12
        seen_lane,seen_rear = [],[]
        lane.process_image = lambda msg:seen_lane.append(msg.header.stamp.to_sec())
        node = rear.RearBevNode.__new__(rear.RearBevNode)
        node.processing_hz, node.max_image_age = 8,.5
        node._latest_image_msg = None
        node._latest_image_lock, node._processing_lock = threading.Lock(),threading.Lock()
        node._last_processed_source_stamp = None
        node._process_image = lambda msg:seen_rear.append(msg.header.stamp.to_sec())
        for callback,tick,seen in ((lane.image_callback,lane.process_latest_image,seen_lane),
                                   (node._image_callback,node._process_latest_image,seen_rear)):
            callback(self.image(9.6))
            callback(self.image(9.9))
            tick(None)
            tick(None)
            self.assertEqual(seen,[9.9])
            callback(self.image(8.0))
            tick(None)
            callback(self.image(11.0))
            tick(None)
            self.assertEqual(seen,[9.9])

    def ground(self):
        mod = imp.load_source('scheduling_ground',os.path.join(ROOT,'camera', 'perception_node.py'))
        mod.rospy = Ros
        n = mod.Node.__new__(mod.Node)
        n.cfg = dict(ground_timeout=1.25,parking_enabled=True,slot_hz=2,
                     white=dict(v_min=200,s_max=60),
                     front_camera=dict(bev_width=2,bev_height=2))
        n.lock = threading.Lock()
        n.latest = dict(front=None,rear=None)
        n.last = dict(front=-1,rear=-1)
        n.slot_last = dict(front=-1,rear=-1)
        n.front_cache = None
        n.slot_at = n.diag_at = -1
        n.next_slot_source = 'front'
        n.parking_requested = False
        n.parking_selecting = False
        n.uturn_requested = False
        n.uturn_vision = None
        n.stats = dict(front_frames=0,slot_frames=0,stale_front=0,stale_rear=0)
        n.pub,n.diag = Pub(),Pub()
        n.debug = dict((k,Pub()) for k in ('front_bev','blue','white','rear_white','rear_blue',
                                          'parking_bev','parking_white'))
        class Bridge:
            def imgmsg_to_cv2(self,msg,encoding):
                return np.zeros((2,2,3),dtype=np.uint8)
        class Detector:
            calls = 0
            def bev(self,frame,parking=False):
                return frame
            def detect(self,bev,rear=False,include_slots=True):
                if include_slots:
                    raise AssertionError('blue path must not run slots')
                return dict(markers=[],slots=[]),bev,bev
            def detect_slots(self,white,rear=False,u_offset=0):
                assert any(m['part'] == 'markers' for m in n.pub.messages)
                self.calls += 1
                return []
            def parking_lines(self,white,u_offset=0):
                return []
        n.bridge,n.detector = Bridge(),Detector()
        return n

    def test_forward_parking_publishes_blue_then_lines_then_slots(self):
        import yaml
        from robot.camera.vision import GroundDetector
        n = self.ground()
        with open(os.path.join(ROOT,'config','competition.yaml')) as stream:
            n.cfg = yaml.safe_load(stream)
        n.cfg['parking_mode'] = 'forward_white'
        n.detector = GroundDetector(n.cfg)
        n.bridge.imgmsg_to_cv2 = lambda msg,encoding:np.zeros((360,640,3),np.uint8)
        n.debug.update(parking_bev=Pub(),parking_white=Pub())
        n.parking_requested = True
        n.front(self.image(9.9))
        n.tick(None)
        n.tick(None)
        self.assertEqual([m['part'] for m in n.pub.messages],['markers','parking_lines','slots'])
        self.assertEqual(n.front_cache[1].shape,(600,1200))
        # Tracking continues on a new image even when bay enumeration is throttled.
        n.front(self.image(9.95))
        n.tick(None)
        self.assertEqual([m['part'] for m in n.pub.messages][-2:],['markers','parking_lines'])

    def test_forward_center_publishes_lines_without_slot_planning(self):
        import yaml
        from robot.camera.vision import GroundDetector
        n = self.ground()
        with open(os.path.join(ROOT,'config','competition.yaml')) as stream:
            n.cfg = yaml.safe_load(stream)
        n.cfg['parking_mode'] = 'forward_center'
        class UturnVision:
            blue_enabled = False
            def update(self,*args):
                raise AssertionError('parking must not wait for UTURN visual motion')
        n.uturn_vision = UturnVision()
        n.detector = GroundDetector(n.cfg)
        n.bridge.imgmsg_to_cv2 = lambda msg,encoding:np.zeros((360,640,3),np.uint8)
        n.debug.update(parking_bev=Pub(),parking_white=Pub())
        n.parking_requested = True
        n.front(self.image(9.9));n.tick(None)
        self.assertEqual([m['part'] for m in n.pub.messages],['markers','parking_lines'])

    def test_queued_s_parking_prepares_lines_during_fixed_straight(self):
        from std_msgs.msg import String
        n = self.ground()
        n.cfg.update(parking_mode='forward_center', parking_entry_style='S')
        n.status(String(data=json.dumps(dict(state='MANEUVER', action='STRAIGHT',
                     pending=None, next_direction='PARKING'))))
        n.front(self.image(9.9))
        n.tick(None)
        self.assertTrue(n.parking_requested)
        self.assertEqual([m['part'] for m in n.pub.messages],
                         ['markers', 'parking_lines'])
        n.status(String(data=json.dumps(dict(state='LANE', pending=None,
                                             next_direction=None))))
        self.assertFalse(n.parking_requested)

    def test_queued_t_parking_does_not_request_white_geometry(self):
        from std_msgs.msg import String
        n = self.ground()
        n.cfg.update(parking_mode='forward_center', parking_entry_style='T')
        n.status(String(data=json.dumps(dict(state='MANEUVER', action='STRAIGHT',
                     pending=None, next_direction='PARKING'))))
        n.front(self.image(9.9))
        n.tick(None)
        self.assertFalse(n.parking_requested)
        self.assertEqual([m['part'] for m in n.pub.messages], ['markers'])

    def test_blue_latest_is_published_without_any_parking_work(self):
        n = self.ground()
        n.front(self.image(9.1))
        n.front(self.image(9.9))
        n.tick(None)
        n.tick(None)
        self.assertEqual(n.detector.calls,0)
        self.assertEqual(len(n.pub.messages),1)
        self.assertEqual(n.pub.messages[0]['stamp'],9.9)

    def test_uturn_rear_markers_publish_without_parking_and_only_once(self):
        n=self.ground()
        n.cfg['parking_enabled']=False
        n.uturn_requested=True
        n.front(self.image(9.9))
        n.rear(self.image(9.9))
        n.tick(None)
        n.tick(None)
        self.assertEqual([(m['source'],m['part']) for m in n.pub.messages],
                         [('front','markers'),('rear','markers')])
        self.assertEqual(n.detector.calls,0)

    def test_blue_publishes_before_slot_search_and_same_image_not_researched(self):
        n = self.ground()
        n.parking_requested = True
        n.front(self.image(9.9))
        n.tick(None)
        n.tick(None)
        self.assertEqual([m['part'] for m in n.pub.messages],['markers','slots'])
        self.assertEqual(n.detector.calls,1)

    def test_auto_selection_prioritizes_front_then_restores_camera_alternation(self):
        n = self.ground()
        n.parking_requested = True
        n.parking_selecting = True
        n.next_slot_source = 'rear'
        n.front(self.image(9.9))
        n.rear(self.image(9.9))
        n.tick(None)
        self.assertEqual(n.pub.messages[-1]['source'],'front')
        n.parking_selecting = False
        n.slot_at = -1
        n.tick(None)
        self.assertEqual(n.pub.messages[-1]['source'],'rear')

    def test_parallel_tracking_alternates_front_and_rear_at_four_hz_each(self):
        n = self.ground()
        n.cfg.update(parking_mode='parallel_reverse',ground_hz=8.)
        n.parking_requested = True
        n.next_slot_source = 'front'
        self.addCleanup(lambda:setattr(Clock,'value',10.))
        for now in (10.,10.125,10.25):
            Clock.value = now
            n.front(self.image(now-.02))
            n.rear(self.image(now-.02))
            n.tick(None)
            n.tick(None)
        observations = [m for m in n.pub.messages if m['part']=='slots']
        self.assertEqual(len(observations),3)
        self.assertEqual([m['source'] for m in observations],
                         ['front','rear','front'])
        self.assertEqual(n.detector.calls,3)

    def test_parallel_status_state_keeps_slot_search_after_pending_consumed(self):
        n = self.ground()
        n.cfg.update(parking_mode='parallel_reverse',ground_hz=8.,
                     parallel_parking_confirm_frames=3)
        n.status(type('Status',(object,),{'data':json.dumps(
            dict(state='PARALLEL_PARKING',pending=None))})())
        self.addCleanup(lambda:setattr(Clock,'value',10.))
        n.front(self.image(9.98))
        n.tick(None)
        observations = [m for m in n.pub.messages if m['part']=='slots']
        self.assertEqual([m['source'] for m in observations], ['front'])

    def test_parallel_missing_rear_continues_front_measurements(self):
        n = self.ground()
        n.cfg.update(parking_mode='parallel_reverse',ground_hz=8.)
        n.parking_requested = True
        self.addCleanup(lambda:setattr(Clock,'value',10.))
        for now in (10.,10.125):
            Clock.value = now
            n.front(self.image(now-.02))
            n.tick(None)
            n.tick(None)
        observations = [m for m in n.pub.messages if m['part']=='slots']
        self.assertEqual([m['source'] for m in observations], ['front','front'])

    def test_parallel_slots_use_wide_view_without_forward_entry_lines(self):
        import yaml
        from robot.camera.vision import GroundDetector
        n = self.ground()
        with open(os.path.join(ROOT,'config','competition.yaml')) as stream:
            n.cfg = yaml.safe_load(stream)
        n.cfg['parking_mode'] = 'parallel_reverse'
        n.detector = GroundDetector(n.cfg)
        n.bridge.imgmsg_to_cv2 = lambda msg,encoding:np.zeros((360,640,3),np.uint8)
        n.parking_requested = True
        n.front(self.image(9.9))
        n.tick(None)
        self.assertEqual([m['part'] for m in n.pub.messages],['markers','slots'])
        self.assertEqual(n.front_cache[1].shape,(600,1200))

    def test_forward_plan_uses_front_slots_at_ground_rate_only(self):
        n = self.ground()
        n.cfg.update(parking_mode='forward_plan',ground_hz=8.)
        n.parking_requested = True
        n.detector.parking_lines = lambda white,u_offset=0: [
            [[.45, -.63], [1.15, -.63]],
            [[.45, -.27], [1.15, -.27]],
            [[1.15, -.63], [1.15, -.27]]]
        self.addCleanup(lambda:setattr(Clock,'value',10.))
        for now in (10.,10.125):
            Clock.value = now
            n.front(self.image(now-.02))
            n.rear(self.image(now-.02))
            n.tick(None)
            n.tick(None)
        observations = [m for m in n.pub.messages if m['part']=='slots']
        self.assertEqual([m['source'] for m in observations], ['front','front'])
        self.assertEqual(observations[0]['lines'], observations[1]['lines'])
        self.assertEqual(observations[0]['stamp'], 9.98)
        self.assertEqual(n.detector.calls,2)


if __name__ == '__main__':
    unittest.main()
