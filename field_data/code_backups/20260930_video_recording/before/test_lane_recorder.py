"""Rendering rejected observations must not end the numeric recording."""
import imp
import os
import unittest
import json
import shutil
import tempfile
import Queue

recorder = imp.load_source('lane_recorder_test_module', os.path.join(
    os.path.dirname(__file__), '../lane/record_run.py'))


class LaneRecorderTests(unittest.TestCase):
    def test_worker_keeps_numeric_stream_after_rejection_and_skips_duplicates(self):
        worker = recorder.Recorder.__new__(recorder.Recorder)
        worker.path = tempfile.mkdtemp(prefix='lane-recorder-test-')
        self.addCleanup(shutil.rmtree,worker.path)
        worker.queue = Queue.Queue()
        worker.counts = {}
        worker.sources = dict(front=set(),bev=set(),observation=set())
        worker.done,worker.disabled,worker.bytes,worker.dropped = True,None,0,0
        worker.limit,worker.reserve = 1024*1024,0
        images = []
        worker.save_image = lambda name,frame: images.append(name)
        for stamp,source,selection in (
                (1.,.9,None),
                (1.1,1.,dict(valid=False,reason='lane_unreliable',target=None)),
                (1.2,1.1,dict(path=[(.5,0),(.8,-.1)],selection='test')),
                (1.3,1.1,dict(path=[(.5,0),(.8,-.1)],selection='test')),
                (1.4,1.3,dict(path=[(.5,0),(.8,-.1)],selection='test'))):
            event = dict(stamp=stamp,source_stamp=source,state='LANE',
                         command=dict(steering_raw=-22),selection=selection)
            msg = type('Message',(),dict(data=json.dumps(event)))()
            worker.queue.put(('target',msg,stamp))
        worker.worker()
        self.assertIsNone(worker.disabled)
        self.assertEqual(worker.counts['target'],5)
        self.assertEqual(len(images),2)
        with open(os.path.join(worker.path,'target.jsonl')) as stream:
            self.assertEqual(len(stream.readlines()),5)

    def test_rejected_observation_without_path_renders(self):
        event = dict(state='LANE', reason='lane_unreliable',
                     command=dict(steering_raw=0), selection=dict(
                         valid=False, reason='lane_unreliable', target=None))
        self.assertEqual(recorder.selection_image(event).shape, (600, 640, 3))

    def test_missing_selection_renders(self):
        event = dict(state='WAIT_GREEN', reason='waiting_green',
                     command=dict(steering_raw=0), selection=None)
        self.assertEqual(recorder.selection_image(event).shape, (600, 640, 3))

    def test_valid_path_renders(self):
        event = dict(state='LANE', command=dict(steering_raw=-10),
                     selection=dict(path=[(.5, .02), (.8, -.08)],
                                    selection='metric_pure_pursuit', target=(.8, -.08)))
        self.assertEqual(recorder.selection_image(event).shape, (600, 640, 3))
