"""Identity, P association and image-time motion replay; never commands a car."""
import math
import unittest
import json
import threading
import numpy as np
from test_timed_parking import config,scene,raw,Follower
from robot.common.geometry import local
from robot.parking.target_line import TargetLine,FramePoses,LineCoordinates,validate_target,p_candidates
from robot.common.config import read_mapping
from robot.parking.timed_vision import both_boundary_curves
from robot.parking.timed_core import TimedParking


def line(x):
    return [[x,-.65],[x,-.3]]


class Projector(object):
    def position(self,bounds):
        return (bounds[0],-.5)


class TargetLineTests(unittest.TestCase):
    def target(self):
        target=TargetLine(config());target.reset(0.)
        data=dict(stamp=0.,route_detections=[
            dict(label='parking',confidence=.99,bounds=[1.5,0.,.2,.2]),
            dict(label='parking',confidence=.99,bounds=[2.5,0.,.2,.2]),
            dict(label='left',confidence=.99,bounds=[3.0,0.,.2,.2]),
            dict(label='parking',confidence=.2,bounds=[3.1,0.,.2,.2])])
        target.acquire_sign(data,(0.,0.,0.),(0.,0.,0.),Projector(),0.)
        self.assertEqual(target.sign_world,(2.5,-.5))
        return target

    def locked(self):
        target=self.target()
        for stamp in (.1,.2):
            data=target.observe([line(1.),line(2.),line(2.8)],stamp,(0.,0.,0.),0.)
        self.assertTrue(data['locked'])
        self.assertTrue(data['visible'])
        self.assertEqual(data['line'],line(2.))
        validate_target(data)
        return target

    def test_locks_farthest_line_before_farthest_p_and_requires_new_frames(self):
        target=self.target()
        data=target.observe([line(1.),line(2.),line(2.8)],.1,(0.,0.,0.),0.)
        self.assertFalse(data['locked'])
        target.observe([line(2.)],.1,(0.,0.,0.),0.)
        self.assertEqual(target.votes,1)
        data=target.observe([line(2.)],.2,(0.,0.,0.),0.)
        self.assertTrue(data['locked'])
        self.assertEqual(data['line'],line(2.))

    def test_motion_and_turn_preserve_identity_with_other_lines_present(self):
        target=self.locked()
        pose=(.35,.04,.12)
        rows=[[local(pose,p) for p in line(x)] for x in (1.,2.,2.8)]
        data=target.observe(rows,.3,pose,-.12)
        self.assertTrue(data['visible'])
        self.assertTrue(np.allclose(target.line_world,line(2.)))
        data=target.observe([rows[0],rows[2]],.4,pose,-.12)
        self.assertFalse(data['visible'])
        self.assertEqual(data['id'],'before_p_white')
        self.assertTrue(np.allclose(target.line_world,line(2.)))
        self.assertTrue(target.observe(rows,.5,pose,-.12)['visible'])

    def test_new_farthest_line_and_new_p_do_not_replace_locked_targets(self):
        target=self.locked()
        target.acquire_sign(dict(stamp=.3,label='PARKING',confidence=1.,
                                 sign_bounds=[3.2,0.,.2,.2]),
                            (0.,0.,0.),(0.,0.,0.),Projector(),0.)
        self.assertEqual(target.sign_world,(2.5,-.5))
        self.assertTrue(target.observe([line(2.),line(2.3)],.3,(0.,0.,0.),0.)['visible'])
        self.assertFalse(target.observe([line(2.3)],.4,(0.,0.,0.),0.)['visible'])
        self.assertEqual(target.line_world,line(2.))

    def test_short_fragment_matches_but_neighbor_and_rotated_line_do_not(self):
        target=self.locked()
        self.assertTrue(target.observe([[[2.,-.55],[2.,-.43]]],.3,(0.,0.,0.),0.)['visible'])
        self.assertTrue(target.observe([[[2.,-.43],[2.,-.3]]],.35,(0.,0.,0.),0.)['visible'])
        self.assertFalse(target.observe([line(1.65)],.4,(0.,0.,0.),0.)['visible'])
        self.assertFalse(target.observe([[[1.9,-.65],[2.1,-.3]]],.5,(0.,0.,0.),0.)['visible'])

    def test_pose_loss_is_unknown_and_no_p_cannot_lock(self):
        target=self.locked()
        data=target.observe([],.3,None,0.)
        self.assertFalse(data['valid'])
        self.assertIsNone(data['visible'])
        validate_target(data)
        target.reset(1.)
        data=target.observe([line(1.),line(2.)],1.1,(0.,0.,0.),0.)
        self.assertFalse(data['locked'])
        self.assertEqual(data['reason'],'WAIT_P_ANCHOR')
        self.assertIsNone(target.line_world)

    def test_unlocked_candidate_missing_resets_acquisition(self):
        target=self.target()
        target.observe([line(2.)],.1,(0.,0.,0.),0.)
        target.observe([],.2,(0.,0.,0.),0.)
        data=target.observe([line(2.)],.3,(0.,0.,0.),0.)
        self.assertFalse(data['locked'])
        self.assertTrue(target.observe([line(2.)],.4,(0.,0.,0.),0.)['locked'])

    def test_candidates_respect_route_region_and_classifier_fallback(self):
        data=dict(label='PARKING',confidence=.99,sign_bounds=[1.,2.,3.,4.],
                  detections=[dict(label='parking',confidence=.99,bounds=[1.,2.,3.,4.])],
                  route_detections=[])
        self.assertEqual(p_candidates(data,.8),[])
        del data['route_detections'];del data['detections']
        self.assertEqual(p_candidates(data,.8),[[1.,2.,3.,4.]])

    def test_production_yolo_park_label_can_acquire_the_p_anchor(self):
        data=dict(stamp=.1,label='PARKING',confidence=.9247,
                  route_detections=[dict(label='park',confidence=.9247,
                                        bounds=[552,101,23,45])])
        self.assertEqual(p_candidates(data,.8),[[552.,101.,23.,45.]])
        target=TargetLine(config());target.reset(0.)
        target.acquire_sign(data,(0.,0.,0.),(0.,0.,0.),Projector(),0.)
        self.assertIsNotNone(target.sign_world)

    def test_pose_history_aligns_delayed_images_and_wraps_yaw(self):
        poses=FramePoses()
        poses.update([[1.,[0.,0.,math.radians(179)]],
                      [1.2,[.2,0.,math.radians(-179)]]])
        pose=poses.at(1.1)
        self.assertAlmostEqual(pose[0],.1)
        self.assertAlmostEqual(abs(pose[2]),math.pi)
        self.assertIsNone(poses.at(.8))
        self.assertIsNone(poses.at(1.4))
        for i in range(200):poses.update([[2.+i*.05,[0.,0.,0.]]])
        self.assertEqual(len(poses.rows),100)

    def test_parking_and_p_coordinates_share_production_body_frame(self):
        import os
        cfg=config();body=cfg['front_camera']
        root=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        parking=dict(body,H=read_mapping(root+'/parallel_parking/reference_camera.yaml')['front_camera']['H'])
        coordinates=LineCoordinates(parking,body)
        points=[[.8,-.5],[1.2,-.3],[2.,-.65]]
        projected=coordinates.to_parking(points)
        self.assertTrue(np.allclose(coordinates.to_body(projected),points))
        # Check the ground image rays themselves, independent of round-trip math.
        def pixel(camera,p):
            ppm=camera['pixels_per_m']
            uv=np.array([camera['origin_u']-ppm*p[1],camera['origin_v']-ppm*p[0],1.])
            ray=np.dot(np.linalg.inv(np.asarray(camera['H']).reshape(3,3)),uv)
            return ray[:2]/ray[2]
        for a,b in zip(points,projected):
            self.assertTrue(np.allclose(pixel(body,a),pixel(parking,b)))

    def test_two_boundaries_must_curve_in_the_same_direction(self):
        xs=np.linspace(.45,1.5,12)
        left=[[float(x),.32+.4*(x-.9)**2] for x in xs]
        right=[[float(x),-.32+.4*(x-.9)**2] for x in xs]
        self.assertTrue(both_boundary_curves(dict(LEFT=left,RIGHT=right))['both_curved'])
        for bad in (dict(LEFT=left),dict(LEFT=left,RIGHT=[[float(x),-.32] for x in xs]),
                    dict(LEFT=left,RIGHT=[[float(x),-.32-.4*(x-.9)**2] for x in xs])):
            self.assertFalse(both_boundary_curves(bad)['both_curved'])

    def test_real_worker_target_metadata_drives_stopped_lidar_handoff(self):
        from robot.parking import timed_vision_node as module
        class Box(object):
            def __init__(self,**values):self.__dict__.update(values)
        clock=Box(value=1.1)
        outputs=[]
        old_ros,old_time=module.rospy,module.time
        self.addCleanup(setattr,module,'rospy',old_ros)
        self.addCleanup(setattr,module,'time',old_time)
        module.rospy=Box(Time=Box(now=lambda:Box(to_sec=lambda:clock.value)),
                         is_shutdown=lambda:False,logwarn_throttle=lambda *a:None,
                         loginfo=lambda *a:None)
        module.time=Box(time=lambda:clock.value)
        cfg=config();cfg.update(parking_mode='forward_center',parking_slot='',
                                parking_side_lidar_enabled=True)
        node=module.Node.__new__(module.Node)
        node.cfg,node.options=cfg,cfg['parking_timed']
        node.lock=threading.Lock();node.poses=FramePoses()
        node.poses.update([[1.,[0.,0.,0.]],[1.1,[0.,0.,0.]]])
        node.mission_hint=dict(stamp=1.1,state='TIMED_PARKING',
                              parking_timed=dict(select_by_lidar=True,triggered_at=1.,phase='APPROACH'))
        node.signs=[dict(stamp=1.,label='PARKING',confidence=.99,
                         route_detections=[dict(label='park',confidence=.99,bounds=[2.5,1.,.2,.2])])]
        node.target=TargetLine(cfg);node.projector=Projector();node.last_target_event=None
        node.coordinates=Box(to_body=lambda points:points)
        camera=dict(origin_u=600.,origin_v=600.,pixels_per_m=400.)
        node.vision=Box(heading=0.,camera=camera,set_lane_observation=lambda lane:None)
        def observe(frame):
            node.vision.lines=[line(1.),line(2.8)]
            if len(outputs)<2:node.vision.lines.append(line(2.))
            return len(node.vision.lines),np.zeros((600,1200,3),dtype=np.uint8)
        node.vision.observe=observe
        node.bridge=Box(imgmsg_to_cv2=lambda *args:object())
        xs=np.linspace(.45,1.5,12)
        boundaries=dict(LEFT=[[float(x),.32+.4*(x-.9)**2] for x in xs],
                        RIGHT=[[float(x),-.32+.4*(x-.9)**2] for x in xs])
        def pair(*args):
            stamp=clock.value
            lane=scene(stamp)['lane_observation'];lane['boundaries']={}
            return Box(header=Box(stamp=Box(to_sec=lambda:stamp))),lane
        node.buffer=Box(newest=pair)
        node.output=Box(publish=lambda msg:outputs.append(json.loads(msg.data)))
        node.debug=Box(get_num_connections=lambda:0)
        def wait(dt):
            clock.value+=dt
            node.mission_hint['stamp']=clock.value
            node.poses.update([[clock.value,[0.,0.,0.]]])
        node.stop=Box(is_set=lambda:len(outputs)>=3,wait=wait)
        node.work()
        self.assertFalse(outputs[0]['target_line']['locked'])
        self.assertTrue(outputs[1]['target_line']['visible'])
        self.assertFalse(outputs[2]['target_line']['visible'])
        self.assertEqual(outputs[2]['lines'],2)
        self.assertFalse(outputs[2]['both_curved'])
        self.assertEqual(outputs[2]['stop_mode'],'target_white_disappearance')
        task=TimedParking(cfg,1.,Follower(),select_by_lidar=True)
        self.addCleanup(task.close)
        for data in outputs[:2]:
            task.observe_visual(data,data['stamp'])
            self.assertEqual(raw(task,data['stamp']),(16,7))
        task.observe_visual(outputs[2],outputs[2]['stamp'])
        self.assertEqual(raw(task,outputs[2]['stamp']),(16,7))
        from test_side_parking_integration import p_frame
        stamp=outputs[2]['stamp']+.02
        task.observe_sign(p_frame(stamp,False),stamp)
        self.assertEqual(raw(task,stamp),(0,0))
        self.assertEqual(task.phase,'SELECT_SLOT')

    def test_white_processing_pauses_after_lidar_selection_starts(self):
        from robot.parking.timed_vision_node import side_processing_active
        hint=dict(stamp=2.,state='TIMED_PARKING',parking_timed=dict(phase='APPROACH'))
        self.assertTrue(side_processing_active(hint,2.1))
        for phase in ('SELECT_SLOT','REVERSE','FINISHED'):
            hint['parking_timed']['phase']=phase
            self.assertFalse(side_processing_active(hint,2.1))


if __name__=='__main__':unittest.main()
