"""Stationary adjacent-bay regression fixtures; no ROS or vehicle access."""
from __future__ import division
import copy
import math
import unittest
import cv2
import numpy as np
from robot.parking.joint import adjacent_bays
from robot.parking.tracking import BayTracker
from robot.parking.detection import detect_bays


class AdjacentGeometryTest(unittest.TestCase):
    def scene(self, left=False, last_length=.14, width=.38, crop=False):
        ppm=400.
        mask=np.zeros((600,1200),np.uint8)
        valid=np.ones_like(mask)
        groups=[]
        mouth=480 if left else 720
        direction=-1 if left else 1
        for i,x in enumerate((.36,.36+width,.36+2*width)):
            length=last_length if i==2 else .25
            a=np.array([mouth,600-x*ppm],float)
            b=a+np.array([direction*length*ppm,0])
            cv2.line(mask,tuple(a.astype(int)),tuple(b.astype(int)),255,5)
            groups.append([a,b,(b-a)/np.linalg.norm(b-a)])
        if crop:
            valid[:,mouth-5:mouth+5]=0
        cfg=dict(slots={'P4':dict(width=.38,length=.45)},
                 white={'dimension_tolerance':.08},parking_visible_side_m=.18)
        metric=lambda u,v:((600-v)/ppm,(600-u)/ppm)
        return adjacent_bays(groups,mask,valid,metric,ppm,cfg)

    def test_three_lines_support_two_bays_without_bottom(self):
        slots,diag=self.scene()
        self.assertEqual(len(slots),2)
        self.assertEqual(slots[0]['adjacent_group'],slots[1]['adjacent_group'])
        self.assertAlmostEqual(slots[0]['x'],.55)
        self.assertAlmostEqual(slots[1]['x'],.93)
        self.assertTrue(all(s['bottom_inferred'] for s in slots))
        self.assertEqual(diag['accepted_pairs'],1)

    def test_left_side_is_mirrored(self):
        slots,unused=self.scene(left=True)
        self.assertEqual(len(slots),2)
        self.assertTrue(all(s['y']>0 and s['yaw']>0 for s in slots))

    def test_wrong_spacing_cannot_force_two_bays(self):
        self.assertEqual(self.scene(width=.60)[0],[])

    def test_cropped_mouth_cannot_force_two_bays(self):
        self.assertEqual(self.scene(crop=True)[0],[])

    def test_insufficient_third_line_is_rejected(self):
        self.assertEqual(self.scene(last_length=.07)[0],[])

    def test_full_detector_prefers_joint_pair_over_duplicate_single_bays(self):
        mask=np.zeros((600,1200),np.uint8)
        for x in (.36,.74,1.12):
            v=int(round(600-x*400))
            cv2.line(mask,(720,v),(820,v),255,5)
        cfg=dict(slots={'P4':dict(width=.38,length=.45)},
                 white={'dimension_tolerance':.08},parking_visible_side_m=.18)
        metric=lambda u,v:((600-v)/400.,(600-u)/400.)
        slots,diag=detect_bays(mask,metric,400.,cfg,np.ones_like(mask))
        self.assertEqual(len(slots),2)
        self.assertEqual(diag['accepted_joint'],2)


class TrackingTest(unittest.TestCase):
    def slots(self):
        return [dict(x=x,y=-.525,yaw=-math.pi/2,pose=[x,-.525,-math.pi/2],
                     kind='perpendicular',length=.45,width=.38,
                     adjacent_group=1,occupancy='FREE',coverage=1.)
                for x in (.55,.93)]

    def confirm_pair(self,tracker):
        for stamp in (10.,10.3,10.6):
            result=tracker.update(self.slots(),stamp,stamp+.05)
        return result

    def test_three_distinct_frames_confirm_pair(self):
        tracker=BayTracker()
        rows=self.confirm_pair(tracker)
        self.assertEqual(len(rows),2)
        self.assertTrue(all(r['confirmed'] for r in rows))
        self.assertEqual(set(r['relative_bay'] for r in rows),set(['near','far']))

    def test_repeated_source_frame_is_not_multiple_votes(self):
        tracker=BayTracker()
        for i in range(5):
            rows=tracker.update(self.slots(),10.,10.+i*.05)
        self.assertFalse(any(r['confirmed'] for r in rows))

    def test_detection_order_does_not_change_identity(self):
        tracker=BayTracker()
        before=self.confirm_pair(tracker)
        after=tracker.update(list(reversed(self.slots())),10.9,10.95)
        self.assertEqual({r['relative_bay']:r['id'] for r in before},
                         {r['relative_bay']:r['id'] for r in after})

    def test_missing_near_bay_does_not_rename_far(self):
        tracker=BayTracker()
        self.confirm_pair(tracker)
        rows=tracker.update([self.slots()[1]],10.9,10.95)
        far=next(r for r in rows if r['relative_bay']=='far')
        near=next(r for r in rows if r['relative_bay']=='near')
        self.assertTrue(far['observed'])
        self.assertFalse(near['observed'])
        self.assertEqual(near['occupancy'],'UNKNOWN')
        self.assertIsNone(near['coverage'])

    def test_missing_tracks_expire_and_are_not_permanent_two(self):
        tracker=BayTracker()
        self.confirm_pair(tracker)
        self.assertEqual(tracker.update([],12.,12.05),[])

    def test_a_lone_candidate_is_not_automatically_near(self):
        tracker=BayTracker()
        for stamp in (10.,10.3,10.6):
            rows=tracker.update([self.slots()[1]],stamp,stamp+.05)
        self.assertEqual(rows[0]['relative_bay'],'unassigned')

    def test_old_observation_never_keeps_free_status(self):
        tracker=BayTracker()
        self.confirm_pair(tracker)
        held=tracker.update(self.slots(),10.6,11.2)
        self.assertTrue(all(not t['observed'] and t['occupancy']=='UNKNOWN' for t in held))

    def test_lost_near_can_be_reconfirmed_beside_existing_far(self):
        tracker=BayTracker()
        self.confirm_pair(tracker)
        for stamp in (10.9,11.2,11.5,11.8):
            tracker.update([self.slots()[1]],stamp,stamp+.05)
        for stamp in (12.1,12.4,12.7):
            rows=tracker.update(self.slots(),stamp,stamp+.05)
        self.assertEqual(set(t['relative_bay'] for t in rows),set(['near','far']))

    def test_pose_is_smoothed_without_reusing_old_occupancy(self):
        tracker=BayTracker()
        self.confirm_pair(tracker)
        changed=copy.deepcopy(self.slots())
        changed[0]['pose'][0]+=.04
        changed[0]['occupancy']='OCCUPIED'
        rows=tracker.update(changed,10.9,10.95)
        near=next(r for r in rows if r['relative_bay']=='near')
        self.assertTrue(.55<near['pose'][0]<.59)
        self.assertEqual(near['occupancy'],'OCCUPIED')


if __name__=='__main__':
    unittest.main()
