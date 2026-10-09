"""Center tracking must correct inward and outward drift on the same bend."""
import math
import unittest
from robot.lane.preview import preview_steering


class Context(object):
    cfg=dict(wheelbase=.26,max_steer=.46275,sensor_timeout=.5,
             lane_curvature_preview=True,steering_command_scale_rad=.03,lookahead=.8)
    action=None
    lane_preview=None
    lane_stamp=1.


class CurveCenterFeedbackTests(unittest.TestCase):
    def command(self,offset,side=-1):
        r=.8
        points=[(r*math.sin(t),side*(r-r*math.cos(t))+offset)
                for t in (.5,.65,.8,.95,1.1)]
        return preview_steering(Context(),points,0.)

    def test_eight_centimeters_inward_must_reduce_right_angle(self):
        self.assertGreater(self.command(.08),self.command(0.)+.003)

    def test_outward_offset_must_increase_right_angle(self):
        self.assertLess(self.command(-.08),self.command(0.)-.003)

    def test_mirrored_bends_correct_equally(self):
        for offset in (-.08,0.,.08):
            self.assertAlmostEqual(self.command(offset),-self.command(-offset,1),places=8)

    def test_small_offsets_do_not_have_a_four_centimeter_dead_zone(self):
        self.assertGreater(self.command(.02),self.command(0.)+.0005)

    def test_reused_camera_frame_tracks_current_vehicle_pose(self):
        from robot.common.geometry import local
        radius=.8
        world_points=[(radius*math.sin(t),radius*math.cos(t)-radius)
                      for t in (.5,.65,.8,.95,1.1)]
        ctx=Context();ctx.pose=(0.,0.,0.)
        preview_steering(ctx,world_points,0.)
        curvature=ctx.lane_preview['curvature']
        ctx.pose=(0.,-.08,.05)
        current=[local(ctx.pose,p) for p in world_points]
        cached_command=preview_steering(ctx,current,0.)
        fresh=Context();fresh.pose=ctx.pose
        fresh_command=preview_steering(fresh,current,0.)
        self.assertAlmostEqual(cached_command,fresh_command,places=8)
        self.assertEqual(ctx.lane_preview['curvature'],curvature)

    def test_closed_loop_circle_corrects_both_offsets_without_growing_oscillation(self):
        from robot.common.geometry import local,bicycle
        for offset in (-.08,.08):
            ctx=Context();pose=(0.,offset,0.);radius=.8;errors=[]
            for i in range(301):
                angle=math.atan2(pose[1]+radius,pose[0])
                path=[local(pose,(radius*math.cos(angle-t),
                    -radius+radius*math.sin(angle-t))) for t in (.5,.65,.8,.95,1.1)]
                ctx.lane_stamp=1.+i*.05
                command=preview_steering(ctx,path,0.)
                errors.append(math.hypot(pose[0],pose[1]+radius)-radius)
                pose=bicycle(pose,.128*.05,command/.03*.46275,.26)
            self.assertLess(abs(errors[-1]),.03)
            self.assertLessEqual(max(abs(e) for e in errors),.09)

    def test_quarter_bend_to_straight_does_not_swing_far_outside(self):
        import numpy as np
        from robot.common.geometry import local,bicycle
        radius=.8;end=math.pi*radius/2
        def path(s):
            if s<end:return radius*math.sin(s/radius),-radius*(1-math.cos(s/radius))
            return radius,-radius-(s-end)
        samples=np.arange(0,4,.005)
        road=np.array([path(s) for s in samples])
        for offset in (-.08,.08):
            ctx=Context();pose=(0.,offset,0.);exit_errors=[]
            for i in range(500):
                index=np.argmin(np.sum((road-np.array(pose[:2]))**2,axis=1))
                s=samples[index]
                points=[local(pose,path(s+d)) for d in np.arange(.4,1.01,.1)]
                ctx.lane_stamp=1.+i*.05
                command=preview_steering(ctx,points,0.)
                if s>end:exit_errors.append(math.hypot(*(road[index]-np.array(pose[:2]))))
                pose=bicycle(pose,.128*.05,command/.03*.46275,.26)
            self.assertTrue(exit_errors)
            self.assertLess(max(exit_errors),.04)
            self.assertLess(exit_errors[-1],.01)


if __name__=='__main__':unittest.main()
