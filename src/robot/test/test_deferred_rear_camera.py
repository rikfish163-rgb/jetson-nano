"""Prewarm during S turns; path-loss handoff remains the fallback trigger."""
import unittest


class DeferredRearCameraTests(unittest.TestCase):
    def camera(self):
        from robot.camera.deferred_rear_node import DeferredRearCamera
        self.starts=[]
        def start():self.starts.append(True);return None
        return DeferredRearCamera(start,10.)

    def status(self,**overrides):
        data=dict(stamp=10.2,state='PARKING',action='PARKING',enabled=True,
                  action_source='turn_path_loss',parking_entry=dict(camera_source='rear'))
        data.update(overrides);return data

    def test_no_camera_at_startup_turn_or_loss_confirmation(self):
        c=self.camera();c.tick(10.1)
        for state in ('WAIT_GREEN','LANE','MANEUVER'):
            c.observe(self.status(state=state));c.tick(10.3)
        for change in (dict(action_source='sign'),dict(enabled=False),
                       dict(parking_entry=dict(camera_source='front'))):
            c.observe(self.status(**change));c.tick(10.3)
        self.assertEqual(self.starts,[])

    def test_fresh_confirmed_loss_starts_once(self):
        c=self.camera();c.observe(self.status());c.tick(10.3);c.tick(10.4)
        self.assertEqual(len(self.starts),1)

    def test_ordinary_turn_prewarms_camera_without_waiting_for_reverse(self):
        for turn in ('LEFT','RIGHT'):
            c=self.camera()
            c.observe(self.status(state='MANEUVER',action=turn,action_source='sign',
                parking_entry=None,parking_completed=False,
                parking_mode='forward_center',parking_entry_style='S'))
            c.tick(10.3);c.tick(10.4)
            self.assertEqual(len(self.starts),1)

    def test_prewarm_ignores_other_modes_completed_bay_and_stale_status(self):
        for change in (dict(parking_mode='timed_sequence'),dict(parking_completed=True),
                       dict(parking_entry_style='T'),dict(stamp=8.),dict(enabled=False)):
            c=self.camera()
            data=self.status(state='MANEUVER',action='RIGHT',action_source='sign',
                parking_entry=None,parking_completed=False,
                parking_mode='forward_center',parking_entry_style='S')
            data.update(change);c.observe(data);c.tick(10.3)
            self.assertEqual(self.starts,[])

    def test_stale_prelaunch_and_future_status_never_start_camera(self):
        for stamp in (9.9,8.,12.):
            c=self.camera();c.observe(self.status(stamp=stamp));c.tick(10.3)
            self.assertEqual(self.starts,[])

    def test_real_controller_opens_camera_only_after_two_source_seconds(self):
        import copy
        from test_core import CONFIG
        from robot.master.controller import Controller
        from robot.master.telemetry import controller_status
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_mode='forward_center',parking_entry_style='S',wait_green=False)
        core=Controller(cfg);self.addCleanup(core.close)
        core.state='LANE';core.last_completed_action='LEFT'
        camera=self.camera()
        for stamp,points in [(10.,[(.3,0.),(.6,0.),(.9,0.)])]+[(t,[]) for t in
                (10.4,10.8,11.2,11.6,12.,12.4)]:
            core.observe_lane(points,.9 if points else 0.,stamp)
            core.execute('parking','turn_reverse_tick',stamp)
            camera.observe(controller_status(core,cfg,stamp,True,True,(0,0.),0))
            camera.tick(stamp)
            self.assertEqual(len(self.starts),1 if stamp==12.4 else 0)

    def test_real_turn_telemetry_prewarms_only_when_s_parking_enabled(self):
        import copy
        from test_core import CONFIG
        from robot.master.controller import Controller
        from robot.master.telemetry import controller_status
        for enabled in (True,False):
            for turn in ('LEFT','RIGHT'):
                cfg=copy.deepcopy(CONFIG)
                cfg.update(parking_mode='forward_center',parking_entry_style='S',
                           parking_enabled=enabled,wait_green=False)
                core=Controller(cfg);self.addCleanup(core.close)
                core.state='MANEUVER';core.action=turn
                camera=self.camera()
                camera.observe(controller_status(core,cfg,10.2,True,True,(0,0.),0))
                camera.tick(10.3)
                self.assertEqual(len(self.starts),1 if enabled else 0)


if __name__=='__main__':unittest.main()
