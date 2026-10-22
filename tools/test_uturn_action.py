"""Offline U-turn configuration and simulated execution, no ROS or chassis."""
import copy
import os
import tempfile
import unittest
import yaml
import uturn_action_test as app
import test_uturn_four_stage as legacy

ROWS = [dict(speed=30,steering=-3,seconds=1.3),
        dict(speed=30,steering=22,seconds=2.9),
        dict(speed=-30,steering=-22,seconds=2.2),
        dict(speed=30,steering=22,seconds=3.6),
        dict(speed=-30,steering=-3,seconds=2.0)]


class ActionTests(unittest.TestCase):
    def setUp(self):
        fd,self.path=tempfile.mkstemp(suffix='.yaml');os.close(fd)
        self.addCleanup(lambda:os.unlink(self.path))
        self.write(ROWS)

    def write(self,rows,**extra):
        data=dict(countdown_s=3.,uturn_trial_pause_s=.3,uturn_trial_sequence=rows)
        data.update(extra)
        with open(self.path,'w') as stream:yaml.safe_dump(data,stream)

    def stages(self,flags=()):
        return app.make_stages(app.arguments(['--config',self.path]+list(flags)))

    def test_five_segments_and_terminal_stop(self):
        stages=self.stages()
        self.assertEqual([(s.speed,s.steering,s.seconds) for s in stages if s.speed],
                         [(r['speed'],r['steering'],r['seconds']) for r in ROWS])
        self.assertFalse(any(s.name=='ENTRY' for s in stages))
        self.assertEqual((stages[-1].name,stages[-1].speed,stages[-1].steering),('SETTLE',0,0))
        self.assertEqual([s.name for s in stages if s.name.startswith('GEAR_PAUSE')],
                         ['GEAR_PAUSE_1','GEAR_PAUSE_3','GEAR_PAUSE_4','GEAR_PAUSE_5'])

    def test_independent_yaml_wins_over_production_tuning_and_reloads(self):
        rows=copy.deepcopy(ROWS);rows[1]['seconds']=1.8;self.write(rows)
        self.assertEqual([s.seconds for s in self.stages() if s.name=='SEGMENT_2'],[1.8])
        rows[1]['seconds']=2.5;self.write(rows)
        self.assertEqual([s.seconds for s in self.stages() if s.name=='SEGMENT_2'],[2.5])

    def test_override_does_not_write_yaml(self):
        before=open(self.path).read()
        stages=self.stages(['--s5-seconds','1.4','--s1-steering','-2'])
        self.assertEqual([s.seconds for s in stages if s.name=='SEGMENT_5'],[1.4])
        self.assertEqual([s.steering for s in stages if s.name=='SEGMENT_1'],[-2])
        self.assertEqual(open(self.path).read(),before)

    def test_stop_after_keeps_zero_settle(self):
        stages=self.stages(['--stop-after','3'])
        self.assertEqual(len([s for s in stages if s.speed]),3)
        self.assertEqual((stages[-1].speed,stages[-1].steering),(0,0))

    def test_invalid_arguments_rejected_before_execute(self):
        for flags in (['--s1-speed','31'],['--s1-speed','-30'],['--s3-speed','0'],
                      ['--s2-seconds','nan'],['--s4-seconds','0'],['--s2-steering','23'],
                      ['--pause','0'],['--countdown','nan'],['--s6-speed','20'],
                      ['--stop-after','0'],['--stop-after','6']):
            with self.assertRaises(ValueError):self.stages(flags)

    def test_unknown_keys_rejected(self):
        rows=copy.deepcopy(ROWS);rows[0]['second']=1;self.write(rows)
        with self.assertRaises(ValueError):self.stages()
        self.write(ROWS,countdon_s=3)
        with self.assertRaises(ValueError):self.stages()

    def test_default_preview_never_executes(self):
        old=app.execute
        app.execute=lambda *a,**k:self.fail('preview attempted to execute')
        try:self.assertEqual(app.main(['--config',self.path]),0)
        finally:app.execute=old

    # Reuse the established runner's fake terminal, clock, ROS graph and serial status.
    graph=legacy.FourStageTests.graph.im_func
    def simulated_run(self,fault=None):
        saved=legacy.app
        legacy.app=app
        try:return legacy.FourStageTests.simulated_run.im_func(self,fault)
        finally:legacy.app=saved
    def test_complete_run(self):self.simulated_run()
    def test_operator_stop(self):self.simulated_run('operator')
    def test_status_loss(self):self.simulated_run('stale')
    def test_bridge_disconnect(self):self.simulated_run('disconnect')
    def test_competing_controller(self):self.simulated_run('competing')
    def test_estop(self):self.simulated_run('estop')
    def test_control_loop_delay(self):self.simulated_run('clock_gap')


if __name__=='__main__':unittest.main()
