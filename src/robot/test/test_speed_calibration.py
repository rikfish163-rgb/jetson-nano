"""Calibration planning, fitting and session tests; no ROS or vehicle output."""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0,os.path.join(os.path.dirname(os.path.dirname(__file__)),'parallel_parking'))
import speed_calibration as calibration


class CalibrationTests(unittest.TestCase):
    def setUp(self):self.folder=tempfile.mkdtemp()
    def tearDown(self):shutil.rmtree(self.folder)

    def test_single_duration_plan_covers_eight_direction_speed_pairs_once(self):
        plan=calibration.make_trials([15,20,25,30],[2.],['forward','reverse'],1)
        self.assertEqual(len(plan),8)
        self.assertEqual(set((r['direction'],r['speed_raw']) for r in plan),
                         set((d,s) for d in ('forward','reverse') for s in (15,20,25,30)))
        self.assertTrue(all(r['steering_raw']==0 for r in plan))

    def test_actual_command_duration_excludes_countdown_and_settling(self):
        recorder=calibration.CommandRecorder(-30)
        for speed,t in [(0,0.),(0,3.),(-30,3.05),(-30,4.),(0,5.10),(0,6.)]:
            recorder(speed,0,t)
        self.assertAlmostEqual(recorder.seconds(),2.05)

    def test_worker_runs_exactly_one_straight_segment_and_records_all_eight_profiles(self):
        original=calibration.execute
        def fake_execute(stages,auto_start=False,reference_search=None,on_command=None):
            self.assertFalse(auto_start)
            self.assertIsNone(reference_search)
            moving=[s for s in stages if s.speed]
            self.assertEqual(len(moving),1)
            self.assertEqual(moving[0].steering,0)
            self.assertEqual(moving[0].seconds,1.)
            self.assertEqual(stages[0].name,'COUNTDOWN')
            self.assertEqual(stages[-1].name,'FINAL_STOP')
            on_command(0,0,0.)
            on_command(moving[0].speed,0,3.)
            on_command(0,0,4.02)
            return 0
        try:
            calibration.execute=fake_execute
            for raw in (15,20,25,30,-15,-20,-25,-30):
                path=os.path.join(self.folder,'worker.json')
                self.assertEqual(calibration.run_child(raw,1.,path),0)
                record=json.load(open(path))
                self.assertEqual(record['status'],'complete')
                self.assertEqual(record['speed_raw'],raw)
                self.assertAlmostEqual(record['command_seconds'],1.02)
        finally:calibration.execute=original

    def test_interrupted_worker_preserves_failed_record_without_a_completed_measurement(self):
        original=calibration.execute
        def stopped(stages,on_command=None):
            on_command(-20,0,3.)
            on_command(0,0,3.2)
            raise ValueError('operator stop')
        try:
            calibration.execute=stopped
            path=os.path.join(self.folder,'failed.json')
            self.assertEqual(calibration.run_child(-20,1.,path),1)
            record=json.load(open(path))
            self.assertEqual(record['status'],'failed')
            self.assertIsNone(record['command_seconds'])
            self.assertEqual(record['error'],'operator stop')
        finally:calibration.execute=original

    def test_unexpected_steering_or_speed_is_rejected(self):
        recorder=calibration.CommandRecorder(15)
        for speed,steer in [(15,1),(-15,0),(20,0)]:
            with self.assertRaises(ValueError):recorder(speed,steer,1.)

    def test_fit_keeps_speed_and_direction_independent_with_offset(self):
        trials=calibration.make_trials([15,30],[1.,2.],['forward','reverse'],1)
        expected={('forward',15):(.12,-.02),('forward',30):(.36,-.03),
                  ('reverse',15):(.10,-.01),('reverse',30):(.31,-.04)}
        for row in trials:
            v,b=expected[(row['direction'],row['speed_raw'])]
            row.update(status='measured',command_seconds=row['requested_seconds']+.02,
                       distance_cm=100*(v*(row['requested_seconds']+.02)+b))
        models=calibration.fit_models(trials)
        for (direction,speed),(v,b) in expected.items():
            model=models[direction][str(speed)]
            self.assertEqual(model['method'],'affine')
            self.assertAlmostEqual(model['speed_mps'],v)
            self.assertAlmostEqual(model['offset_m'],b)

    def test_single_duration_is_labeled_effective_average(self):
        trials=calibration.make_trials([20],[2.],['reverse'],1)
        trials[0].update(status='measured',command_seconds=2.,distance_cm=40.)
        model=calibration.fit_models(trials)['reverse']['20']
        self.assertEqual(model['method'],'effective_average')
        self.assertAlmostEqual(model['speed_mps'],.2)

    def test_zero_motion_is_recorded_but_not_a_usable_speed_mapping(self):
        trials=calibration.make_trials([15],[1.,2.],['forward'],1)
        for row in trials:row.update(status='measured',command_seconds=row['requested_seconds'],distance_cm=0.)
        model=calibration.fit_models(trials)['forward']['15']
        self.assertFalse(model['usable'])
        self.assertEqual(model['method'],'no_motion')

    def test_invalid_measurements_and_plan_are_rejected(self):
        for value in ('-1','nan','inf','1001'):
            with self.assertRaises(ValueError):calibration.measurement(value)
        for speeds,durations in [([31],[1.]),([15],[0.]),([15],[6.]),([True],[1.])]:
            with self.assertRaises(ValueError):calibration.make_trials(speeds,durations,['forward'],1)

    def session(self):
        return calibration.new_session([15],[1.,2.],['forward','reverse'],1)

    def runner(self,row,path):
        with open(path,'w') as stream:
            json.dump(dict(status='complete',speed_raw=row['signed_speed_raw'],
                command_seconds=row['requested_seconds']+.02),stream)
        return 0

    def test_session_saves_measured_results_and_partial_progress(self):
        session=self.session();answers=iter(['10','22','q'])
        result=calibration.run_session(session,self.folder,self.runner,lambda _:next(answers))
        self.assertEqual(result,0)
        saved=json.load(open(os.path.join(self.folder,'results.json')))
        self.assertFalse(saved['complete'])
        self.assertEqual([r['status'] for r in saved['trials']],
                         ['measured','measured','awaiting_measurement','pending'])
        self.assertTrue(os.path.isfile(os.path.join(self.folder,'measurements.csv')))
        self.assertTrue(os.path.isfile(os.path.join(self.folder,'speed_table.yaml')))

    def test_resume_does_not_repeat_completed_motion_awaiting_measurement(self):
        session=self.session();answers=iter(['q'])
        calibration.run_session(session,self.folder,self.runner,lambda _:next(answers))
        saved=calibration.load_session(self.folder)
        calls=[]
        def runner(row,path):calls.append(row['index']);return self.runner(row,path)
        answers=iter(['10','22','8','19'])
        calibration.run_session(saved,self.folder,runner,lambda _:next(answers))
        self.assertEqual(calls,[2,3,4])
        self.assertTrue(json.load(open(os.path.join(self.folder,'results.json')))['complete'])

    def test_redo_requires_a_new_trial_and_does_not_keep_old_measurement(self):
        session=self.session();calls=[]
        def runner(row,path):calls.append(row['index']);return self.runner(row,path)
        answers=iter(['r','10','q'])
        calibration.run_session(session,self.folder,runner,lambda _:next(answers))
        self.assertEqual(calls,[1,1,2])
        self.assertEqual(session['trials'][0]['distance_cm'],10.)

    def test_failed_motion_never_enters_calibration_fit(self):
        session=self.session()
        self.assertEqual(calibration.run_session(session,self.folder,lambda row,path:1,
                         lambda _:self.fail('failed motion must not ask for a measurement')),1)
        self.assertEqual(calibration.fit_models(session['trials']),{})

    def test_preview_needs_no_ros_or_output_directory(self):
        path=os.path.join(self.folder,'unused')
        proc=subprocess.Popen([sys.executable,'-B',calibration.__file__,'--output',path],
                              stdout=subprocess.PIPE,stderr=subprocess.PIPE,
                              env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:1'))
        out,err=proc.communicate()
        self.assertEqual(proc.returncode,0,out+err)
        self.assertIn(b'PREVIEW ONLY',out)
        self.assertEqual(out.count(b' pending'),8)
        self.assertEqual(out.count(b'2.00s'),8)
        self.assertFalse(os.path.exists(path))


if __name__=='__main__':unittest.main()
