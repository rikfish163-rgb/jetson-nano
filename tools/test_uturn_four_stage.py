"""Offline tests: ROS, terminal, clock and serial status are simulated."""
from __future__ import print_function
import copy
import json
import os
import sys
import tempfile
import types
import unittest
import yaml
import uturn_four_stage_test as app
import parallel_open_loop_test as runner
from open_loop_core import require_ownership

ROWS = [
    dict(speed=30, steering=0, seconds=1.5),
    dict(speed=30, steering=22, seconds=2.0),
    dict(speed=-30, steering=-22, seconds=1.7),
    dict(speed=30, steering=-22, seconds=1.9)]


class FourStageTests(unittest.TestCase):
    def setUp(self):
        fd, self.path = tempfile.mkstemp(suffix='.yaml')
        os.close(fd)
        self.write(ROWS)
        self.addCleanup(lambda: os.unlink(self.path))

    def write(self, rows):
        with open(self.path, 'w') as stream:
            yaml.safe_dump(dict(uturn_trial_sequence=rows, uturn_trial_entry_m=.4,
                                uturn_trial_pause_s=.7), stream)

    def stages(self, flags=()):
        return app.make_stages(app.arguments(['--config', self.path]+list(flags)))

    def test_four_commands_without_entry_or_correction(self):
        stages = self.stages()
        moving = [s for s in stages if s.speed]
        self.assertEqual([(s.speed,s.steering,s.seconds) for s in moving],
                         [(r['speed'],r['steering'],r['seconds']) for r in ROWS])
        self.assertEqual([s.name for s in stages],
                         ['COUNTDOWN','GEAR_PAUSE_1','SEGMENT_1','SEGMENT_2',
                          'GEAR_PAUSE_3','SEGMENT_3','GEAR_PAUSE_4','SEGMENT_4','SETTLE'])
        self.assertEqual((stages[-1].speed,stages[-1].steering),(0,0))

    def test_overrides_are_local_to_one_run(self):
        with open(self.path) as f: before=f.read()
        stages=self.stages(['--s2-speed','28','--s2-seconds','2.4','--s4-steering','22'])
        moving=[s for s in stages if s.speed]
        self.assertEqual((moving[1].speed,moving[1].seconds),(28,2.4))
        self.assertEqual(moving[3].steering,22)
        with open(self.path) as f: self.assertEqual(f.read(),before)

    def test_invalid_arguments_rejected_before_execute(self):
        for flags in (['--s1-speed','31'],['--s3-speed','0'],
                      ['--s2-seconds','nan'],['--s4-seconds','0'],
                      ['--s2-steering','15'],['--pause','0'],['--countdown','nan']):
            with self.assertRaises(ValueError): self.stages(flags)

    def test_exactly_four_required_and_typo_rejected(self):
        self.write(ROWS[:3])
        with self.assertRaises(ValueError): self.stages()
        rows=copy.deepcopy(ROWS);rows[0]['second']=1
        self.write(rows)
        with self.assertRaises(ValueError): self.stages()

    def test_preview_does_not_execute(self):
        old=app.execute
        app.execute=lambda *a,**k: self.fail('preview tried to execute')
        try: self.assertEqual(app.main(['--config',self.path]),0)
        finally: app.execute=old

    def graph(self):
        return ({'/ackermann_cmd':['/uturn_test_bridge'],
                 '/base_controller/status':['/base_controller']},
                {'/uturn/four_stage_cmd':['/uturn_test_bridge'],
                 '/ackermann_cmd':['/base_controller']})

    def test_dedicated_graph_rejects_other_controller(self):
        pubs,subs=self.graph()
        require_ownership(pubs,subs,'/test',command='/uturn/four_stage_cmd',bridge='/uturn_test_bridge')
        pubs['/control/cmd']=['/competition_controller']
        with self.assertRaises(ValueError):
            require_ownership(pubs,subs,'/test',command='/uturn/four_stage_cmd',bridge='/uturn_test_bridge')

    def simulated_run(self, fault=None):
        now=[0.];commands=[];callbacks={};restored=[];saved={}
        def module(name, **attrs):
            saved[name]=sys.modules.get(name)
            value=types.ModuleType(name);value.__dict__.update(attrs);sys.modules[name]=value
            return value
        class Message(object):
            def __init__(self,data=None): self.data=data
        class Publisher(object):
            def __init__(self,topic,*a,**k): self.topic=topic
            def get_num_connections(self):
                return 0 if fault=='disconnect' and now[0]>4 else 1
            def publish(self,message):
                if self.topic!='/uturn/four_stage_cmd': return
                data=json.loads(message.data)
                commands.append((now[0],data['speed_raw'],data['steering_raw']))
                if fault=='stale' and now[0]>4: return
                callback=callbacks.get('/base_controller/status')
                if callback: callback(Message(json.dumps(dict(serial_open=True,command_received=True,
                                                              timed_out=False,serial_bytes=11))))
                if fault=='estop' and now[0]>4:
                    callbacks['/competition/estop'](Message(True))
        outer=self
        class Master(object):
            def __init__(self,name): pass
            def getSystemState(self):
                pubs,subs=outer.graph()
                if fault=='competing' and now[0]>4: pubs['/control/cmd']=['/other']
                return list(pubs.items()),list(subs.items()),[]
        class Terminal(object):
            def isatty(self): return True
            def fileno(self): return 42
        def sleep(seconds):
            now[0]+=seconds
            if fault=='clock_gap' and 4<now[0]<4.3: now[0]+=.4
        old=(sys.stdin,runner.time,runner.os,runner.monotonic_clock)
        try:
            module('rospy',init_node=lambda *a,**k:None,get_name=lambda:'/uturn_test_123',
                   get_param=lambda name,default=None: '/uturn/four_stage_cmd'
                   if name=='/uturn_test_bridge/control_topic' else default,
                   Publisher=Publisher,Subscriber=lambda t,k,c,**kw:callbacks.update({t:c}),
                   is_shutdown=lambda:False,
                   Time=type('Clock',(),{'now':staticmethod(lambda:type('Stamp',(),{
                       'to_sec':lambda self:1000+now[0]})())}))
            module('rosgraph',Master=Master)
            msg=module('std_msgs.msg',String=Message,Bool=Message);module('std_msgs',msg=msg)
            module('select',select=lambda *a:([sys.stdin],[],[])
                   if fault=='operator' and now[0]>4 else ([],[],[]))
            module('termios',tcgetattr=lambda stream:'saved',TCSADRAIN=0,
                   tcsetattr=lambda *a:restored.append(True))
            module('tty',setcbreak=lambda fd:None)
            module('signal',SIGINT=2,SIGTERM=15,SIGHUP=1,signal=lambda *a:None)
            sys.stdin=Terminal()
            runner.monotonic_clock=lambda:lambda:now[0]
            runner.time=types.ModuleType('fake_time');runner.time.sleep=sleep
            runner.os=types.ModuleType('fake_os');runner.os.read=lambda *a:b'q'
            if fault:
                with self.assertRaises(ValueError):
                    runner.execute(self.stages(),auto_start=True,**app.PROFILE)
            else:
                self.assertEqual(runner.execute(self.stages(),auto_start=True,**app.PROFILE),0)
        finally:
            sys.stdin,runner.time,runner.os,runner.monotonic_clock=old
            for name,value in saved.items():
                if value is None:sys.modules.pop(name,None)
                else:sys.modules[name]=value
        self.assertTrue(restored)
        self.assertTrue(any(speed for _,speed,_ in commands))
        self.assertTrue(all(speed==steer==0 for _,speed,steer in commands[-10:]))
        return commands

    def test_execute_uses_dedicated_topic_and_stops_after_four(self):
        commands=self.simulated_run()
        moving=[]
        for _,speed,steer in commands:
            if speed and (not moving or moving[-1]!=(speed,steer)):moving.append((speed,steer))
        self.assertEqual(moving,[(30,0),(30,22),(-30,-22),(30,-22)])

    def test_operator_stop(self): self.simulated_run('operator')
    def test_status_loss(self): self.simulated_run('stale')
    def test_bridge_disconnect(self): self.simulated_run('disconnect')
    def test_competing_controller(self): self.simulated_run('competing')
    def test_estop(self): self.simulated_run('estop')
    def test_control_loop_delay(self): self.simulated_run('clock_gap')


if __name__=='__main__': unittest.main()
