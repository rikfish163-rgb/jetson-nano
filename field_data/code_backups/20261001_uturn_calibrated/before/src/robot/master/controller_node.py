#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import json
import copy
import math
import threading
import time
from collections import deque
import rospy
import rosgraph
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String,Float32,Bool
from robot.master.controller import Controller
from robot.lidar.scan import Scan
from robot.common.geometry import bicycle
from robot.common.geometry import wrap
from robot.common.geometry import local
from robot.common.geometry import world
from robot.master.telemetry import controller_status
from robot.master.telemetry import active_planned_path
from robot.master.telemetry import active_scene_task
from robot.common.contracts import decode
from robot.common.contracts import ground
from robot.common.contracts import number
from robot.common.contracts import boolean
from robot.common.contracts import validate_config
from robot.common.contracts import encode_command


def yaw(q):
    values = [number(v) for v in (q.x,q.y,q.z,q.w)]
    norm = math.sqrt(sum(v*v for v in values))
    if not 0.9 < norm < 1.1:
        raise ValueError('invalid quaternion')
    x,y,z,w = [v/norm for v in values]
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


class Node(object):
    def __init__(self):
        self.cfg = rospy.get_param('/competition/config')
        validate_config(self.cfg)
        self.core = Controller(self.cfg)
        self.lock = threading.RLock()
        self.seq = 0
        self.confidence, self.confidence_time = 0.0,-1.0
        self.applied, self.applied_at = (0,0),-1.0
        self.last_tick = rospy.Time.now().to_sec()
        self.history = deque(maxlen=100)
        self.odom_origin = None
        self.odom_frame = None
        self.status_at = -1.0
        self.last_sign_event = None
        self.last_state_event = None
        self.control_events = deque(maxlen=32)
        self.last_control_event = None
        self.live_was_enabled = False
        # No publisher on the real command topic is even registered in shadow mode.
        self.live = boolean(rospy.get_param('~live',False))
        # Subscribe once before the control timer starts. Later reads receive
        # parameter updates locally instead of making 20 XMLRPC calls/second.
        if self.live:
            boolean(rospy.get_param_cached('~enabled',False))
        self.last_command = (0,0.)
        if self.live:
            publishers,_,_ = rosgraph.Master(rospy.get_name()).getSystemState()
            if dict(publishers).get('/control/cmd'):
                raise RuntimeError('another /control/cmd publisher already exists; stop old controller first')
        self.output = rospy.Publisher('/control/cmd' if self.live else '/competition/control_preview',String,queue_size=1)
        self.status = rospy.Publisher('/competition/status',String,queue_size=1)
        self.target_debug = rospy.Publisher('/competition/lane_target',String,queue_size=50)
        self.pathpub = rospy.Publisher('/competition/trajectory',Path,queue_size=1,latch=True)
        if self.cfg.get('lane_observation_topic'):
            rospy.Subscriber(self.cfg['lane_observation_topic'],String,self.lane_observation,queue_size=1)
        else:
            rospy.Subscriber(self.cfg['lane_path_topic'],Path,self.lane,queue_size=1)
            rospy.Subscriber(self.cfg['lane_confidence_topic'],Float32,self.conf,queue_size=1)
        rospy.Subscriber('/vision/left_boundary',Path,self.left_boundary,queue_size=1)
        if self.cfg.get('lidar_enabled',True) or (self.cfg.get('parking_enabled',True) and self.cfg.get('parking_mode')!='forward_center'):
            rospy.Subscriber(self.cfg['scan_topic'],LaserScan,self.scan,queue_size=1)
        rospy.Subscriber('/competition/sign',String,self.sign,queue_size=1)
        rospy.Subscriber('/competition/uturn_scene',String,self.uturn_scene,queue_size=1)
        if self.cfg.get('parking_mode') in ('parallel_reverse','forward_plan'):
            # Ground observations build the scene in-process.  Keep the
            # existing subscriber for replay/external producers, while this
            # publisher supplies the real camera-to-scene path on the normal
            # vehicle launch.
            self.parallel_scene_pub = rospy.Publisher(
                '/competition/parallel_parking_scene',String,queue_size=1,
                latch=True)
            self.parallel_scene_published = -1.0
            rospy.Subscriber('/competition/parallel_parking_scene',String,self.parallel_scene,queue_size=1)
        rospy.Subscriber('/competition/ground',String,self.ground,queue_size=8)
        rospy.Subscriber('/competition/estop',Bool,self.estop,queue_size=1)
        rospy.Subscriber('/keyboard/control_cmd',String,self.keyboard,queue_size=1)
        rospy.Subscriber('/joystick/control_cmd',String,self.keyboard,queue_size=1)
        rospy.Subscriber('/ackermann_cmd',AckermannDriveStamped,self.applied_cb,queue_size=1)
        rospy.Subscriber(self.cfg['odom_topic'],Odometry,self.odom,queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.timer = rospy.Timer(rospy.Duration(0.05),self.tick)
        self.telemetry_timer = rospy.Timer(rospy.Duration(0.2),self.telemetry)

    def source_pose(self,stamp):
        if not self.history:
            return self.core.pose
        t,p = min(self.history,key=lambda row:abs(row[0]-stamp))
        if abs(t-stamp) > self.cfg['sensor_timeout']:
            raise ValueError('pose history cannot align source timestamp')
        return p

    def conf(self,msg):
        with self.lock:
            try:
                self.confidence = max(0,min(1,number(msg.data)))
                self.confidence_time = rospy.Time.now().to_sec()
            except ValueError:
                self.confidence = 0

    def left_boundary(self,msg):
        self.lane(msg,left_boundary=True)

    def uturn_scene(self,msg):
        with self.lock:
            try:
                now=rospy.Time.now().to_sec()
                data,stamp=decode(msg.data,now,self.cfg['sensor_timeout'])
                self.core.observe_relative_scene(data,now)
            except (ValueError,KeyError,TypeError) as exc:
                rospy.logwarn_throttle(2,'U-turn scene rejected: %s',str(exc))

    def parallel_scene(self,msg):
        if self.cfg.get('parking_mode') not in ('parallel_reverse','forward_plan'):
            return
        with self.lock:
            try:
                now=rospy.Time.now().to_sec()
                data,stamp=decode(msg.data,now,self.cfg['sensor_timeout'])
                # The in-process camera producer publishes on this same
                # latched topic for compatibility with replay consumers.
                # Do not feed our wire-format copy back into the builder: the
                # decoder intentionally drops its private confirmation state.
                if stamp <= getattr(self, 'parallel_scene_published', -1.0):
                    return
                self.core.observe_parallel_scene(data,now)
            except (ValueError,KeyError,TypeError) as exc:
                rospy.logwarn_throttle(2,'Parallel parking scene rejected: %s',str(exc))

    def lane_observation(self,msg):
        """Consume one image's path and confidence atomically, never mix topics."""
        with self.lock:
            now = rospy.Time.now().to_sec()
            try:
                data,stamp = decode(msg.data,now,self.cfg['sensor_timeout'])
                if data.get('frame') != self.cfg['lane_frame']:
                    raise ValueError('lane coordinate frame')
                confidence = number(data['confidence'])
                if not 0 <= confidence <= 1:
                    raise ValueError('lane confidence range')
                rows = data['points']
                if not isinstance(rows,list) or len(rows)>1000:
                    raise ValueError('lane points length/type')
                points = []
                for row in rows:
                    if not isinstance(row,list) or len(row)!=2:
                        raise ValueError('lane point shape')
                    x,y = number(row[0]),number(row[1])
                    if not 0 < x <= 8 or abs(y)>8:
                        raise ValueError('lane point outside local range')
                    if points and x <= points[-1][0]:
                        raise ValueError('lane points must run near to far')
                    points.append((x,y))
                min_span = self.cfg.get('lane_min_path_span',.15)
                # Direct-point lane control can use one current point; do not
                # discard the entire observation for old curve-fit requirements.
                raw_boundaries = data.get('boundaries',{})
                if not isinstance(raw_boundaries,dict) or set(raw_boundaries)-set(('LEFT','RIGHT')):
                    raise ValueError('lane boundary sides')
                boundaries = {}
                short_boundaries = []
                for side, rows in raw_boundaries.items():
                    if not isinstance(rows,list) or len(rows)>1000:
                        raise ValueError('lane boundary length/type')
                    boundary = []
                    for row in rows:
                        if not isinstance(row,list) or len(row)!=2:
                            raise ValueError('lane boundary point shape')
                        x,y = number(row[0]),number(row[1])
                        if not 0 < x <= 8 or abs(y)>8 or (boundary and x<=boundary[-1][0]):
                            raise ValueError('lane boundary geometry')
                        boundary.append((x,y))
                    if boundary and (len(boundary)<3 or boundary[-1][0]-boundary[0][0]+1e-9<min_span):
                        # Optional maneuver geometry must not prevent the center
                        # path and its source timestamp from being refreshed.
                        short_boundaries.append(side)
                        boundary = []
                    boundaries[side] = boundary
                if stamp <= self.core.lane_stamp:
                    return
                original = self.core.pose
                try:
                    self.core.pose = self.source_pose(stamp)
                    self.core.observe_lane(points,confidence,stamp,boundaries)
                finally:
                    self.core.pose = original
                reason = data.get('diagnostic',{})
                reason = reason.get('reason','unknown') if isinstance(reason,dict) else 'unknown'
                allowed = ('ok','no_trusted_centers','too_few_near_points',
                           'near_span_too_short','tracking_failed')
                self.lane_observation_status = dict(stamp=stamp,receive_age_s=now-stamp,
                    path_reason=reason if reason in allowed else 'unknown',
                    short_boundaries=short_boundaries)
            except (ValueError,KeyError,TypeError,OverflowError,RuntimeError) as exc:
                rospy.logwarn_throttle(2,'lane observation rejected: %s',str(exc))

    def lane(self,msg,left_boundary=False):
        with self.lock:
            now,stamp = rospy.Time.now().to_sec(),msg.header.stamp.to_sec()
            try:
                if msg.header.frame_id != self.cfg['lane_frame'] or not 0 <= now-stamp <= self.cfg['sensor_timeout'] or len(msg.poses)>1000:
                    raise ValueError('lane frame/stamp/length')
                points = [(number(p.pose.position.x),number(p.pose.position.y)) for p in msg.poses]
                if any(abs(x)>8 or abs(y)>8 for x,y in points):
                    raise ValueError('lane point outside local range')
                confidence = (self.confidence if not left_boundary and
                              now-self.confidence_time <= self.cfg['sensor_timeout'] else 0.0)
                original = self.core.pose
                try:
                    self.core.pose = self.source_pose(stamp)
                    if left_boundary:
                        self.core.observe_left_boundary(points,stamp)
                    else:
                        self.core.observe_lane(points,confidence,stamp)
                finally:
                    self.core.pose = original
            except (ValueError,KeyError,TypeError) as exc:
                rospy.logwarn_throttle(2,'lane rejected: %s',str(exc))

    def scan(self,msg):
        try:
            stamp = number(msg.header.stamp.to_sec())
            if not 2 <= len(msg.ranges) <= 20000:
                raise ValueError('scan size')
            duration=number(msg.time_increment)*(len(msg.ranges)-1)
            completed=stamp+duration
            timeout=self.cfg.get('lidar_timeout',self.cfg['sensor_timeout'])
            # LaserScan header is the FIRST ray. Judge delivery freshness from
            # the final ray while retaining source stamp/pose for geometry.
            if (not 0<=duration<=timeout or
                    msg.header.frame_id != self.cfg['scan_frame'] or
                    not 0<=rospy.Time.now().to_sec()-completed<=timeout):
                raise ValueError('scan frame/stamp')
            with self.lock:
                if self.core.scan is not None and stamp<=self.core.scan.stamp:return
                pose = self.source_pose(stamp)
            # Fit outside the control lock, so a scan cannot stall command ticks.
            started=time.time()
            scan = Scan(msg.ranges,number(msg.angle_min),number(msg.angle_increment),
                        number(msg.range_min),number(msg.range_max),pose,self.cfg['lidar'],stamp)
            scan.completed_stamp=completed
            scan.processing_ms=(time.time()-started)*1000.
            with self.lock:
                if self.core.scan is None or stamp > self.core.scan.stamp:
                    self.core.scan = scan
        except (ValueError,KeyError,TypeError) as exc:
            rospy.logwarn_throttle(2,'scan rejected: %s',str(exc))

    def sign(self,msg):
        with self.lock:
            try:
                now = rospy.Time.now().to_sec()
                data,stamp = decode(msg.data,now,self.cfg['sign_timeout'])
                label,confidence=data['label'],number(data['confidence'])
                startup_only=data.get('startup_only') is True
                if startup_only:
                    # Never let delayed startup detections release RED while
                    # driving, or interpret another label as a startup signal.
                    if self.core.state!='WAIT_GREEN' or label!='GREEN':
                        label,confidence='',0.0
                self.core.observe_sign(label,confidence,stamp,now)
                if (label=='PARKING' and confidence>=self.cfg['sign_confidence'] and
                        self.core.sign_info['decision']!='stale_or_duplicate' and
                        self.cfg.get('parking_sign_association',False)):
                    from robot.parking.sign_anchor import ParkingSignProjector
                    if not hasattr(self,'parking_projector'):
                        self.parking_projector=ParkingSignProjector(self.cfg)
                    point=self.parking_projector.position(data.get('sign_bounds'))
                    if point is not None:
                        point=world(self.source_pose(stamp),point)
                        self.core.parking_sign=dict(point=point,stamp=stamp)
                        entry=self.core.parking_entry
                        if entry is not None and entry.phase=='APPROACH':
                            old=entry.sign_anchor
                            if old is None or math.hypot(point[0]-old[0],point[1]-old[1])<.35:
                                entry.sign_anchor=point
                    self.core.sign_info['parking_anchor']=self.core.parking_sign
                self.core.sign_info['source']=data.get('source','classifier')
                if data.get('range_reason') in ('in_range','too_far','no_candidate',
                                                'clipped_candidate','invalid_geometry',
                                                'startup_green_unfiltered'):
                    self.core.sign_info['range_reason']=data['range_reason']
                    depth=data.get('sign_depth_m')
                    if depth is not None:
                        try:
                            depth=number(depth)
                            if depth>0:self.core.sign_info['sign_depth_m']=depth
                        except (ValueError,TypeError):
                            pass
                event = (self.core.sign_info['label'],self.core.sign_info['decision'])
                if event != self.last_sign_event:
                    self.last_sign_event = event
                    rospy.loginfo('competition_sign_decision %s',json.dumps(self.core.sign_info,allow_nan=False))
            except (ValueError,KeyError,TypeError) as exc:
                rospy.logwarn_throttle(2,'sign rejected: %s',str(exc))

    def ground(self,msg):
        with self.lock:
            try:
                data,stamp = ground(msg.data,rospy.Time.now().to_sec(),self.cfg.get('ground_timeout',1.25))
                original = self.core.pose
                try:
                    self.core.pose = self.source_pose(stamp)
                    correction = self.core.observe_ground(data,stamp)
                finally:
                    self.core.pose = original
                if correction is not None:
                    dx,dy,da = correction
                    self.core.set_pose((original[0]+dx,original[1]+dy,wrap(original[2]+da)),self.core.pose_stamp)
                    self.history = deque(((t,(p[0]+dx,p[1]+dy,wrap(p[2]+da)))
                                          for t,p in self.history),maxlen=100)
                self.publish_parallel_scene()
            except (ValueError,KeyError,TypeError) as exc:
                rospy.logwarn_throttle(2,'ground rejected: %s',str(exc))

    def publish_parallel_scene(self):
        """Publish a confirmed measured parking scene, if available."""
        publisher = getattr(self, 'parallel_scene_pub', None)
        if publisher is None:
            return
        scene = getattr(self.core, 'parallel_scene', None)
        if not isinstance(scene, dict) or scene.get('ready') is not True:
            return
        try:
            stamp = number(scene['stamp'])
            if stamp <= getattr(self, 'parallel_scene_published', -1.0):
                return
            from robot.parallel_parking.scene import public_scene
            payload = public_scene(scene)
            publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
            self.parallel_scene_published = stamp
        except (KeyError,TypeError,ValueError):
            return

    def estop(self,msg):
        with self.lock:
            if msg.data:
                self.core.estop = True
            elif rospy.get_param_cached('~enabled',False) is False:
                self.core.estop = False  # explicit reset only while automatic output is disabled

    def keyboard(self,msg):
        try:
            data = json.loads(msg.data) if len(msg.data) <= 4096 else {}
            if isinstance(data,dict) and data.get('enabled') is True:
                with self.lock:
                    self.core.estop = True  # stays latched after keyboard releases
        except ValueError:
            pass

    def applied_cb(self,msg):
        with self.lock:
            try:
                speed,steer = number(msg.drive.speed),number(msg.drive.steering_angle)
                if abs(speed) > 100 or abs(steer) > 22:
                    raise ValueError('applied raw exceeds chassis contract')
                stamp = number(msg.header.stamp.to_sec())
                if not 0 <= rospy.Time.now().to_sec()-stamp < .25 or stamp <= self.applied_at:
                    raise ValueError('applied source stamp stale or out of order')
                # Startup trim represents a trial straight-ahead neutral, not
                # intended curvature. It must not invent yaw in command odometry.
                model_raw = steer
                if self.core.state == 'STARTUP_STRAIGHT':
                    model_raw -= self.cfg.get('startup_steering_raw',0)
                self.applied = (speed/self.cfg['speed_sign'],model_raw/self.cfg['steering_sign']/self.cfg['steering_raw_limit']*self.cfg['max_steer'])
                self.applied_at = stamp
                # GAP holds command-space steering; pose estimation above
                # keeps the chassis mapping. Do not apply command gain twice.
                command_scale = self.cfg.get('steering_command_scale_rad',self.cfg['max_steer'])
                held_steer = steer/self.cfg['steering_sign']/self.cfg['steering_raw_limit']*command_scale
                self.core.observe_applied_steering(held_steer,self.applied_at)
            except ValueError:
                self.applied = (0,0)
                self.core.applied_stamp = -1.0

    def odom(self,msg):
        if self.cfg['pose_mode'] != 'odom':
            return
        with self.lock:
            try:
                from robot.common.geometry import local
                from robot.common.geometry import wrap
                p = (number(msg.pose.pose.position.x),number(msg.pose.pose.position.y),yaw(msg.pose.pose.orientation))
                if msg.child_frame_id != self.cfg['lane_frame']:
                    raise ValueError('odom child frame must be rear axle base_link')
                stamp = number(msg.header.stamp.to_sec())
                if not 0 <= rospy.Time.now().to_sec()-stamp <= self.cfg['odom_timeout']:
                    raise ValueError('odom stamp stale or future')
                if self.odom_frame is not None and msg.header.frame_id != self.odom_frame:
                    raise ValueError('odom parent frame changed')
                if self.odom_origin is None:
                    self.odom_origin = p
                    self.odom_frame = msg.header.frame_id
                xy = local(self.odom_origin,p)
                self.core.set_pose((xy[0],xy[1],wrap(p[2]-self.odom_origin[2])),stamp)
            except ValueError as exc:
                rospy.logwarn_throttle(2,'odom rejected: %s',str(exc))

    def tick(self,event):
        with self.lock:
            now = rospy.Time.now().to_sec()
            dt = max(0,min(0.10,now-self.last_tick))
            if now < self.last_tick:
                self.core.estop = True  # restart after ROS /clock rollback
            self.last_tick = now
            if self.cfg['pose_mode'] == 'command_model':
                speed,steer = self.applied if 0 <= now-self.applied_at < 0.25 else (0,0)
                gain = self.cfg['raw_to_mps']['forward' if speed >= 0 else 'reverse']
                self.core.set_pose(bicycle(self.core.pose,speed*gain*dt,steer,self.cfg['wheelbase']),now)
            self.history.append((now,self.core.pose))
            try:
                enabled = boolean(rospy.get_param_cached('~enabled',False)) if self.live else True
                self.core.lane_target = None
                command = self.core.tick(now) if enabled else (0,0.0)
                payload = encode_command(command[0],command[1],self.cfg,self.seq)
                payload['stamp'] = now
                self.output.publish(String(data=json.dumps(payload)))
                self.last_command = command
                self.queue_control_event(now,enabled,command)
                if getattr(self,'target_debug',None) is not None and self.target_debug.get_num_connections():
                    self.target_debug.publish(String(data=json.dumps(dict(
                        stamp=now, source_stamp=(self.core.lane_target['source_stamp']
                            if self.core.lane_target else self.core.lane_stamp),
                        state=self.core.state, reason=self.core.reason,
                        curve_lock=self.core.lane_curve_lock,
                        selection=self.core.lane_target, command=payload),allow_nan=False)))
                self.seq = (self.seq+1)%256
            except Exception as exc:
                self.core.estop = True
                self.last_command = (0,0.)
                self.output.publish(String(data=json.dumps(dict(encode_command(0,0,self.cfg,self.seq),stamp=now))))
                self.seq = (self.seq+1)%256
                rospy.logerr_throttle(1,'control fault, stopped: %s',str(exc))

    def queue_control_event(self,now,enabled,command):
        task=self.core.timed_bypass
        key=(enabled,self.core.state,self.core.reason,
             task['phase'] if task else None,command[0]==0)
        if key==getattr(self,'last_control_event',None):
            return
        self.last_control_event=key
        if not hasattr(self,'control_events'):
            self.control_events=deque(maxlen=32)
        # Capture at the command decision. Console transport runs in the
        # telemetry thread, even when a busy state lock prevents full status.
        self.control_events.append(copy.deepcopy(dict(stamp=now,
            state=self.core.state,reason=self.core.reason,command=command,
            timed_bypass=task,obstacle_check=self.core.obstacle_check)))

    def telemetry(self,event):
        events=getattr(self,'control_events',None)
        for unused in range(8):
            try:
                snapshot=events.popleft() if events is not None else None
            except IndexError:
                break
            if snapshot is None:
                break
            rospy.loginfo('competition_control_event %s',json.dumps(snapshot,allow_nan=False))
        # Skip a busy control cycle. Transport, JSON and console I/O must never
        # keep lane/scan callbacks or the command timer behind the state lock.
        if not self.lock.acquire(False):
            return
        try:
            now = rospy.Time.now().to_sec()
            enabled = boolean(rospy.get_param_cached('~enabled',False)) if self.live else True
            status = copy.deepcopy(controller_status(self.core,self.cfg,now,self.live,
                enabled,self.last_command,(self.seq-1)%256,
                getattr(self,'lane_observation_status',None)))
            state_event = (enabled,self.core.state,self.core.reason,self.core.action,
                self.core.pending,self.core.next_direction,
                self.core.right_lock['phase'] if self.core.right_lock else None)
            changed = state_event != self.last_state_event
            self.last_state_event = state_event
        finally:
            self.lock.release()
        try:
            self.status.publish(String(data=json.dumps(status,allow_nan=False)))
            if changed:
                console = {key:status[key] for key in ('stamp','state','reason','command',
                    'enabled','lane_confidence','source_ages','lidar_processing_ms','sign')}
                console['timed_bypass'] = status['timed_bypass']
                console['lane_curve_lock'] = status['lane_curve_lock']
                for key in ('pending','last_blue_trigger','blue_approach','straight_search',
                            'front_blue_lines_local','blue_consumed'):
                    console[key]=status[key]
                check = status['obstacle_check']
                console['obstacle_check'] = {key:check[key] for key in
                    ('kind','reason','point_local','support_points','evidence_type') if key in check}
                rospy.loginfo('competition_state %s',json.dumps(console,allow_nan=False))
            self.publish_path(now)
        except Exception as exc:
            rospy.logwarn_throttle(2,'telemetry failed: %s',str(exc))

    def publish_path(self,now):
        if not self.pathpub.get_num_connections():
            return
        msg = Path()
        msg.header.stamp, msg.header.frame_id = rospy.Time.from_sec(now),'competition_local'
        with self.lock:
            task=active_scene_task(self.core)
            if task is not None:
                msg.header.frame_id=task.frame
            # Snapshot geometry; publishing below does not hold the state lock.
            path=list(task.follower.path[task.follower.index:] if task and task.follower else
                      [] if task else active_planned_path(self.core))
        for p in path:
            item = PoseStamped(); item.header = msg.header
            item.pose.position.x,item.pose.position.y = p[:2]
            item.pose.orientation.z,item.pose.orientation.w = math.sin(p[2]/2),math.cos(p[2]/2)
            msg.poses.append(item)
        self.pathpub.publish(msg)

    def shutdown(self):
        try:
            self.output.publish(String(data=json.dumps(dict(encode_command(0,0,self.cfg,self.seq),
                stamp=rospy.Time.now().to_sec()))))
        finally:
            self.core.close()


if __name__ == '__main__':
    rospy.init_node('competition_controller')
    Node()
    rospy.spin()
