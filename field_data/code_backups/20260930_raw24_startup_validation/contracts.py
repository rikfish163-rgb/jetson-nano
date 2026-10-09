"""Small ROS input boundary; rejects non-finite, unbounded and stale JSON."""
from __future__ import division
import json
import math


def number(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)):
        raise ValueError('expected numeric value')
    value = float(value)
    if math.isnan(value) or math.isinf(value):
        raise ValueError('nonfinite number')
    return value


def boolean(value):
    if type(value) is not bool:
        raise ValueError('expected true/false boolean, not string or integer')
    return value


def model_to_command_steering(angle, cfg):
    fraction = max(-1.0,min(1.0,angle/cfg['max_steer']))
    return fraction*cfg.get('steering_command_scale_rad',cfg['max_steer'])


def command_to_model_steering(command, cfg):
    raw = encode_command(0,command,cfg,0)['steering_raw']
    return raw/cfg['steering_sign']/cfg['steering_raw_limit']*cfg['max_steer']


def validate_config(cfg):
    vision=boolean(cfg.get('uturn_vision_enabled',False))
    blue_landmarks=boolean(cfg.get('uturn_blue_landmarks_enabled',False))
    if blue_landmarks:
        if not vision: raise ValueError('blue landmarks require visual U-turn mode')
        if not .8<=number(cfg.get('uturn_blue_line_length_m',1.))<=1.2:
            raise ValueError('invalid blue landmark length')
        if not .02<=number(cfg.get('uturn_blue_end_gap_m',.1))<=.25:
            raise ValueError('invalid blue landmark gap')
    if vision and not boolean(cfg.get('uturn_relative_enabled',False)):
        raise ValueError('uturn vision requires relative U-turn mode')
    trial=boolean(cfg.get('uturn_trial_enabled',False))
    boolean(cfg.get('uturn_trial_resume_lane',False))
    if boolean(cfg.get('uturn_course_test',False)):
        if not trial or not cfg.get('uturn_trial_resume_lane',False) or 'uturn_trial_sequence' not in cfg:
            raise ValueError('course test requires timed sequence and lane resume')
    if trial:
        if cfg.get('uturn_relative_enabled',False):
            raise ValueError('choose one U-turn mode: trial or relative')
        from robot.uturn.timed import TimedUturn
        TimedUturn(cfg)
    if not .003 <= number(cfg.get('parking_goal_tolerance',.01)) <= .05:
        raise ValueError('parking_goal_tolerance must be in [0.003,0.05] m')
    for key,default,low,high in (('straight_align_distance',1.6,.3,2.5),
                               ('straight_align_tolerance_deg',5,1,15),
                               ('straight_blue_clearance',.10,.05,.40)):
        if not low <= number(cfg.get(key,default)) <= high:
            raise ValueError('invalid '+key)
    for retired in ('uturn_initial_turn_s', 'stop_at_next_blue', 'straight_search_speed_raw'):
        if retired in cfg:
            raise ValueError('retired parameter: '+retired)
    frames = number(cfg.get('exit_frames',3))
    if frames != int(frames) or not 2 <= frames <= 30:
        raise ValueError('exit_frames must be an integer in [2,30]')
    parking_votes = number(cfg.get('parking_sign_votes',3))
    direction_votes = number(cfg.get('direction_sign_votes',2))
    if direction_votes != int(direction_votes) or not 1 <= direction_votes <= 10:
        raise ValueError('direction_sign_votes must be an integer in [1,10]')
    if parking_votes != int(parking_votes) or not 1 <= parking_votes <= 10:
        raise ValueError('parking_sign_votes must be an integer in [1,10]')
    for key,default,low,high in (('parking_visible_side_m',.18,.12,.35),
                               ('parking_capture_yaw_deg',45,20,55),
                               ('parking_near_start_tolerance_m',.12,.03,.15)):
        if not low<=number(cfg.get(key,default))<=high:
            raise ValueError('invalid '+key)
    for key,default,upper in (('uturn_align_heading_deg',8,30),
            ('uturn_align_stable_s',.5,5),
            ('uturn_alignment_max_distance',.5,1),('uturn_blue_blind_entry_m',.6,1),
            ('uturn_blue_blind_max_m',.35,.6),
            ('uturn_reverse_yaw_rad',.15,.5),
            ('uturn_rear_blind_entry_m',.2,.4),('uturn_rear_blind_max_m',.6,1),
            ('uturn_axle_tolerance_m',.06,.15)):
        if not 0 < number(cfg.get(key,default)) <= upper:
            raise ValueError('invalid '+key)
    for key in ('uturn_align_speed_raw','uturn_reverse_speed_raw'):
        if not 0 < number(cfg.get(key,12)) <= cfg['speed_raw_limit']:
            raise ValueError('invalid '+key)
    if not 0 <= number(cfg.get('uturn_brake_lead_m',.02)) <= cfg.get('uturn_axle_tolerance_m',.06):
        raise ValueError('uturn brake lead exceeds axle tolerance')
    mode = cfg.get('parking_mode','reverse_plan')
    boolean(cfg.get('timed_bypass_enabled',False))
    if not 0 <= number(cfg.get('timed_bypass_settle_s',0.)) <= 2:
        raise ValueError('timed_bypass_settle_s must be in [0,2]')
    if not 0 < number(cfg.get('timed_bypass_left_s',2.)) <= 6:
        raise ValueError('timed_bypass_left_s must be in (0,6]')
    if not 0 < number(cfg.get('timed_bypass_right_s',6.)) <= 12:
        raise ValueError('timed_bypass_right_s must be in (0,12]')
    trigger_distance = number(cfg.get('timed_bypass_trigger_distance_m',.50))
    if not cfg['wheelbase']+cfg['front_overhang']+cfg['obstacle_margin'] < trigger_distance <= 2.0:
        raise ValueError('timed_bypass_trigger_distance_m must exceed the front footprint and be <= 2 m')
    boolean(cfg.get('parking_sign_association',False))
    if mode not in ('reverse_plan','forward_white','parallel_reverse','forward_center','forward_plan'):
        raise ValueError('invalid parking_mode')
    if mode == 'parallel_reverse' and cfg['parking_slot'] != 'AUTO':
        slot = cfg.get('slots',{}).get(cfg['parking_slot'],{})
        if slot.get('kind') != 'parallel':
            raise ValueError('parallel_reverse requires AUTO or a parallel slot')
    if mode in ('parallel_reverse','forward_plan'):
        for key,default,low,high in (
                ('parallel_parking_turn_radius_m',.65,.1,3),
                ('parallel_parking_max_setup_m',2,.05,3),
                ('parallel_parking_sample_step_m',.025,.005,.05),
                ('parallel_parking_double_arc_yaw_tolerance_deg',3,.1,10),
                ('parallel_parking_goal_position_tolerance_m',.03,.005,.10),
                ('parallel_parking_goal_yaw_tolerance_deg',10,1,15),
                ('parallel_parking_tracking_error_m',.15,.02,.3),
                ('parallel_parking_slot_drift_m',.12,.01,.2),
                ('parallel_parking_slot_size_tolerance_m',.05,.001,.2),
                ('parallel_parking_slot_body_margin_m',.01,0,.05),
                ('parallel_parking_timeout_s',60,5,120)):
            if not low <= number(cfg.get(key,default)) <= high:
                raise ValueError('invalid '+key)
        for key,default,low,high in (
                ('parallel_parking_confirm_frames',3,2,30),
                ('parallel_parking_speed_raw',12,1,cfg['speed_raw_limit'])):
            value = number(cfg.get(key,default))
            if value != int(value) or not low <= value <= high:
                raise ValueError('invalid '+key)
    if mode == 'forward_white' and (cfg['parking_slot']!='AUTO' or not cfg.get('lidar_enabled',True)):
        raise ValueError('forward_white requires parking_slot AUTO and lidar_enabled true')
    for key,default,low,high in (
            ('parking_bottom_clearance_m',.04,.01,.15),
            ('parking_align_heading_deg',6,1,15),('parking_align_lateral_m',.035,.01,.08),
            ('parking_align_stable_s',.5,.1,2),('parking_final_max_m',.30,.03,.5),
            ('parking_entry_timeout_s',35,5,90),('parking_edge_match_m',.07,.02,.12),
            ('parking_edge_match_deg',18,5,30)):
        if not low <= number(cfg.get(key,default)) <= high:
            raise ValueError('invalid '+key)
    for key,default in (('parking_entry_speed_raw',12),('parking_final_speed_raw',8)):
        value = number(cfg.get(key,default))
        if value != int(value) or not 1 <= value <= min(20,cfg['speed_raw_limit']):
            raise ValueError('invalid '+key)
    if not .05 <= number(cfg.get('parking_divider_trigger_x',cfg['wheelbase']+cfg['front_overhang'])) <= 1:
        raise ValueError('invalid parking_divider_trigger_x')
    boolean(cfg.get('right_direct_left_follow',False))
    boolean(cfg.get('right_exit_on_blue',False))
    boolean(cfg.get('uturn_relative_enabled',False))
    from robot.uturn.planner import search_options
    search_options(cfg)
    if cfg.get('uturn_followed_boundary','AUTO') not in ('AUTO','LEFT','RIGHT'):
        raise ValueError('uturn_followed_boundary must be AUTO, LEFT or RIGHT')
    for key,default,low,high in (
            ('uturn_lane_spacing',.60,.05,2),
            ('uturn_goal_forward_step',.10,.01,1),
            ('uturn_goal_heading_tolerance_deg',10,1,45),
            ('uturn_tracking_error_max',.15,.01,.75),
            ('uturn_replan_cooldown_s',1,0,10)):
        if not low <= number(cfg.get(key,default)) <= high:
            raise ValueError('invalid '+key)
    forward_min = number(cfg.get('uturn_goal_forward_min',0))
    forward_max = number(cfg.get('uturn_goal_forward_max',0))
    if not -2 <= forward_min <= forward_max <= 2:
        raise ValueError('invalid uturn longitudinal goal interval')
    replans = number(cfg.get('uturn_replan_limit',0))
    if replans != int(replans) or not 0 <= replans <= 3:
        raise ValueError('uturn_replan_limit must be an integer in [0,3]')
    if cfg.get('right_exit_on_blue',False) and not cfg.get('right_turn_full_lock',False):
        raise ValueError('right_exit_on_blue requires right_turn_full_lock')
    if not .10 <= number(cfg.get('right_left_capture_max_m',.80)) <= 1.20:
        raise ValueError('right_left_capture_max_m must be in [0.10,1.20] m')
    if not .05 <= number(cfg.get('right_align_lookahead',.30)) <= 2:
        raise ValueError('right_align_lookahead must be in [0.05,2] m')
    for key,default,upper in (('left_reference_heading_deg',8,40),
                              ('left_reference_lateral_m',.08,.30),
                              ('left_reference_stable_s',.5,5),
                              ('left_reference_raw_rate',40,200)):
        if not 0 < number(cfg.get(key,default)) <= upper:
            raise ValueError('invalid '+key)
    if not 0 < number(cfg.get('left_reference_speed_raw',12)) <= cfg['speed_raw_limit']:
        raise ValueError('invalid left_reference_speed_raw')
    for key,default in (('sign_confidence',.6),('red_sign_confidence',.8),('sign_width_m',.195),('sign_max_distance_m',.6)):
        if not 0 < number(cfg.get(key,default)) <= (1 if key in ('sign_confidence','red_sign_confidence') else 5):
            raise ValueError('invalid '+key)
    if not 0 <= number(cfg.get('uturn_reverse_distance_m',0)) <= 3:
        raise ValueError('uturn_reverse_distance_m must be in [0,3]')
    for key,default,upper in (('straight_wait_s',1.0,60),
                              ('blue_align_duration_s',1.0,5),
                              ('straight_right_blue_offset_m',.30,1),
                              ('blue_aligned_advance_m',.36,2),
                              ('straight_search_max_distance',.8,3),
                              ('straight_search_timeout',10.0,60),
                              ('straight_search_heading_deg',20,45)):
        if not 0 < number(cfg.get(key,default)) <= upper:
            raise ValueError('invalid '+key)
    if not 0 < number(cfg.get('straight_speed_raw',18)) <= cfg['speed_raw_limit']:
        raise ValueError('invalid straight_speed_raw')
    trim = number(cfg.get('startup_steering_raw',0))
    if trim != int(trim) or abs(trim) > 5:
        raise ValueError('startup_steering_raw must be an integer in [-5,5]')
    if not .05 <= number(cfg.get('lane_min_path_span',.15)) <= .30:
        raise ValueError('lane_min_path_span must be 0.05..0.30 m')
    if not .01 <= number(cfg.get('lane_lateral_full_scale_m',.075)) <= 2.:
        raise ValueError('lane_lateral_full_scale_m must be 0.01..2.0 m')
    boolean(cfg.get('lane_curvature_preview',False))
    curve_speed = number(cfg.get('lane_curve_speed_raw',12))
    if curve_speed != int(curve_speed) or not 1 <= curve_speed <= 20:
        raise ValueError('lane_curve_speed_raw must be an integer in [1,20]')
    for name in ('wait_green','bypass_enabled','parking_blue_required','planner_require_initial_forward'):
        boolean(cfg[name])
    boolean(cfg['lidar']['unknown_is_obstacle'])
    boolean(cfg.get('lidar_enabled',True))
    boolean(cfg['lidar'].get('shape_filter',False))
    lidar = cfg['lidar']
    if lidar.get('shape_mode','line_reject') not in ('line_reject','round_only'):
        raise ValueError('invalid lidar shape_mode')
    if not 0 < number(lidar.get('line_max_ratio',.025)) <= .25:
        raise ValueError('invalid line_max_ratio')
    if not number(lidar.get('line_max_ratio',.025)) < number(lidar.get('obstacle_min_ratio',.05)) <= .5:
        raise ValueError('obstacle_min_ratio must exceed line_max_ratio')
    if not 0 < number(lidar.get('line_max_bend_deg',10)) < number(lidar.get('obstacle_min_bend_deg',25)) <= 90:
        raise ValueError('invalid line/obstacle bend interval')
    if not 0 < number(lidar.get('line_inlier_ratio',.9)) <= 1:
        raise ValueError('invalid line_inlier_ratio')
    if not 0 < number(lidar.get('compact_min_span',.015)) < number(lidar.get('compact_max_span',.50)) <= 2:
        raise ValueError('invalid compact span interval')
    if not .005 <= number(lidar.get('round_radius_min',.025)) < number(lidar.get('round_radius_max',.25)) <= 1:
        raise ValueError('invalid lidar round radius interval')
    for key,default,maximum in (('cluster_gap',.045,.3),('round_fit_error',.008,.1),
                                ('round_min_arc_deg',40,180),('cluster_min_points',6,100),
                                ('flat_line_error',.0025,.02),('flat_min_width',.04,1),('wall_min_width',.55,6)):
        if not 0 < number(lidar.get(key,default)) <= maximum:
            raise ValueError('invalid lidar '+key)
    count = number(lidar.get('cluster_min_points',6))
    if count < 6 or count != int(count):
        raise ValueError('lidar cluster_min_points must be an integer >= 6')
    count = number(lidar.get('shape_max_clusters',128))
    if not 1 <= count <= 512 or count != int(count):
        raise ValueError('invalid shape_max_clusters')
    if lidar.get('wall_min_width',.55) <= lidar.get('flat_min_width',.04):
        raise ValueError('wall_min_width must exceed flat_min_width')
    boolean(cfg.get('parking_enabled',True))
    boolean(cfg.get('latch_direction_sign',False))
    boolean(cfg.get('right_turn_full_lock',False))
    minimum = number(cfg.get('right_lock_min_angle_deg',55))
    maximum = number(cfg.get('right_lock_max_angle_deg',120))
    if not 0 < minimum < maximum < 180:
        raise ValueError('right lock angles must satisfy 0 < min < max < 180')
    if not 0 < number(cfg.get('right_lock_heading_deg',15)) <= 40:
        raise ValueError('right_lock_heading_deg must be in (0,40]')
    for key,default in (('ground_timeout',1.25),('slot_hz',2.0)):
        if not 0 < number(cfg.get(key,default)) <= (4.0 if key == 'ground_timeout' else 30.0):
            raise ValueError('invalid '+key)
    if not 0 <= number(cfg.get('intersection_wait_s',0.0)) <= 60:
        raise ValueError('invalid intersection_wait_s')
    if not 1 <= number(cfg.get('turn_angle_deg',90.0)) <= 180:
        raise ValueError('invalid turn_angle_deg')
    if not 0 < number(cfg.get('uturn_reverse_max_distance',1.0)) <= 3:
        raise ValueError('invalid uturn_reverse_max_distance')
    if not 0 <= number(cfg.get('right_reverse_entry_m',0.0)) <= 3:
        raise ValueError('invalid right_reverse_entry_m')
    if not 0 < number(cfg.get('right_lock_command_rad',0.05)) <= 1.2:
        raise ValueError('invalid right_lock_command_rad')
    for side in ('left','right'):
        for key in ('turn_entry','turn_radius','turn_exit','turn_angle_deg'):
            name = side+'_'+key
            if name not in cfg:
                continue
            value = number(cfg[name])
            if key == 'turn_angle_deg':
                valid = 1 <= value <= 180
            elif key == 'turn_radius':
                valid = 0 < value <= 10
            else:
                valid = 0 <= value <= 10
            if not valid:
                raise ValueError('invalid '+name)
    if not 0 < number(cfg.get('action_lookahead',cfg['lookahead'])) <= 2:
        raise ValueError('invalid action_lookahead')
    if cfg['pose_mode'] not in ('command_model','odom'):
        raise ValueError('pose_mode must be command_model or odom')
    boolean(cfg.get('left_turn_full_lock',False))
    if not .1 <= number(cfg.get('lidar_timeout',cfg['sensor_timeout'])) <= 2.:
        raise ValueError('lidar_timeout must be in [0.1,2.0] s')
    for name in ('wheelbase','track','body_width','max_steer','lookahead','parking_lookahead',
                 'sensor_timeout','sign_timeout','odom_timeout','action_timeout','planner_step',
                 'planner_xy_resolution','planner_yaw_resolution','planner_timeout','ground_hz','sign_hz',
                 'steering_raw_limit','speed_raw_limit','obstacle_stop_distance','obstacle_preview_distance'):
        if not 0 < number(cfg[name]) <= 1000:
            raise ValueError('invalid positive parameter: '+name)
    if not 0.05 <= cfg['max_steer'] <= 1.2:
        raise ValueError('max_steer must be radians in [0.05,1.2]')
    if not .01 <= number(cfg.get('steering_command_scale_rad',cfg['max_steer'])) <= 1.2:
        raise ValueError('steering_command_scale_rad must be in [0.01,1.2]')
    if cfg['steering_raw_limit'] > 22 or cfg['speed_raw_limit'] > 100:
        raise ValueError('raw limit exceeds current bridge contract')
    for name in ('steering_sign','speed_sign'):
        if number(cfg[name]) not in (-1,1):
            raise ValueError('direction sign must be +/-1')
    for value in cfg['speed_raw'].values():
        if not 0 <= number(value) <= cfg['speed_raw_limit']:
            raise ValueError('invalid speed_raw')
    for value in cfg['raw_to_mps'].values():
        if not 0 < number(value) < 1:
            raise ValueError('invalid raw_to_mps coefficient')
    if cfg['parking_slot'] != 'AUTO' and cfg['parking_slot'] not in cfg['slots']:
        raise ValueError('unknown requested slot')
    for name,default,upper in (('parking_observe_s',.8,10),('parking_prep_offset',0,3)):
        if not 0 <= number(cfg.get(name,default)) <= upper:
            raise ValueError('invalid '+name)
    if not 0 < number(cfg.get('parking_sync_s',.5)) <= 1.25:
        raise ValueError('invalid parking_sync_s')
    if not 0 < number(cfg['parking_road_half_width']) <= 3:
        raise ValueError('invalid parking_road_half_width')
    if not 0 <= number(cfg['parking_visual_pose_gain']) <= 1:
        raise ValueError('parking_visual_pose_gain must be in [0,1]')
    for name in ('parking_visual_max_shift','parking_visual_max_yaw_shift'):
        if not 0 < number(cfg[name]) <= 0.2:
            raise ValueError('invalid visual correction bound')


def decode(raw, now, timeout):
    if len(raw) > 65536:
        raise ValueError('message too large')
    data = json.loads(raw)
    if not isinstance(data,dict):
        raise ValueError('object required')
    stamp = number(data['stamp'])
    if not 0 <= now-stamp <= timeout:
        raise ValueError('source stamp stale or future')
    return data,stamp


def ground(raw,now,timeout):
    data,stamp = decode(raw,now,timeout)
    if 'slot_diagnostic' in data:
        diag=data['slot_diagnostic']
        allowed=('raw_segments','merged_segments','pairs','reject_parallel','reject_width',
                 'reject_short','reject_anchor','accepted')
        if not isinstance(diag,dict) or any(k not in allowed for k in diag):
            raise ValueError('invalid slot diagnostics')
        for value in diag.values():
            v=number(value)
            if not 0<=v<=100000 or v!=int(v):
                raise ValueError('invalid slot diagnostic count')
    if data.get('frame') != 'base_link' or data.get('source') not in ('front','rear'):
        raise ValueError('unknown coordinate frame/source')
    if data.get('part','all') not in ('all','markers','slots','parking_lines'):
        raise ValueError('unknown observation part')
    lines=data.get('blue_lines',[])
    if not isinstance(lines,list) or len(lines)>30:
        raise ValueError('invalid blue lines')
    for line in lines:
        if (any(abs(number(line[k]))>8 for k in ('x','y')) or
                abs(number(line['yaw']))>math.pi/2+.001 or
                not 0<number(line['length'])<=8):
            raise ValueError('invalid blue line geometry')
    if data.get('part') == 'parking_lines':
        if data.get('source') != 'front' or data.get('markers') or data.get('slots'):
            raise ValueError('parking lines must be an independent front observation')
        lines = data.get('lines')
        if not isinstance(lines,list) or len(lines)>80:
            raise ValueError('invalid parking lines')
        for line in lines:
            if not isinstance(line,list) or len(line)!=2:
                raise ValueError('invalid line endpoints')
            for point in line:
                if (not isinstance(point,list) or len(point)!=2 or
                        any(abs(number(v))>8 for v in point)):
                    raise ValueError('invalid parking endpoint')
    for name,limit in (('markers',30),('slots',20)):
        if not isinstance(data.get(name),list) or len(data[name]) > limit:
            raise ValueError('invalid observations')
        for item in data[name]:
            for field in ('x','y'):
                if abs(number(item[field])) > 8:
                    raise ValueError('observation outside local range')
            kinds = ('junction','tick') if name == 'markers' else ('parallel','perpendicular')
            if item['kind'] not in kinds:
                raise ValueError('invalid kind')
            if name == 'slots':
                number(item['yaw'])
    if data.get('part') == 'markers' and data['slots']:
        raise ValueError('markers part contains slots')
    if data.get('part') == 'slots' and data['markers']:
        raise ValueError('slots part contains markers')
    return data,stamp


def encode_command(speed,steer,cfg,seq):
    speed,steer = number(speed),number(steer)
    speed = int(round(max(-cfg['speed_raw_limit'],min(cfg['speed_raw_limit'],speed))*cfg['speed_sign']))
    scale = cfg.get('steering_command_scale_rad',cfg['max_steer'])
    raw = steer/scale*cfg['steering_raw_limit']*cfg['steering_sign']
    raw = int(round(max(-cfg['steering_raw_limit'],min(cfg['steering_raw_limit'],raw))))
    return dict(version=1,seq=int(seq)%256,speed_raw=speed,steering_raw=raw)
