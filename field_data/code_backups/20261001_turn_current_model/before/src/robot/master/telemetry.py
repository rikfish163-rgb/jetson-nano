"""Read-only status serialization, separated from the 20 Hz control adapter."""
import math
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.common.planning import turn_parameters
from robot.common.contracts import encode_command


def active_scene_task(core):
    if core.state == 'UTURN':
        return core.relative_uturn
    if core.state in ('PARALLEL_PARKING','FINISHED'):
        return core.parallel_parking
    return None


def scene_maneuver_status(core, now):
    task = active_scene_task(core)
    if task is None:
        return None
    follower = task.follower
    path = follower.path[follower.index:] if follower else []
    replanning = core.state == 'UTURN' and core.relative_replan_future is not None
    return dict(kind='UTURN' if core.state == 'UTURN' else 'PARALLEL_PARKING',
                phase='REPLAN' if replanning else task.phase,
                reason='relative_uturn_replanning' if replanning else task.reason,frame=task.frame,
                pose=task.scene['pose'],pose_source=task.scene.get('pose_source','measured_scene'),
                observation_age=now-task.scene['stamp'],goal=task.goal,
                goal_error_m=math.hypot(task.goal[0]-task.scene['pose'][0],
                                       task.goal[1]-task.scene['pose'][1]),
                goal_heading_error_deg=math.degrees(wrap(task.goal[2]-task.scene['pose'][2])),
                slot=task.scene.get('slot'),
                followed_boundary=task.scene.get('followed_boundary'),
                replan_attempts=getattr(task,'replan_attempts',0),
                # These points are in frame, never in the command-model BEV.
                path=path[:300],regions=task.scene['regions'])


def active_planned_path(core):
    if active_scene_task(core) is not None:
        return []
    if core.follower is None or core.right_lock is not None or core.straight_search is not None:
        return []
    if core.state == 'UTURN' and (core.uturn or {}).get('phase') not in ('LEFT_FIRST','LEFT_SECOND'):
        return []
    if core.state not in ('MANEUVER','PARKING','UTURN'):
        return []
    return core.follower.path[core.follower.index:]


def controller_status(core, cfg, now, live, enabled, command, seq, lane_observation=None):
    turn_action = core.action or core.pending or core.last_completed_action
    status = dict(stamp=now,state=core.state,reason=core.reason,
        uturn_course=(dict(stop_pending=core.course_stop_pending,
            stop_armed=core.course_stop_armed,clear_frames=core.course_clear_frames,
            finished=core.course_finished) if cfg.get('uturn_course_test',False) else None),
        live=live,enabled=enabled,pose_mode=cfg['pose_mode'],pose=core.pose,
        pending=core.pending,slot=core.slot,slot_age=now-core.slot_stamp,
        parking_candidates=core.parking_diagnostics,
        parking_detection=core.parking_detection,
        parking_entry=core.parking_entry.debug if core.parking_entry else None,
        parking_mode=cfg.get('parking_mode','reverse_plan'),
        parking_front_age=now-core.parking_candidate_stamp,
        parking_approach_m=core.slot.get('approach_distance') if core.slot else None,
        pending_age=now-core.pending_at if core.pending else None,
        next_direction=core.next_direction,
        next_direction_age=now-core.next_direction_at if core.next_direction else None,
        sign=core.sign_info,
        sign_age=now-core.sign_stamp if core.sign_stamp >= 0 else None,
        lane_confidence=core.lane_confidence,lane_points=len(core.lane),
        lane_observation=lane_observation,
        lane_source=core.lane_source,right_lock=core.right_lock,
        lane_curve_lock=core.lane_curve_lock,
        lane_speed_state=core.lane_speed_state,
        timed_bypass_candidate=core.timed_bypass_candidate,
        startup_curve_lock=core.startup_curve_lock,
        startup_straight=(dict(distance_m=local(core.startup_origin,core.pose)[0],
            target_m=max(1.25,cfg.get('straight_distance',1.25)),
            distance_source=cfg['pose_mode'],slowdown=core.startup_curve_lock is not None)
            if core.state == 'STARTUP_STRAIGHT' and core.startup_origin is not None else None),
        lane_recovery=core.lane_recovery,
        straight_search=core.straight_search,timed_bypass=core.timed_bypass,
        timed_bypass_completed=core.timed_bypass_completed,
        exit_confirmation=dict(frames=core.exit_count,
            reason=getattr(core,'exit_reason','not_checked')),
        right_turn_debug=dict(
            exit_on_blue=cfg.get('right_exit_on_blue',False),
            left_reference=core.left_reference,
            left_fit=core.left_fit_diagnostic,
            left_capture_max_m=cfg.get('right_left_capture_max_m',.80),
            reference_heading_deg=cfg.get('left_reference_heading_deg',8),
            reference_lateral_m=cfg.get('left_reference_lateral_m',.08),
            reference_stable_s=cfg.get('left_reference_stable_s',.5),
            align_lookahead_m=cfg.get('right_align_lookahead',.30),
            lane_lookahead_m=cfg['lookahead'],
            direct_left_follow=cfg.get('right_direct_left_follow',False),
            following_left=core.follow_left_boundary,
            command_scale=cfg.get('steering_command_scale_rad',cfg['max_steer']),
            lock_command=-cfg.get('right_lock_command_rad',.05),
            encoded_command=encode_command(command[0],command[1],cfg,seq),
            reverse_entry_m=cfg.get('right_reverse_entry_m',0),
            left_entry_m=cfg.get('left_turn_entry',cfg['turn_entry']),
            min_turn_deg=cfg.get('right_lock_min_angle_deg',55),
            max_turn_deg=cfg.get('right_lock_max_angle_deg',120),
            capture_heading_deg=50,
            aligned_heading_deg=cfg.get('right_lock_heading_deg',15),
            aligned_lateral_m=cfg['exit_lateral_tolerance'],
            required_frames=cfg['exit_frames'],
            sensor_timeout=cfg['sensor_timeout'],
            lane_min_confidence=cfg['lane_min_confidence']),
        lidar_enabled=cfg.get('lidar_enabled',True),
        lidar_shape_filter=cfg['lidar'].get('shape_filter',False),
        lidar_shape_mode=cfg['lidar'].get('shape_mode','line_reject'),
        lidar_clusters=core.scan.cluster_diagnostics[:32] if core.scan else [],
        lidar_cluster_frame=cfg['scan_frame'],
        lidar_cluster_stamp=core.scan.stamp if core.scan else None,
        lidar_completed_stamp=getattr(core.scan,'completed_stamp',core.scan.stamp) if core.scan else None,
        lidar_processing_ms=getattr(core.scan,'processing_ms',None) if core.scan else None,
        lidar_cluster_total=len(core.scan.cluster_diagnostics) if core.scan else 0,
        left_boundary_points=len(core.left_boundary),
        left_boundary_age=now-core.left_boundary_stamp if core.left_boundary_stamp >= 0 else None,
        gap_steer=core.gap_steer,
        obstacle_check=core.obstacle_check if enabled else dict(kind='not_checked'),
        marker_local=local(core.pose,core.marker[0]) if core.marker else None,
        marker_age=now-core.marker[1] if core.marker else None,
        last_blue_trigger=core.last_blue_trigger,
        blue_approach=core.blue_approach,
        blue_consumed=core.blue_consumed,
        wait_remaining=(max(0,core.blue_approach['stop_until']-now)
            if core.state == 'BLUE_STOP' and core.blue_approach is not None else
            max(0,core.wait_until-now) if core.state == 'INTERSECTION_WAIT' else 0),
        action=core.action,
        action_source=core.action_source,
        blue_default_straight=cfg.get('blue_default_straight',False),
        uturn=core.uturn,
        scene_maneuver=scene_maneuver_status(core,now),
        rear_marker_age=now-core.rear_marker_stamp if core.rear_marker_stamp>=0 else None,
        rear_markers_local=[local(core.pose,p) for p in core.rear_markers],
        front_blue_lines_local=[dict(point=local(core.pose,b['point']),
            heading_deg=math.degrees(wrap(b['yaw']-core.pose[2])),length=b['length'])
            for b in core.front_blue_lines],
        front_marker_age=now-core.front_marker_stamp if core.front_marker_stamp >= 0 else None,
        front_markers_local=[local(core.pose,p) for p in core.front_markers],
        last_completed_action=core.last_completed_action,
        turn_action=turn_action,
        turn=dict(turn_parameters(cfg,'LEFT' if turn_action == 'UTURN' else turn_action),
                  intersection_wait_s=cfg.get('intersection_wait_s',0.0)),
        turn_profiles=dict(left=turn_parameters(cfg,'LEFT'),
                           right=turn_parameters(cfg,'RIGHT')),
        source_ages=dict(lane=now-core.lane_stamp,scan=now-core.scan.stamp if core.scan else None,
            scan_completed=now-getattr(core.scan,'completed_stamp',core.scan.stamp) if core.scan else None,
                         front_ground=now-core.front_marker_stamp,rear_ground=now-core.rear_marker_stamp),
        lane_path_local=[local(core.pose,p) for p in core.lane[:200]],
        lane_path_fresh=0 <= now-core.lane_stamp <= cfg['sensor_timeout'],
        planned_path_local=[local(core.pose,p) for p in active_planned_path(core)[:200]],
        pose_evidence='command_estimate' if cfg['pose_mode']=='command_model' else 'odom_input',
        command=command,estop=core.estop)
    return status
