"""Unit fixtures end entry at a chosen pose; no ROS publishers.

The car is placed 0.36 m before that pose, receives measured aligned blue
frames, advances to the pose, and uses zero wait. Full alignment motion and
the production one-second wait are tested separately.
"""
from robot.common.geometry import world


def observe_direction(c,label,confidence,stamp):
    """Supply two distinct accepted frames to route-control fixtures."""
    c.observe_sign(label,confidence,stamp-.05,stamp-.05)
    c.observe_sign(label,confidence,stamp,stamp)


def enter_blue_action(c, action, now):
    assert c.pending == action, (c.pending, action)
    wait=c.cfg['intersection_wait_s']
    align_duration=c.cfg.get('blue_align_duration_s',1.)
    c.cfg['intersection_wait_s']=0.
    c.cfg['blue_align_duration_s']=.10  # fixture compression; timer has dedicated tests
    endpoint=c.pose
    advance=c.cfg['blue_aligned_advance_m']
    c.pose=tuple(world(endpoint,(-advance,0)))+(endpoint[2],)
    c.pose_stamp=now-.5
    line_x=(advance+c.cfg['wheelbase']+c.cfg['front_overhang']+.02
            if action=='STRAIGHT' else .9)
    for t in (now-.39,now-.29,now-.19,now-.14,now-.09):
        c.set_pose(c.pose,t)
        c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[dict(kind='junction',x=line_x,y=0.,length=.6)],
            blue_lines=[dict(x=line_x,y=0.,yaw=0.,length=.6)]),t)
        c.tick(t)
    assert c.blue_approach['phase']==('STOP_LINE' if action=='STRAIGHT' else 'ADVANCE'), c.blue_approach
    c.set_pose(endpoint,now-.05)
    c.tick(now-.05)
    assert c.state=='BLUE_STOP', c.state
    c.set_pose(c.pose,now)
    c.tick(now)
    c.cfg['intersection_wait_s']=wait
    c.cfg['blue_align_duration_s']=align_duration
    assert c.action==action, (c.action,action)


def clear_blue(c, now):
    for t in (now-.4,now-.2,now):
        c.observe_ground(dict(source='front',part='markers',slots=[],markers=[]),t)
