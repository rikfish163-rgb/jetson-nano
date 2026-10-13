"""Stopped-only side bay confirmation. No ROS or actuator dependencies."""
from robot.common.contracts import number
from robot.parallel_parking.open_loop_core import finite


def validate_settings(cfg):
    options = cfg.get('parking_side_lidar')
    if not isinstance(options,dict):
        raise ValueError('parking_side_lidar settings required')
    for key,lo,hi,integer in (
            ('min_points',1,2000,True),('confirm_frames',1,20,True),
            ('scan_timeout_s',.1,2.,False),('selection_timeout_s',1.,120.,False),
            ('min_valid_rays',8,5000,True),('processing_hz',.5,10.,False)):
        finite(options.get(key),key,lo,hi,integer)
    return options


class SideSelection(object):
    def __init__(self,cfg,started):
        self.cfg,self.options,self.started = cfg,validate_settings(cfg),started
        self.last_stamp = None
        self.candidate,self.votes,self.slot = None,0,None
        self.counts,self.reason = None,'WAIT_LIDAR'

    def reject(self,reason):
        if self.slot is None:
            self.candidate,self.votes,self.counts,self.reason = None,0,None,reason

    def observe(self,data,now):
        if self.slot is not None:
            return
        if (not isinstance(data,dict) or data.get('source') != 'side_parking_lidar' or
                data.get('frame') != self.cfg['scan_frame']):
            raise ValueError('side bay source/frame')
        stamp = number(data.get('stamp'))
        if number(data.get('request_stamp')) != self.started or stamp <= self.started:
            self.reject('WAIT_POST_STOP_SCAN')
            return
        if not 0 <= now-stamp <= self.options['scan_timeout_s']:
            self.reject('LIDAR_STALE')
            return
        # Repeated telemetry/worker output never supplies additional votes.
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        if self.last_stamp is not None and stamp-self.last_stamp > self.options['scan_timeout_s']:
            self.reject('LIDAR_GAP')
        self.last_stamp = stamp
        if data.get('valid') is not True:
            self.reject('NO_VALID_REFERENCE')
            return
        line = data.get('line')
        if not isinstance(line,dict):
            raise ValueError('missing reference line')
        if (number(line.get('count')) < 8 or number(line.get('length')) < .12 or
                not 0 <= number(line.get('rms')) <= .02):
            raise ValueError('invalid reference line')
        midpoint = line.get('midpoint')
        if not isinstance(midpoint,(list,tuple)) or len(midpoint) != 2:
            raise ValueError('invalid reference midpoint')
        x,y = map(number,midpoint)
        if not x > 0 or not y < 0 or x*x+y*y > 1.:
            raise ValueError('reference outside viewer ROI')
        rays = number(data.get('valid_rays'))
        if rays != int(rays) or rays < self.options['min_valid_rays'] or rays > 5000:
            raise ValueError('invalid scan coverage')
        counts = data.get('counts')
        if not isinstance(counts,dict) or set(counts) != set(('1','2')):
            raise ValueError('missing side bay counts')
        values = [number(counts[key]) for key in ('1','2')]
        if any(v != int(v) or not 0 <= v <= rays for v in values):
            raise ValueError('invalid side bay counts')
        occupied = [v >= self.options['min_points'] for v in values]
        self.counts = dict(zip(('1','2'),map(int,values)))
        if all(occupied):
            self.reject('BOTH_OCCUPIED_STOPPED')
            self.counts = dict(zip(('1','2'),map(int,values)))
            return
        slot = 'P1' if occupied[0] else 'P2' if occupied[1] else 'P3'
        self.votes = self.votes+1 if slot == self.candidate else 1
        self.candidate,self.reason = slot,'CONFIRM_'+slot
        if self.votes >= self.options['confirm_frames']:
            self.slot,self.reason = slot,'SELECTED_'+slot

    def status(self):
        return dict(started=self.started,counts=self.counts,candidate=self.candidate,
                    votes=self.votes,slot=self.slot,reason=self.reason,
                    last_stamp=self.last_stamp,min_points=self.options['min_points'])
