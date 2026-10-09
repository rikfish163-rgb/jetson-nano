"""Bicycle estimate from timestamped applied commands, never measured odometry."""
from robot.common.geometry import bicycle


class CommandOdometry(object):
    def __init__(self, pose, wheelbase, timeout=.25):
        self.pose, self.wheelbase, self.timeout = tuple(pose), wheelbase, timeout
        self.stamp, self.velocity, self.steering = None, 0., 0.

    def estimate(self, now):
        if self.stamp is None:
            return self.pose
        dt = max(0., min(self.timeout, now-self.stamp))
        return bicycle(self.pose, self.velocity*dt, self.steering, self.wheelbase)

    def observe(self, stamp, velocity, steering):
        if self.stamp is not None and stamp <= self.stamp:
            raise ValueError('command odometry source stamp out of order')
        # Commit the previous held command to the new command's source stamp.
        # Estimates between feedback samples are extrapolated, not accumulated,
        # so delayed stop feedback can correct that interval without double count.
        self.pose = self.estimate(stamp)
        self.stamp, self.velocity, self.steering = stamp, velocity, steering

    def stop(self, now):
        self.pose = self.estimate(now)
        self.stamp, self.velocity, self.steering = now, 0., 0.
