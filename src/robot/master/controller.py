"""Stable ROS facade. The coordinator alone applies module state updates."""
from robot.common.executor import _SimpleFuture
from robot.common.executor import _SingleJobExecutor
from robot.master.state_machine import initial_state
from robot.master.runtime import ModuleRuntime


class Controller(object):
    def __init__(self, cfg):
        object.__setattr__(self, '_state_data', initial_state(cfg))
        object.__setattr__(self, '_runtime', ModuleRuntime(
            parallel_enabled=cfg.get('parking_mode') in ('parallel_reverse','forward_plan')))

    def __getattr__(self, name):
        try:
            return self._state_data[name]
        except KeyError:
            raise AttributeError(name)

    def __setattr__(self, name, value):
        # Compatibility for ROS observations, diagnostics and existing fixtures.
        if name in self._runtime.owners:
            self._runtime.overrides[name] = value
        else:
            self._state_data[name] = value

    def execute(self, module, operation, *args, **kwargs):
        result = self._runtime.execute(module, operation, self._state_data, *args, **kwargs)
        self._state_data.update(result.updates)
        return result

    def close(self, *args, **kwargs):
        return self.execute('mission', 'close', *args, **kwargs).value

    def set_pose(self, *args, **kwargs):
        return self.execute('mission', 'set_pose', *args, **kwargs).value

    def start_follow(self, *args, **kwargs):
        return self.execute('mission', 'start_follow', *args, **kwargs).value

    def resume_lane(self, *args, **kwargs):
        return self.execute('mission', 'resume_lane', *args, **kwargs).value

    def dispatch(self, *args, **kwargs):
        return self.execute('mission', 'dispatch', *args, **kwargs).value

    def begin_startup(self, *args, **kwargs):
        return self.execute('mission', 'begin_startup', *args, **kwargs).value

    def course_blue_stop(self, *args, **kwargs):
        return self.execute('uturn', 'course_blue_stop', *args, **kwargs).value

    def startup_tick(self, *args, **kwargs):
        return self.execute('uturn', 'startup_tick', *args, **kwargs).value

    def tick(self, *args, **kwargs):
        return self.execute('mission', 'tick', *args, **kwargs).value

    def observe_lane(self, *args, **kwargs):
        return self.execute('camera', 'observe_lane', *args, **kwargs).value

    def observe_left_boundary(self, *args, **kwargs):
        return self.execute('camera', 'observe_left_boundary', *args, **kwargs).value

    def observe_ground(self, *args, **kwargs):
        return self.execute('camera', 'observe_ground', *args, **kwargs).value

    def observe_sign(self, *args, **kwargs):
        return self.execute('signs', 'observe_sign', *args, **kwargs).value

    def lane_valid(self, *args, **kwargs):
        return self.execute('lane', 'lane_valid', *args, **kwargs).value

    def lane_command(self, *args, **kwargs):
        return self.execute('lane', 'lane_command', *args, **kwargs).value

    def park_line_command(self, *args, **kwargs):
        return self.execute('lane', 'park_line_command', *args, **kwargs).value

    def left_reference_command(self, *args, **kwargs):
        return self.execute('lane', 'left_reference_command', *args, **kwargs).value

    def left_exit_line(self, *args, **kwargs):
        return self.execute('lane', 'left_exit_line', *args, **kwargs).value

    def front_straight_reference(self, *args, **kwargs):
        return self.execute('turn', 'front_straight_reference', *args, **kwargs).value

    def straight_heading_command(self, *args, **kwargs):
        return self.execute('turn', 'straight_heading_command', *args, **kwargs).value

    def straight_search_tick(self, *args, **kwargs):
        return self.execute('turn', 'straight_search_tick', *args, **kwargs).value

    def right_center_exit(self, *args, **kwargs):
        return self.execute('turn', 'right_center_exit', *args, **kwargs).value

    def right_blue_exit_tick(self, *args, **kwargs):
        return self.execute('turn', 'right_blue_exit_tick', *args, **kwargs).value

    def right_lock_tick(self, *args, **kwargs):
        return self.execute('turn', 'right_lock_tick', *args, **kwargs).value

    def direction_exit_reached(self, *args, **kwargs):
        return self.execute('turn', 'direction_exit_reached', *args, **kwargs).value

    def exit_lane_confirmed(self, *args, **kwargs):
        return self.execute('turn', 'exit_lane_confirmed', *args, **kwargs).value

    def uturn_phase(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_phase', *args, **kwargs).value

    def uturn_follow(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_follow', *args, **kwargs).value

    def uturn_outer_blue_confirmed(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_outer_blue_confirmed', *args, **kwargs).value

    def uturn_outer_blue_tick(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_outer_blue_tick', *args, **kwargs).value

    def uturn_blue_target(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_blue_target', *args, **kwargs).value

    def uturn_begin_alignment(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_begin_alignment', *args, **kwargs).value

    def uturn_align_first(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_align_first', *args, **kwargs).value

    def uturn_reverse_rear(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_reverse_rear', *args, **kwargs).value

    def uturn_fixed_exit_blue(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_fixed_exit_blue', *args, **kwargs).value

    def uturn_fixed_reverse(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_fixed_reverse', *args, **kwargs).value

    def uturn_trial_tick(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_trial_tick', *args, **kwargs).value

    def uturn_tick(self, *args, **kwargs):
        return self.execute('uturn', 'uturn_tick', *args, **kwargs).value

    def observe_relative_scene(self, *args, **kwargs):
        return self.execute('uturn', 'observe_relative_scene', *args, **kwargs).value

    def relative_uturn_tick(self, *args, **kwargs):
        return self.execute('uturn', 'relative_uturn_tick', *args, **kwargs).value

    def start_parking(self, *args, **kwargs):
        return self.execute('parking', 'start_parking', *args, **kwargs).value

    def auto_parking_tick(self, *args, **kwargs):
        return self.execute('parking', 'auto_parking_tick', *args, **kwargs).value

    def observe_parallel_scene(self, *args, **kwargs):
        if 'parallel_parking' not in self._runtime.modules:
            return
        return self.execute('parallel_parking', 'observe_parallel_scene', *args, **kwargs).value

    def observe_timed_parking(self, *args, **kwargs):
        return self.execute('camera','observe_timed_parking',*args,**kwargs).value

    def parallel_parking_tick(self, *args, **kwargs):
        return self.execute('parallel_parking', 'parallel_parking_tick', *args, **kwargs).value

    def scan_ready(self, *args, **kwargs):
        return self.execute('obstacle', 'scan_ready', *args, **kwargs).value

    def sweep_clear(self, *args, **kwargs):
        return self.execute('obstacle', 'sweep_clear', *args, **kwargs).value

    def checked_command(self, *args, **kwargs):
        return self.execute('obstacle', 'checked_command', *args, **kwargs).value

    def observe_applied_steering(self, *args, **kwargs):
        return self.execute('motion', 'observe_applied_steering', *args, **kwargs).value

    def current_steering(self, *args, **kwargs):
        return self.execute('motion', 'current_steering', *args, **kwargs).value

    def stop(self, *args, **kwargs):
        return self.execute('motion', 'stop', *args, **kwargs).value
    def follow_path_tick(self, *args, **kwargs):
        return self.execute('motion', 'follow_path_tick', *args, **kwargs).value
