"""In-process module boundary; no ROS, hardware publisher or Controller reference.

Containers are copied per request. Algorithm objects (followers, planners) and
executor/future handles are explicit persistent state, not sensor messages.
They retain their identity and must be operated only under the caller's lock.
"""
from collections import namedtuple

ModuleResult = namedtuple('ModuleResult', 'value updates')


def copy_containers(value):
    if isinstance(value, dict):
        return dict((key, copy_containers(item)) for key, item in value.items())
    if isinstance(value, list):
        return [copy_containers(item) for item in value]
    if isinstance(value, tuple):
        return tuple(copy_containers(item) for item in value)
    return value


class ModuleContext(object):
    """Declared data fields and service calls visible during one invocation."""
    __slots__ = ('_data', '_fields', '_calls', '_invoke')

    def __init__(self, data, fields, calls, invoke):
        object.__setattr__(self, '_data', data)
        object.__setattr__(self, '_fields', frozenset(fields))
        object.__setattr__(self, '_calls', frozenset(calls))
        object.__setattr__(self, '_invoke', invoke)

    def __getattr__(self, name):
        if name not in self._fields or name not in self._data:
            raise AttributeError('undeclared or missing module input: '+name)
        return self._data[name]

    def __setattr__(self, name, value):
        if name not in self._fields or name == 'cfg':
            raise AttributeError('undeclared module output: '+name)
        self._data[name] = value

    def call(self, module, operation, *args, **kwargs):
        if (module, operation) not in self._calls:
            raise ValueError('undeclared module call: %s.%s' % (module, operation))
        return self._invoke(module, operation, *args, **kwargs)


class ModuleRuntime(object):
    """Routes explicit calls on detached data; the caller decides when to apply."""
    def __init__(self, modules=None, parallel_enabled=True):
        if modules is None:
            from robot.master import state_machine as mission
            from robot.motion import controller as motion
            from robot.camera import observations as camera
            from robot.signs import decisions as signs
            from robot.lane import controller as lane
            from robot.turn import controller as turn
            from robot.uturn import controller as uturn
            from robot.parking import controller as parking
            from robot.obstacle import controller as obstacle
            modules = dict(mission=mission, motion=motion, camera=camera, signs=signs,
                           lane=lane, turn=turn, uturn=uturn, parking=parking,
                           obstacle=obstacle)
            if parallel_enabled:
                from robot.parallel_parking import controller as parallel_parking
                modules['parallel_parking'] = parallel_parking
        self.modules = dict(modules)
        self.owners = {}
        self.overrides = {}
        for owner, module in self.modules.items():
            for operation in module.OPERATIONS:
                if operation in self.owners:
                    raise ValueError('duplicate module operation: '+operation)
                self.owners[operation] = owner

    def execute(self, module, operation, snapshot, *args, **kwargs):
        data = copy_containers(snapshot)
        outputs = set()

        def invoke(owner, name, *values, **options):
            implementation = self.modules.get(owner)
            if implementation is None or name not in implementation.OPERATIONS:
                raise ValueError('unknown module operation: %s.%s' % (owner, name))
            outputs.update(implementation.FIELDS)
            if name in self.overrides:
                return self.overrides[name](*values, **options)
            ctx = ModuleContext(data, implementation.FIELDS, implementation.CALLS, invoke)
            return getattr(implementation, name)(ctx, *values, **options)

        value = invoke(module, operation, *args, **kwargs)
        updates = dict((key, data[key]) for key in outputs if key != 'cfg' and key in data)
        return ModuleResult(value, updates)
