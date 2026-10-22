"""Restore import-time test doubles so discovery does not corrupt later tests."""
import sys


def preserve_modules():
    names = ('rospy','cv2','yaml','cv_bridge','sensor_msgs','sensor_msgs.msg',
             'std_msgs','std_msgs.msg','nav_msgs','nav_msgs.msg','parking_rear_metric')
    saved = {name: sys.modules.get(name) for name in names}
    def restore():
        for name, value in saved.items():
            if value is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = value
    return restore
