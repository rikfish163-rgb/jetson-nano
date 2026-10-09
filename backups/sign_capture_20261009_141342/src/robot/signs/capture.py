"""Lossless evidence from the actual inference input, never a later camera frame."""
import json
import os
import cv2


def save_capture(directory, frame, crop, info):
    if not os.path.isdir(directory):
        os.makedirs(directory)
    label = info['label'] or ('NO_CANDIDATE' if crop is None else 'REJECTED')
    prefix = os.path.join(directory, '%.9f_%s' % (info['stamp'], label))
    for suffix, pixels in (('_frame.png', frame), ('_crop.png', crop)):
        if pixels is None:
            continue
        if not cv2.imwrite(prefix+suffix, pixels):
            raise IOError('cannot save '+prefix+suffix)
    with open(prefix+'.json', 'w') as stream:
        json.dump(info, stream, sort_keys=True, indent=2, allow_nan=False)
    return prefix
