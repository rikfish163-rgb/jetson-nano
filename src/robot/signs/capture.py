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


class CaptureWriter(object):
    """Bounded background writer; disk I/O never runs in the inference timer."""
    def __init__(self, directory, on_saved=None, on_error=None,
                 max_bytes=128*1024*1024, min_free_bytes=256*1024*1024,
                 queue_size=4, session=None):
        import threading
        try:
            from Queue import Queue
        except ImportError:
            from queue import Queue
        self.directory = directory
        self.on_saved, self.on_error = on_saved, on_error
        self.max_bytes, self.min_free_bytes = max_bytes, min_free_bytes
        self.queue = Queue(maxsize=queue_size)
        self.stop = threading.Event()
        self.disabled = False
        self.bytes_written = 0
        self.session = dict(session or {}, max_bytes=max_bytes,
                            min_free_bytes=min_free_bytes)
        self.worker = threading.Thread(target=self._run, name='sign_capture_writer')
        self.worker.daemon = True
        self.worker.start()

    def submit(self, frame, crop, info):
        import copy
        try:
            from Queue import Full
        except ImportError:
            from queue import Full
        if self.stop.is_set() or self.disabled:
            return False
        # Own the exact inference pixels and metadata before callbacks reuse them.
        item = (frame.copy(), None if crop is None else crop.copy(),
                copy.deepcopy(info))
        try:
            self.queue.put_nowait(item)
            return True
        except Full:
            self._error('sign capture queue full; skipped frame %.9f' % info['stamp'])
            return False

    def close(self):
        self.stop.set()
        self.worker.join(2.0)

    def _error(self, message):
        if self.on_error is not None:
            self.on_error(message)

    def _run(self):
        try:
            from Queue import Empty
        except ImportError:
            from queue import Empty
        while not self.stop.is_set() or not self.queue.empty():
            try:
                frame, crop, info = self.queue.get(timeout=.1)
            except Empty:
                continue
            try:
                if self.disabled:
                    continue
                root = os.path.abspath(self.directory)
                while not os.path.isdir(root):
                    root = os.path.dirname(root)
                stats = os.statvfs(root)
                free = stats.f_bavail * stats.f_frsize
                # Conservative PNG + JSON budget before creating any files.
                estimate = frame.nbytes + (crop.nbytes if crop is not None else 0)
                estimate += len(json.dumps(info, allow_nan=False)) + 65536
                if self.bytes_written + estimate > self.max_bytes:
                    self.disabled = True
                    self._error('sign capture session limit reached; saving paused')
                    continue
                if free - estimate < self.min_free_bytes:
                    self.disabled = True
                    self._error('sign capture low disk space; saving paused')
                    continue
                if not os.path.isdir(self.directory):
                    os.makedirs(self.directory)
                session_path = os.path.join(self.directory, '_session.json')
                if not os.path.exists(session_path):
                    with open(session_path, 'w') as stream:
                        json.dump(self.session, stream, sort_keys=True, indent=2)
                prefix = save_capture(self.directory, frame, crop, info)
                self.bytes_written += sum(os.path.getsize(prefix+suffix)
                    for suffix in ('_frame.png', '_crop.png', '.json')
                    if os.path.exists(prefix+suffix))
                if self.on_saved is not None:
                    self.on_saved(prefix, info)
            except Exception as exc:
                self._error('sign capture failed: ' + str(exc))
            finally:
                self.queue.task_done()
