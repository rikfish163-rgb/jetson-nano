"""Python 2-compatible single planning worker; no ROS dependencies."""
import threading


class _SimpleFuture(object):
    def __init__(self):
        self._event = threading.Event()
        self._result = None
        self._exception = None

    def _set_result(self, result):
        self._result = result
        self._event.set()

    def _set_exception(self, exc):
        self._exception = exc
        self._event.set()

    def done(self):
        return self._event.is_set()

    def result(self, timeout=None):
        self._event.wait(timeout)
        if not self.done():
            raise RuntimeError('future timed out')
        if self._exception is not None:
            raise self._exception
        return self._result


class _SingleJobExecutor(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._future = None
        self._closed = False

    def submit(self, fn, *args, **kwargs):
        with self._lock:
            if self._closed:
                raise RuntimeError('cannot schedule new futures after shutdown')
            if self._future is not None and not self._future.done():
                raise RuntimeError('single-job executor is busy')
            future = _SimpleFuture()
            self._future = future
            worker = threading.Thread(target=self._run, args=(future, fn, args, kwargs))
            worker.start()
            return future

    @staticmethod
    def _run(future, fn, args, kwargs):
        try:
            future._set_result(fn(*args, **kwargs))
        except BaseException as exc:
            future._set_exception(exc)

    def shutdown(self, wait=True):
        with self._lock:
            self._closed = True
            future = self._future
        if wait and future is not None:
            future._event.wait()

