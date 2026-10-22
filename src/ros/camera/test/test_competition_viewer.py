"""Read-only HTTP boundary and source-age tests; no ROS node is started."""
import imp
import json
import os
import sys
import types
import unittest

from vision_test_support import preserve_modules


class ViewerTests(unittest.TestCase):
    def setUp(self):
        self.addCleanup(preserve_modules())
        import rospy
        stub = types.ModuleType('rospy')
        stub.__dict__.update(rospy.__dict__)
        sys.modules['rospy'] = stub
        path = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'ros_flask_viewer.py')
        self.viewer = imp.load_source('competition_viewer_test', path)
        self.subscriptions = []
        def subscribe(topic, *args, **kwargs):
            sub = types.ModuleType('subscription')
            sub.topic, sub.closed = topic, False
            def unregister():
                sub.closed = True
            sub.unregister = unregister
            self.subscriptions.append(sub)
            return sub
        self.viewer.rospy.Subscriber = subscribe
        self.viewer.rospy.Time = type('Clock', (), {
            'now': staticmethod(lambda: type('Stamp', (), {'to_sec': lambda self: 10.0})())})
        self.client = self.viewer.app.test_client()

    def test_only_requested_stream_is_subscribed_and_idle_stream_expires(self):
        self.client.get('/status')
        self.assertEqual(self.subscriptions, [])
        self.client.get('/frame/front_raw')
        self.client.get('/frame/front_raw')
        self.assertEqual(len(self.subscriptions), 1)
        self.assertEqual(self.subscriptions[0].topic, '/debug/front_raw')
        self.viewer.requested_at['front_raw'] -= 10
        self.viewer.expire_streams(None)
        self.assertTrue(self.subscriptions[0].closed)
        self.assertNotIn('front_raw', self.viewer.subscriptions)
        self.client.get('/frame/front_raw')
        self.assertEqual(len(self.subscriptions), 2)

    def test_unwatched_images_are_not_converted(self):
        self.viewer.make_callback('front_raw')(None)
        self.assertEqual(self.viewer.frames, {})

    def test_stream_keeps_subscription_and_rejects_unknown_topic(self):
        self.assertEqual(self.client.get('/stream/unknown').status_code, 404)
        self.viewer.rospy.is_shutdown = lambda: False
        self.viewer.frames['front_raw'] = (9.5, b'jpeg')
        generator = self.viewer.generate_stream('front_raw')
        self.assertIn(b'jpeg', next(generator))
        self.assertIn('front_raw', self.viewer.subscriptions)
        generator.close()

    def test_frame_requires_fresh_source_and_known_topic(self):
        self.assertEqual(self.client.get('/frame/unknown').status_code, 404)
        self.assertEqual(self.client.get('/frame/front_raw').status_code, 503)
        self.viewer.frames['front_raw'] = (8.0, b'jpeg')
        self.assertEqual(self.client.get('/frame/front_raw').status_code, 503)
        self.viewer.frames['front_raw'] = (11.0, b'jpeg')
        self.assertEqual(self.client.get('/frame/front_raw').status_code, 503)
        self.viewer.frames['front_raw'] = (9.5, b'jpeg')
        response = self.client.get('/frame/front_raw')
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.headers['Cache-Control'], 'no-store')

    def test_status_reports_missing_and_stale_controller(self):
        payload = json.loads(self.client.get('/status').data)
        self.assertFalse(payload['controller_fresh'])
        self.assertFalse(payload['streams']['rear']['fresh'])
        self.viewer.controller.update(stamp=9.5, state='LANE')
        self.assertTrue(json.loads(self.client.get('/status').data)['controller_fresh'])
        self.viewer.controller['stamp'] = 8.0
        self.assertFalse(json.loads(self.client.get('/status').data)['controller_fresh'])

    def test_http_has_no_mutation_routes(self):
        for rule in self.viewer.app.url_map.iter_rules():
            self.assertFalse(set(rule.methods) & set(['POST', 'PUT', 'PATCH', 'DELETE']))
        self.assertEqual(self.client.post('/').status_code, 405)


if __name__ == '__main__':
    unittest.main()
