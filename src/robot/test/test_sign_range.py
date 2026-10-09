import unittest
from robot.signs.range import SignRange

class SignRangeTests(unittest.TestCase):
    def setUp(self):
        cal=dict(image_width=640,image_height=360,
            camera_matrix=dict(data=[400,0,320,0,400,180,0,0,1]),
            distortion_coefficients=dict(data=[0,0,0,0,0]))
        self.f=SignRange(cal,.195,.6)

    def test_far_near_and_boundary(self):
        for w,allowed in ((65,False),(130,True),(156,True)):
            ok,depth,reason=self.f.evaluate((200,100,w,100),(360,640,3))
            self.assertEqual(ok,allowed)
            self.assertAlmostEqual(depth,400*.195/w)

    def test_resolution_scaling(self):
        ok,depth,_=self.f.evaluate((100,50,65,50),(180,320,3))
        self.assertTrue(ok);self.assertAlmostEqual(depth,.6)

    def test_missing_and_clipped_bounds_fail_closed(self):
        for box in (None,(0,20,200,100),(200,20,440,100)):
            self.assertFalse(self.f.evaluate(box,(360,640,3))[0])
