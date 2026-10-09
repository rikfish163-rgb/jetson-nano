"""White boundaries must no longer steer UTURN blue alignment."""
import unittest
from test_uturn_rear_sequence import RearUturnTests


class WhiteFallbackTests(unittest.TestCase):
    def test_right_boundary_cannot_replace_missing_blue(self):
        fixture=RearUturnTests('test_white_lane_alone_cannot_align_uturn')
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        fixture.align()
        for t in (2,2.1,2.3,2.6):
            fixture.ground(t,[],'front')
            fixture.c.observe_lane([(.5,-.3),(.7,-.25),(.9,-.2)],.9,t)
            self.assertEqual(fixture.c.tick(t),(0,0))
        self.assertEqual(fixture.c.uturn['phase'],'ALIGN_FIRST')


if __name__=='__main__': unittest.main()
